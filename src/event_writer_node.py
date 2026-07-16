#!/usr/bin/env python3
import rospy
import csv
import os
from abstract_node import AbstractNode
from foma.srv import Write, WriteRequest, WriteResponse
from foma.msg import TrialEvent


class EventWriterNode(AbstractNode):
    """Subscribes to a TrialEvent topic and appends each event to a CSV file
    during an active trial. Start/stop controlled via Write service, matching
    the pattern used by csv_writer_node / video_writer_node."""

    def __init__(self):
        node_name = rospy.get_name().split('/')[-1]
        super().__init__(node_name, f'Event Writer Node ({node_name})')

        self.__topic    = rospy.get_param('~topic')
        self.__filename = rospy.get_param('~filename')
        self.__fields   = rospy.get_param('~fields', ['timestamp', 'event_type', 'details'])

        self.__write       = False
        self.__csv_file    = None
        self.__csv_writer  = None
        self.__folder      = ""
        self.__subscriber  = None

        rospy.Service(f'{node_name}/write', Write, self.__set_write)
        rospy.on_shutdown(self.__on_shutdown)

    def __start_trial(self):
        if not os.path.exists(self.__folder):
            self.logerr(f"Trial folder does not exist: {self.__folder}")
            return

        filepath = os.path.join(self.__folder, self.__filename)
        self.__csv_file = open(filepath, 'a', newline='')
        self.__csv_writer = csv.writer(self.__csv_file)

        if os.stat(filepath).st_size == 0:
            self.__csv_writer.writerow(self.__fields)
            self.__csv_file.flush()

        self.loginfo(f"Created event log at {filepath}")
        self.__subscriber = rospy.Subscriber(self.__topic, TrialEvent, self.__event_callback, queue_size=100)
        self.loginfo(f"Subscribed to {self.__topic}")

    def __stop_trial(self):
        if self.__subscriber is not None:
            self.__subscriber.unregister()
            self.__subscriber = None
            self.loginfo(f"Unsubscribed from {self.__topic}")

        rospy.sleep(0.1)

        if self.__csv_file is not None:
            self.__csv_file.close()
            self.__csv_file = None
            self.__csv_writer = None
            self.loginfo("Closed event log file.")

    def __event_callback(self, event: TrialEvent):
        if not self.__write or self.__csv_writer is None:
            return
        stamp = event.stamp.to_sec() if event.stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        self.__csv_writer.writerow([stamp, event.event_type, event.details])
        self.__csv_file.flush()
        self.loginfo(f"Event logged: {event.event_type} - {event.details}")

    def __set_write(self, write: WriteRequest):
        if write.msg == "start":
            self.__folder = write.folder
            self.__start_trial()
            self.__write = True
        elif write.msg == "stop" and self.__write:
            self.__write = False
            self.__stop_trial()
        return WriteResponse(success=True)

    def __on_shutdown(self):
        self.loginfo("Shutting down EventWriterNode...")
        self.__write = False
        self.__stop_trial()


if __name__ == "__main__":
    rospy.init_node('event_writer_node')
    EventWriterNode()
    rospy.spin()
