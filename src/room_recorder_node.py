#!/usr/bin/env python3
"""Records the ceiling camera's RTSP H.264 stream directly to mp4 with no
decode/encode — saves CPU and lets us record at native sensor resolution
without saturating the station PC. Start/stop controlled via the standard
Write service, matching csv_writer_node / video_writer_node."""
import os
import signal
import subprocess

import rospy
from abstract_node import AbstractNode
from foma.srv import Write, WriteRequest, WriteResponse


class RoomRecorderNode(AbstractNode):
    def __init__(self):
        node_name = rospy.get_name().split('/')[-1]
        super().__init__(node_name, f'Room Recorder ({node_name})')

        self.__filename = rospy.get_param('~filename', 'room_video.mp4')
        self.__rtsp_url = rospy.get_param('/ROOM_RTSP_URL')

        self.__proc = None
        self.__folder = ''

        rospy.Service(f'{node_name}/write', Write, self.__set_write)
        rospy.on_shutdown(self.__on_shutdown)

    def __build_pipeline(self, filepath):
        """gst-launch-1.0 -e ... ! mp4mux ! filesink — `-e` forces EOS on
        SIGINT/SIGTERM so mp4mux finalizes the moov atom before exit."""
        return [
            'gst-launch-1.0', '-e',
            'rtspsrc', f'location={self.__rtsp_url}', 'protocols=tcp', 'latency=200',
            '!', 'rtph264depay',
            '!', 'h264parse',
            '!', 'mp4mux',
            '!', 'filesink', f'location={filepath}',
        ]

    def __start(self):
        if self.__proc is not None and self.__proc.poll() is None:
            self.logwarn("Recorder already running; ignoring start.")
            return
        if not os.path.isdir(self.__folder):
            self.logerr(f"Trial folder missing: {self.__folder}")
            return

        filepath = os.path.join(self.__folder, self.__filename)
        cmd = self.__build_pipeline(filepath)
        logpath = os.path.join(self.__folder, 'room_recorder.log')
        logf = open(logpath, 'a')
        self.__proc = subprocess.Popen(
            cmd, stdout=logf, stderr=logf,
            start_new_session=True,    # own process group so SIGINT goes to gst only
            close_fds=True,
        )
        self.loginfo(f"room_recorder pid={self.__proc.pid} → {filepath}")

    def __stop(self):
        if self.__proc is None:
            return
        if self.__proc.poll() is not None:
            self.__proc = None
            return
        self.loginfo("Stopping room_recorder (SIGINT for EOS)...")
        try:
            # gst-launch with -e: SIGINT triggers EOS → mp4mux writes trailer.
            self.__proc.send_signal(signal.SIGINT)
            self.__proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.logwarn("room_recorder did not exit after EOS; killing.")
            self.__proc.kill()
            self.__proc.wait()
        self.__proc = None
        self.loginfo("room_recorder stopped.")

    def __set_write(self, req: WriteRequest):
        if req.msg == 'start':
            self.__folder = req.folder
            self.__start()
        elif req.msg == 'stop':
            self.__stop()
        return WriteResponse(success=True)

    def __on_shutdown(self):
        self.__stop()


if __name__ == '__main__':
    rospy.init_node('room_recorder_node')
    RoomRecorderNode()
    rospy.spin()
