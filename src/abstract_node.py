import rospy
from foma.srv import String, StringRequest, StringResponse

class AbstractNode:
    def __init__(self, node_name, node_repr, system_on = False):
        self._node_name = node_name
        self._node_repr = node_repr
        self._system_on = system_on
        self._mode = None
        rospy.Service(f'{node_name}/set_mode', String, self.set_mode)
    
    def set_mode(self, msg:StringRequest):
        self._mode = msg.msg
        self.loginfo(f"Mode set to: {self._mode}")
        return StringResponse(result=True)

    def loginfo(self, msg):
        rospy.loginfo(f"{self._node_repr}: {msg}")

    def logwarn(self, msg):
        rospy.logwarn(f"{self._node_repr}: {msg}")

    def logerr(self, msg):
        rospy.logerr(f"{self._node_repr}: {msg}")
    