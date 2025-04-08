import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from foma.srv import Check, CheckRequest, CheckResponse

class AbstractNode:
    def __init__(self, node_name, node_repr, system_on = False):
        self._node_name = node_name
        self._node_repr = node_repr
        self._system_on = system_on
        self._system_toggle_service = rospy.Service('{}/system_toggle'.format(node_name), SetBool, self.system_toggle)
        self._system_check_service = rospy.Service('{}/system_check'.format(node_name), Check, self.system_toggle)

    def system_toggle(self, request:SetBoolRequest):
        self._system_on = request.data
        return SetBoolResponse(success = True, message = "{} turned {}.".format(self._node_repr ,self._system_on))
    
    def system_check(self, check: CheckRequest):
        return CheckResponse()

    def loginfo(self, msg):
        rospy.loginfo(f"{self._node_repr}:{msg}")

    def logwarn(self, msg):
        rospy.logwarn(f"{self._node_repr}:{msg}")

    def logerr(self, msg):
        rospy.logerr(f"{self._node_repr}:{msg}")
    