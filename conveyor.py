#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg  
import actionlib
import tf2_ros
import tf2_msgs.msg

from pkg_t2_examples.msg import start

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.msg import Model


class Conveyor:

    # Constructor
    def __init__(self):

        rospy.init_node('node_conveyor', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,callback=self.camera,queue_size=10)
        rospy.Subscriber('conveyor_start',start,callback=self.con_start,queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.pr=conveyorBeltPowerMsg()
        self.box=LogicalCameraImage()
        self.dif=Model()
        self.ur5_pose_1 = geometry_msgs.msg.Pose()
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')
        self.box_length = 0.15               # Length of the Package
        self.vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = self.vacuum_gripper_width + (self.box_length/2)  # 0.19
        self.message=start()
        self.message=False
        # Teams may use this info in Tasks

    def handle_conveyor(self,power):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            move = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', self.pr)
            response= move(power)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def camera(self,data):
        self.box=data

    def con_start(self,data):
        self.message=data

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Conveyor Deleted." + '\033[0m')

def main():

    ur5 = Conveyor() 
    while(not ur5.message):
        pass
    ur5.handle_conveyor(99)
    flag=True


    while not rospy.is_shutdown():
        flag_2=True
        if (len(ur5.box.models)>0):
            for mod in ur5.box.models:
                try:
                    if(mod.type[0]=='p'):
                        key=mod.type[-1]
                        int_key=ord(key)-ord('0')-1
                        if(abs(mod.pose.position.y)<=2e-2):
                            ur5.handle_conveyor(0)
                            flag=False
                        elif (int_key==0 and flag):
                            ur5.handle_conveyor(40)
                        else:
                            ur5.handle_conveyor(15)
                except:
                    print("Index error")

    del ur5


if __name__ == '__main__':
    main()
