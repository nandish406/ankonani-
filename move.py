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

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.msg import Model
from pkg_vb_sim.srv import vacuumGripper

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,callback=self.camera,queue_size=10)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.box=LogicalCameraImage()
        self.dif=Model()
        self.ur5_pose_1 = geometry_msgs.msg.Pose()
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')
        self.box_length = 0.15               # Length of the Package
        self.vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = self.vacuum_gripper_width + (self.box_length/2)  # 0.19
        # Teams may use this info in Tasks

        self.home_pose = geometry_msgs.msg.Pose()
        self.home_pose.position.x = -0.8
        self.home_pose.position.y = 0
        self.home_pose.position.z = 1 + self.vacuum_gripper_width + (self.box_length/2)
        # This to keep EE parallel to Ground Plane
        self.home_pose.orientation.x = -0.5
        self.home_pose.orientation.y = -0.5
        self.home_pose.orientation.z = 0.5
        self.home_pose.orientation.w = 0.5


    def handle_conveyor(self,power):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            move = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', self.pr)
            response= move(power)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def handle_vg(self,act):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            vg = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            response= vg(act)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +
                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )
            self.ur5_pose_1.position.x=trans.transform.translation.x
            self.ur5_pose_1.position.y=trans.transform.translation.y
            self.ur5_pose_1.position.z=trans.transform.translation.z
            self.ur5_pose_1.orientation.x =trans.transform.rotation.x
            self.ur5_pose_1.orientation.y =trans.transform.rotation.y
            self.ur5_pose_1.orientation.z =trans.transform.rotation.z
            self.ur5_pose_1.orientation.w =trans.transform.rotation.w
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def camera(self,data):
        self.box=data

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def joint_angle_transition(self,arg_joint_angles):
        list1=self._group.get_current_joint_values()
        list2=[]
        zip_object= zip(list1,arg_joint_angles)
        for list1_i,arg_joint_angles_i in zip_object:
            list2.append(list1_i-arg_joint_angles_i)
        self.set_joint_angles(list2)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():

    ur5 = CartesianPath()
    ur5.go_to_pose(ur5.home_pose)
    ur5.func_tf_print('world','logical_camera_2_frame')
    ref=ur5.ur5_pose_1.position.x

    goal_pose= geometry_msgs.msg.Pose()
    bin_pose=[geometry_msgs.msg.Pose() for i in range(3)]
    #Destination Bin poses 

    bin_pose[0].position.x=0.0280518015705
    bin_pose[0].position.y=0.553104827039
    bin_pose[0].position.z=1.11405642079
    bin_pose[0].orientation.x=-1.08034359204e-05
    bin_pose[0].orientation.y=-0.811637216272
    bin_pose[0].orientation.z=0.584161730391
    bin_pose[0].orientation.w=0.000319050479666

    bin_pose[2].position.x=0.0388507599852
    bin_pose[2].position.y=-0.578291991027
    bin_pose[2].position.z= 1.16781935701
    bin_pose[2].orientation.x=-0.00632122158936
    bin_pose[2].orientation.y=0.621946567178
    bin_pose[2].orientation.z= -0.77631458142
    bin_pose[2].orientation.w=0.102362983581

    bin_pose[1].position.x=0.817188461183
    bin_pose[1].position.y=0.109556760312
    bin_pose[1].position.z=0.944458336299
    bin_pose[1].orientation.x=0.000241460168283
    bin_pose[1].orientation.y=-0.999999969712
    bin_pose[1].orientation.z=1.20678298204e-08
    bin_pose[1].orientation.w=4.76741946812e-05

    joint_angle=[[],[],[]]
    joint_angle[0]=[1.517,0,0,0,0,0]
    joint_angle[1]=[3.142,0,0,0,0,0]
    joint_angle[2]=[-1.517,0,0,0,0,0]

    while not rospy.is_shutdown():
        if(ur5.box.models):
            for mod in range ur5.box.models:
                try:
                    if(mod[n].type[0]=='p'):
                        if(abs(mod[n].pose.position.y)<=1e-2):
                            key_frame='logical_camera_2_'+mod[n].type+'_frame'
                            ur5.func_tf_print('world',key_frame)
                            goal_pose.position.x=ur5.ur5_pose_1.position.x-ref
                            print(mod[n].pose)
                            print(goal_pose)
                            ur5.ee_cartesian_translation(goal_pose.position.x,0,0)
                            ur5.handle_vg(True)
                            ur5.ee_cartesian_translation(0,0,(ur5.box_length+0.2))
                            key=mod[n].type[-1]
                            int_key=ord(key)-ord('0')-1
                            ur5.joint_angle_transition(joint_angle[int_key])
                            ur5.handle_vg(False)
                            ur5.go_to_pose(ur5.home_pose)
                except:
                    print("index error")

    del ur5


if __name__ == '__main__':
    main()
