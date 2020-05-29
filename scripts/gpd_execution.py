# -*- coding: utf-8 -*-

"""
Code for executing GPD Grasps on a Panda Robot 
Author: Snehal
"""

# Imports for MoveIt Gazebo Integration 
import sys
import rospy
import rosservice
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

# Imports for GPD 
from gpd_ros import msg
from autolab_core import RigidTransform

# Transform Listener
import tf

class grasp_gpd():
    def __init__(self): 
	#Subscribe to selected grasps from gpd
        rospy.Subscriber("/detect_grasps/clustered_grasps", 
			   msg.GraspConfigList , 
		           self.callback, 
		           queue_size=10)
        self.flag = True
        
    def callback(self, grasp_msg):
        """
	grasp_msg: position,orientation,grasp width 
		    of selected grasps
	Imp: grasps are arranged in descending order of score
        """
	# Flag: To make the callback execute only one
        if self.flag == True:
		# grasp message values 
		p = grasp_msg.grasps[0].position
		ap = grasp_msg.grasps[0].approach
		b = grasp_msg.grasps[0].binormal 
		ax = grasp_msg.grasps[0].axis
                self.width = np.float(grasp_msg.grasps[0].width.data)
		# approach offset
                off = 0.05
		# Approach Grasp Position 
		self.position= np.array([p.x - off, p.y , (p.z -0.060)])
                # Grasp Position 
                self.position1 = np.array([p.x - off, p.y , p.z]) 
		# Orientation
		self.orientation = np.array([[ap.x,b.x,ax.x],
					     [ap.y,b.y,ax.y],
					     [ap.z,b.z,ax.z]])
		self.flag = False
    

    def processing(self):
	"""
        Transformation: Link8 -> Grasp -> Camera -> Link0
	"""
        # Transforms using Listener
        listener = tf.TransformListener()
        while True:
           try:
               (tcr,rcr) = listener.lookupTransform('/panda_link0',
							 '/color', 
							 rospy.Time(0))
               (tref,rref) = listener.lookupTransform('/panda_link0',
							   '/panda_link8', 
							   rospy.Time(0))
           except (tf.LookupException, 
		   tf.ConnectivityException, 
                   tf.ExtrapolationException):
               continue
           break
	
	# Ready Position
	qref = [rref[3],rref[0],rref[1],rref[2]]
	self.ready_pose = RigidTransform(rotation = qref , 
					translation = tref,
                                 	from_frame='link8', 
					to_frame='link0').pose_msg

	# Transformation from Camera to Robot link 0 (base) Frame
        qcr = [rcr[3],rcr[0],rcr[1],rcr[2]]
        T_camera_robot = RigidTransform(rotation = qcr , 
					     translation = tcr,
                                 	     from_frame='camera', 
					     to_frame='link0')

        # Transformation from Approach Grasp Frame to Camera Frame 
        T_gripper_camera = RigidTransform(rotation = self.orientation, 
						translation = self.position, 
                                                from_frame='grasp', 
				                to_frame='camera')

        # Transformation from Grasp Frame to Camera Frame 
        T_gripper_camera1 = RigidTransform(rotation = self.orientation, 
					        translation = self.position1, 
                                   	        from_frame='grasp', 
                                                to_frame='camera')

	# Transformation from End-Effector/Tool Frame (Panda Link 8) to Grasp Frame 
	### Rotation: YZX = [90,45,0]
	eef_rot = [0.6532815, 0.2705981, 0.6532815, 0.2705981]
        eef_pos = [-0.050, -0.030,-0.01]
        T_eef_gripper = RigidTransform(rotation = eef_rot,
					      translation = eef_pos,                       
                                   	      from_frame='eef', 
					      to_frame='grasp')

	# Transformation from End-Effector/Tool Frame (Panda Link 8) to Robot Base Frame
	# Approach
        self.T_gripper_robot = T_camera_robot * T_gripper_camera * T_eef_gripper
	# Grasp
        self.T_gripper_robot1 = T_camera_robot * T_gripper_camera1 * T_eef_gripper
        
    def pose_set(self,transform):
	# Set pose of Arm using Moveit Commander
        pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.x = transform.quaternion[1]
        pose_goal.orientation.y = transform.quaternion[2]
        pose_goal.orientation.z = transform.quaternion[3]
        pose_goal.orientation.w = transform.quaternion[0]
        pose_goal.position.x = transform.translation[0] 
        pose_goal.position.y = transform.translation[1]
        pose_goal.position.z = transform.translation[2]
	return pose_goal

    def open_gripper(self):
	# open gripper to required width
	a = self.width/2.0
	openG = min(0.04,a+0.02)
	self.group_h.set_joint_value_target([openG,openG])
	self.group_h.go(wait=True)
	self.group_h.stop()
	self.group_h.clear_pose_targets()

    def close_gripper(self):
	# close gripper 
        self.group_h.set_joint_value_target([0.00,0.00])
	self.group_h.go(wait=True)
	self.group_h.stop()
	self.group_h.clear_pose_targets()

    def group_action(self):
	# pose setting and clearning
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets() 

    def pre_grasp_action(self):
	# set arm to pre grasp position
	approach_position = self.pose_set(self.T_gripper_robot)
        self.group.set_pose_target(approach_position)
	self.group_action()

    def grasping_action(self):
	# set arm to grasp position
	grasping_position = self.pose_set(self.T_gripper_robot1)
        self.group.set_pose_target(grasping_position)
	self.group_action()

    def post_grasp_action(self):
	# set arm to post grasp position
	grasping_position = self.pose_set(self.T_gripper_robot1)
	grasping_position.position.z = grasping_position.position.z + 0.2 
        self.group.set_pose_target(grasping_position)
	self.group_action()

    def back_to_ready(self):
	# set to ready state
	self.group.set_pose_target(self.ready_pose)
	self.group_action()

    def moveit_setup(self):  
	# Moveit setup for planning
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group_h = moveit_commander.MoveGroupCommander("hand")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = self.group.get_planning_frame()  

    def execution(self): 
	# Final execution sequence
	self.moveit_setup()
        print "Moving towards Object"
	self.open_gripper()
	rospy.sleep(2)
        self.pre_grasp_action()
	rospy.sleep(5) 
        print "Grasping Object"
        self.grasping_action()
        rospy.sleep(5)
	self.close_gripper()
        rospy.sleep(5)
	print "Moving away with Object"
	self.post_grasp_action()	
        rospy.sleep(15) 
	# This motion could be jerky 
	print "Returning to ready position!!"
	self.back_to_ready() 
        
def main(args):       
     print "==================GPD ROS EXECUTION STARTED====================="
     rospy.init_node('simulation_gpd', anonymous=True)
     gp = grasp_gpd()
     # Buffer for subscriber
     rospy.sleep(5) 
     gp.processing()
     gp.execution()
     print "=====================EXECUTION HAS ENDED============================"
     rospy.spin()

if __name__ == '__main__':
     main(sys.argv)

        

   


