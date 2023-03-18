#!/usr/bin/python

import readchar
import rospy
import roslaunch
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
import kinova_positional_control.srv as posctrl_srv
from utils import *

def roslaunch_pid():
    """
    Launch the 'arm_controller.launch' file from the 'kinova_pid' package. 
    This function initializes the Kinova PID controller node.
    """
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    pid_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/kinova_pid/launch/arm_controller.launch'])
    pid_launch.start()

def roslaunch_relaxed_ik():
    """
    Launch the 'relaxed_ik.launch' file from the 'relaxed_ik_ros1' package. 
    This function initializes the RelaxedIK node.
    """
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    relaxed_ik_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/relaxed_ik_ros1/launch/relaxed_ik.launch'])
    relaxed_ik_launch.start()
         
# Callback function that updates motionFinished flag
def pid_motion_finished_callback(data):
    """
    Callback function for the '/pid/motion_finished' topic that updates the
    'motionFinished' flag variable.
    """
    global motionFinished
    
    motionFinished = data.data
    
def wait_motion_finished():
    """
    Block code execution until the 'motionFinished' flag is set or the ROS node
    is shutdown.
    """
    global motionFinished
    
    # Allow motion to start
    rospy.sleep(1)

    # Block code execution
    while not motionFinished and not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    
    rospy.init_node('keyboard_ikgoal_driver')

    # Launch files
    roslaunch_pid()
    roslaunch_relaxed_ik()
    rospy.sleep(2)
    
    #Publishers
    ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
    quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)

    # Service
    pid_vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', posctrl_srv.pid_vel_limit)

    # Subscribers
    rospy.Subscriber('/pid/motion_finished', Bool, pid_motion_finished_callback)
        
    # Set 20% velocity
    pid_vel_limit_srv(0.2)

    # Homing position
    print("\nHoming has started...\n")

    # Let the node initialized
    rospy.sleep(2)

    # Home using relaxedIK
    relaxedik_publish([0, 0, 0], [1, 0, 0, 0])

    # Block until the motion is finished
    wait_motion_finished()

    print("\nHoming has finished.\n") 

    pid_vel_limit_srv(1.0)

    print("\nSystem is ready.\n")

    pos_stride = 0.015
    rot_stride = 0.055

    position_r = [0,0,0]
    rotation_r = [1,0,0,0]

    position_l = [0,0,0]
    rotation_l = [1,0,0,0]

    seq = 1
    rate = rospy.Rate(1000)
    
    while not rospy.is_shutdown():

        print("Pos R: {}, Pos L: {}".format(position_r, position_l))

        key = readchar.readkey()
        if key == 'w':
            position_r[0] += pos_stride
        elif key == 'x':
            position_r[0] -= pos_stride
        elif key == 'a':
            position_r[1] += pos_stride
        elif key == 'd':
            position_r[1] -= pos_stride
        elif key == 'q':
            position_r[2] += pos_stride
        elif key == 'z':
            position_r[2] -= pos_stride
        elif key == '1':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] += rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '2':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[0] -= rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '3':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] += rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '4':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[1] -= rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '5':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] += rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '6':
            euler = list(T.euler_from_quaternion(rotation_r))
            euler[2] -= rot_stride
            rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

        elif key == 'i':
            position_l[0] += pos_stride # left
        elif key == 'm':
            position_l[0] -= pos_stride
        elif key == 'j':
            position_l[1] += pos_stride
        elif key == 'l':
            position_l[1] -= pos_stride
        elif key == 'u':
            position_l[2] += pos_stride
        elif key == 'n':
            position_l[2] -= pos_stride
        elif key == '=':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] += rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '-':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[0] -= rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '0':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] += rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '9':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[1] -= rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '8':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] += rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '7':
            euler = list(T.euler_from_quaternion(rotation_l))
            euler[2] -= rot_stride
            rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == 'q':
            q = Bool()
            q.data = True
            quit_pub.publish(q)
        elif key == 'c':
            rospy.signal_shutdown()

        ee_pose_goals = EEPoseGoals()
        pose_r = Pose()
        pose_r.position.x = position_r[0]
        pose_r.position.y = position_r[1]
        pose_r.position.z = position_r[2]

        pose_r.orientation.w = rotation_r[0]
        pose_r.orientation.x = rotation_r[1]
        pose_r.orientation.y = rotation_r[2]
        pose_r.orientation.z = rotation_r[3]

        pose_l = Pose()
        pose_l.position.x = position_l[0]
        pose_l.position.y = position_l[1]
        pose_l.position.z = position_l[2]

        pose_l.orientation.w = rotation_l[0]
        pose_l.orientation.x = rotation_l[1]
        pose_l.orientation.y = rotation_l[2]
        pose_l.orientation.z = rotation_l[3]
        ee_pose_goals.ee_poses.append(pose_r)
        ee_pose_goals.ee_poses.append(pose_l)

        ee_pose_goals.header.seq = seq
        seq += 1
        ee_pose_goals_pub.publish(ee_pose_goals)

        q = Bool()
        q.data = False
        quit_pub.publish(q)
