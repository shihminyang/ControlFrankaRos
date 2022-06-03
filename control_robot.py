import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from scipy.spatial.transform import Rotation


class ControlFrankaEmika:
    def __init__(self, topic) -> None:

        self.pub_pose = rospy.Publisher(
            topic['pose'], PoseStamped, queue_size=10)
        # self.pub_stiff = rospy.Publisher(
        #     topic['stiffness'], TwistStamped, queue_size=10)

    def move_end_effector(self, pose, frame_id='panda_link0'):
        """
            Args:
                pose    (list): Position (meter) and
                                euler roll-pitch-yaw (radian)
                                [x, y, z, roll, pitch, yaw]
                frame_id (str): Based frame id of the robot
        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = rospy.Time.now()

        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]

        quant = Rotation.from_euler('zyx', pose[3:]).as_quat()
        pose_msg.pose.orientation = Quaternion(*quant)
        # pose_msg.pose.orientation.x = quant[0]
        # pose_msg.pose.orientation.y = quant[1]
        # pose_msg.pose.orientation.z = quant[2]
        # pose_msg.pose.orientation.w = quant[3]

        self.pub_pose.publish(pose_msg)
        return True


if __name__ == '__main__':
    import numpy as np
    """
    Topic: /equilibrium_pose is used in cartesian impedance controller
    """
    topic = {'pose': "/equilibrium_pose"}
    robot = ControlFrankaEmika(topic=topic)
    pose = [0.05, 0.05, 0.05, 0, np.pi, 0]

    rospy.init_node('control_robot_node')
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        cmd = input("Move (x,y,z,rx,ry,rz): ")
        if cmd == '':
            continue
        cmd = cmd.split()
        m = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}
        pose[m[cmd[0]]] += float(cmd[1])

        robot.move_end_effector(pose)
        r.sleep()
