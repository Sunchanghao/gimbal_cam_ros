#!/usr/bin/env python
import roslib

roslib.load_manifest('dji_gimbal')
import rospy
import tf
#from geometry_msgs.msg import Twist
#from hector_uav_msgs.srv import EnableMotors
#from std_msgs.msg import Header

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

pitch_ang = 0
def rcv(imu_data):
    global pitch_ang
    angle = Float64()
    quaternion = (
        imu_data.orientation.x,
        imu_data.orientation.y,
        imu_data.orientation.z,
        imu_data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    angle.data = -euler[0]
    roll.publish(angle)
    angle.data = -euler[1]+pitch_ang
    pitch.publish(angle)


def pitch_rcv(data):
    global pitch_ang
    pitch_ang = data.data

if __name__ == "__main__":
    rospy.init_node('stab')
    yaw = rospy.Publisher('dji_gimbal/yaw_position_controller/command', Float64, queue_size=10)
    pitch = rospy.Publisher('dji_gimbal/pitch_position_controller/command', Float64, queue_size=10)
    roll = rospy.Publisher('dji_gimbal/roll_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("raw_imu", Imu, rcv)
    rospy.Subscriber("pitch", Float64, pitch_rcv)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print e
        pass
    finally:

        angle = Float64()
        angle.data=0
        yaw.publish(angle)
        pitch.publish(angle)
        roll.publish(angle)