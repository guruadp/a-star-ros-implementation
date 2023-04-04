import rospy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
import time

def get_input():
    x = int(input("Enter start X: "))
    y = int(input("Enter start Y: "))
    theta = int(input("Enter start theta: "))
    return (x, y, theta)

def talker():
    msg = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('robot_talker', anonymous=True)

    while not rospy.is_shutdown():
        start_pos = get_input()
        msg.angular.z = start_pos[2]
        msg.linear.x = start_pos[0]
        msg.linear.y = start_pos[1]
        pub.publish(msg)
        time.sleep(0.1)
        
# def pub_coordinates():
#     # initializing node
#     rospy.init_node("start_coordinate_pub_node")

#     # defining publisher with topic name, message type
#     pub = rospy.Publisher("start_coordinate", Int16MultiArray, queue_size=10)

#     # to publish 5 message per second
#     rate = rospy.Rate(5) 
    
#     while not rospy.is_shutdown():
#         # get_input()
#         start_pos = get_input()
#         start_node = Int16MultiArray() 
#         start_node.data = start_pos
#         # pub.publish(start_pos)
#         pub.publish(start_node)
#         # pub.publish("Y: ",start_pos[1])
#         # pub.publish("theta: ",start_pos[2])
#         rate.sleep()

if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass