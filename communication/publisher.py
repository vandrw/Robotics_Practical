import rospy
from my_msgs.msg import Velocities

def create_msg(wheels, speed):
    msg = Velocities()
    if wheels == "both":
        msg.left_wheel = speed
        msg.right_wheel = speed
    elif wheels == "left":
        msg.left_wheel = speed
        msg.right_wheel = 0
    elif wheels == "right":
        msg.left_wheel = 0
        msg.right_wheel = speed
    else:
        msg.left_wheel = 0
        msg.right_wheel = 0

    return msg

def publisher_broadcast():
    pub_left = rospy.Publisher('cool_board', Velocities, queue_size=1)
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        pub.publish(create_msg("both", 1))
        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("publisher_broadcast")
    publisher_broadcast()
