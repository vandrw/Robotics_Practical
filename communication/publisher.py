import rospy
from my_msgs.msg import Velocities

def create_msg(wheels):
    msg = Velocities()
    if wheels == "both":
        msg.left_vel = 10        # PLACEHOLDERS!
        msg.right_vel = 10
    elif wheels == "left":
        msg.left_vel = 10
        msg.right_vel = 0
    elif wheels == "right":
        msg.left_vel = 0
        msg.right_vel = 10
    else:
        msg.left_vel = 0
        msg.right_vel = 0

    return msg

def publisher_broadcast():
    pub_left = rospy.Publisher('cool_board', Velocities, queue_size=1)
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        pub.publish(create_msg("both"))
        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node("publisher_broadcast")
    publisher_broadcast()
