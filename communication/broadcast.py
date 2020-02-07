import rospy
import time
from std_msgs.msg import String

def create_msg(string):
    msg = String()
    
    msg.data = string

    print("Sent: ", msg)
    return msg

def callback(msg):
    data = msg.data
    print("Received: "msg)
    
    time.sleep(1)
    
    if data == "right":
        pub.publish(create_msg("left"))
    elif data == "left":
        pub.publish(create_msg("right"))
    elif data == "done":
        pub.publish(create_msg("count"))
    else:
        print("Invalid message!")      

def broadcast():
    rate = rospy.Rate(20)
    
    while rospy.is_shutdown():
        continue
    
    
    pub.publish(create_msg("count"))
    rate.sleep()

def subsc():
    rospy.spin()
    
pub = rospy.Publisher('command', String, queue_size=1)
sub = rospy.Subscriber('feedback', String, callback)
        
if __name__ == "__main__":
    rospy.init_node("broadcast")
    broadcast()
    subsc()
