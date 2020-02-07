import rospy
import sys
import os
from my_msgs.msg import Velocities

class Velocities:
    def __init__(self):
        self.left_wheel = 0
        self.right_wheel = 0

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

def create_msg(left_wheel, right_wheel):
    msg = Velocities()
    
    msg.left_wheel = left_wheel
    msg.right_wheel = right_wheel

    print("Sent: {} left, {} right".format(msg.left_wheel, msg.right_wheel))
    return msg


def broadcast():
    pub = rospy.Publisher('command', Velocities, queue_size=1)
    rate = rospy.Rate(20)
    
    while rospy.is_shutdown():
        continue
    
    right_val = 0
    left_val = 0
    
    while not rospy.is_shutdown():
    # while True:
        keyp = getch()
        
        print(keyp)
        
        if keyp == 'w':
            right_val += 0.5
            left_val += 0.5
        elif keyp == 'a':
            right_val -= 0.25
            left_val += 0.25
            if (right_val < 0):
                right_val = 0
        elif keyp == 's':
            if ((left_val < 0) and (right_val > 0)):
                left_val = 0
                
            if ((left_val > 0) and (right_val < 0)):
                right_val = 0

            right_val -= 0.5
            left_val -= 0.5
        elif keyp == 'd':
            right_val += 0.25
            left_val -= 0.25
            if (left_val < 0):
                left_val = 0
        elif keyp == ' ':
            right_val = 0
            left_val = 0
        elif keyp == 'p':
            break
        else:
            pass
        
        # create_msg(left_val, right_val)
        pub.publish(create_msg(left_val, right_val))
    
if __name__ == "__main__":
    rospy.init_node("broadcast")
    broadcast()