import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class MySubscriber:
    def __init__(self):
        rospy.Subscriber('/mystery/output2', Float32, self.callback)

    def callback(self, msg):
        units = msg.units
        value = msg.value
        rospy.loginfo("I got " + str(units) + " units and " + str(value) + " data")

if __name__ == '__main__':
    rospy.init_node('MySubscriber')
    myNode = MySubscriber()
    rospy.spin()
