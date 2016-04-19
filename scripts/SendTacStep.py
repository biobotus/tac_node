#!/usr/bin/python

# added for ROS
import rospy
from std_msgs.msg import String

data = '{{"module_type": "tac",  "module_id":"{0}", "params": {{"name": "manip", "args": {{"stop": {{"{0}": {2},}}, "set": {{"temperature": {3}, "spin":{4}}}}}}}}}'

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('step', String, queue_size=10)
        rospy.init_node('fake_behavior',anonymous=True)
        # 10hz
        rate = rospy.Rate(10)

        tac_id = '1234'
        stop_param = 'time'
        stop_value = 10

        target_spin = 50
        target_temp = 23.0

        data = data.format(tac_id, stop_param, stop_value, target_temp, target_spin)
        pub.publish(data)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass

