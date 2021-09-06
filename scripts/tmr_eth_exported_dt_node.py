#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import rospy
from std_msgs.msg import String

try:
    from tmr_listener.msg import TMREthernet
except ImportError:
    print('TMREthernet msg not generated, if you are not using exported data table,'
          'then you can ignore this error message completely. See README.md')
    exit(0)

pub = rospy.Publisher(
    '/tmr_eth_slave/exported_data_table', TMREthernet, queue_size=1)


def eth_dtable_cb(raw_data):
    msg = TMREthernet()
    result = json.loads(raw_data.data)
    collapsed = {elem['Item']: elem['Value'] for elem in result}

    try:
        for elem in TMREthernet.__slots__:
            msg.__setattr__(elem, collapsed[elem])

        pub.publish(msg)
    except Exception as e:
        rospy.logerr(
            'Mismatch between server and exported data table: {}, aborting'.format(elem))
        rospy.signal_shutdown(e)


if __name__ == '__main__':
    rospy.init_node('tmr_eth_export_dt', anonymous=True)
    rospy.Subscriber('/tmr_eth_slave/raw_data_table',
                     String, eth_dtable_cb)
    rospy.loginfo('starting node')
    rospy.spin()
