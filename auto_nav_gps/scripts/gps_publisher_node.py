#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import random

def gps_publisher():
    pub = rospy.Publisher('/fix', NavSatFix, queue_size=10)
    rospy.init_node('gps_publisher_node', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz

    lat = 25.0330   # 初始緯度（台北101）
    lon = 121.5654  # 初始經度
    alt = 10.0      # 高度

    while not rospy.is_shutdown():
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "gps_link"

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # 模擬微小移動
        msg.latitude = lat + random.uniform(-0.00001, 0.00001)
        msg.longitude = lon + random.uniform(-0.00001, 0.00001)
        msg.altitude = alt + random.uniform(-0.1, 0.1)

        msg.position_covariance = [0.0]*9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
