#1/usr/bin/env python2

import rospy

import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped

def poster():
    pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=1)
    
    ipose=PoseWithCovarianceStamped()

    ipose.header.stamp=rospy.Time.now()
    ipose.header.frame_id="map"

    ipose.pose.pose.position.x=-11.30141
    ipose.pose.pose.position.y=13.85292625
    ipose.pose.pose.position.z=0

    ipose.pose.pose.orientation.z=-0.91244
    ipose.pose.pose.orientation.w=0.409212
    pub.publish(ipose)





    



if __name__ =='__main__':
    talker()
