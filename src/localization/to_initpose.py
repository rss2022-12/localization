#1/usr/bin/env python2

import rospy

import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped

def poster():
    pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=1)
    rospy.init_node('initpose')
    
    ipose=PoseWithCovarianceStamped()
    ipose.header.frame_id="map"
    ipose.pose.pose.position.x=-15.413
    ipose.pose.pose.position.y=10.18536
    ipose.pose.pose.position.z=0

    ipose.pose.pose.orientation.z=0.39342
    ipose.pose.pose.orientation.w=0.91935
    #ipose.pose.covariance=[0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0685389]
    pub.publish(ipose)





    



if __name__ =='__main__':
    poster()
