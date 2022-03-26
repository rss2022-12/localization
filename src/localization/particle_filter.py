#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import tf

class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.lidar_callback, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odom_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
        # Initialize the modelsPublish a transformation frame between the map
        # and the particle_filter_frame.
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.numparticles=50
        self.particles=None
        self.precompute_particles()
        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        
        def precompute_particles(self):
            self.particles = np.zeros((self.numparticles, 3))
         

            
            
            
        def lidar_callback(self,particles,lidarscan):
            #Whenever you get sensor data use the sensor model to 
            #compute the particle probabilities. Then resample the
            #particles based on these probabilities
            
            probs=self.sensor_model.evaluate(particles,lidarscan.ranges)
            
            num_particles=len(probs)
            ranges=np.arange(num_particles)            
            new_particles=np.random.choice(ranges,probs,p=probs)
            self.particles=particles[new_particles]
            
            # #return mean of particles
            # self.x_mean=np.mean(self.particles[:,0])
            # self.y_mean=np.mean(self.particles[:,1])
            # self.theta_mean=np.arctan2(np.sin(self.particles[:,2]),np.cos(self.particles[:,2]))
            
            
            
        def odom_callback(self,particles,odometry):
            # Whenever you get odometry data use the motion model 
            # to update the particle positions
            x=odometry.twist.linear.x
            y=odometry.twist.linear.y
            theta=np.arccos(odometry.twist.angular.x)
            
            new_odom=[x,y,theta]
            self.particles=self.MotionModel.evaluate(particles,new_odom)
            
            # #return mean of particles
            # self.x_mean=np.mean(self.particles[:,0])
            # self.y_mean=np.mean(self.particles[:,1])
            # self.theta_mean=np.arctan2(np.sin(self.particles[:,2]),np.cos(self.particles[:,2]))
            
            
        def pose_callback(self,pose):
            #new odometry message
            new_pose=Odometry()
            
            #calculate mean of new pose
            x_mean=np.mean(self.particles[:,0])
            y_mean=np.mean(self.particles[:,1])
            theta_mean=np.arctan2(np.sin(self.particles[:,2]),np.cos(self.particles[:,2]))
            
            #assign x y values in odometry message
            new_pose.pose.pose.position.x=x_mean
            new_pose.pose.pose.position.y=y_mean
            
            #get quaternion and assign 
            pose_matrix=np.array([[np.cos(theta_mean),-np.sin(theta_mean), 0,x_mean],
                                  [np.sin(theta_mean),np.cos(theta_mean),0,y_mean],
                                  [0,0,1,0]
                                  [0,0,0,1]])
            
            pose_quat=tf.transformations.quaternion_from_matrix(pose_matrix)
            
            
            new_pose.pose.pose.quaternion.x= pose_quat[0]
            new_pose.pose.pose.quaternion.y= pose_quat[1]
            new_pose.pose.pose.quaternion.z= pose_quat[2]
            new_pose.pose.pose.quaternion.w= pose_quat[3]
            
            #publish new odometry
            self.odom_pub.publish(new_pose)
            
            


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
