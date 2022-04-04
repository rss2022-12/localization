#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf
import tf2_ros
import threading
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose




class ParticleFilter:

    def __init__(self):
        self.map_acquired = False

        # Get parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")

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

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.precompute_particles, queue_size=1)


        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
        self.marker_pub= rospy.Publisher("/visualization_marker_array", PoseArray, queue_size=1)

        # Initialize the models. Publish a transformation frame between the map
        # and the particle_filter_frame.
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.numparticles=200
        self.particles=np.zeros((self.numparticles, 3))
        self.last_odom_time = rospy.Time.now()
        self.lock = threading.Lock()
        self.lastmsg=None
        self.weights=None
        
        self.tfpub= rospy.Publisher(self.particle_filter_frame,TransformStamped, queue_size =1)
        
        #broadcaster for frame
        
        self.broadcast= tf2_ros.TransformBroadcaster()
        self.prev_time=0
        self.cur_time=0
        
        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #

        self.map_sub  = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_received, queue_size=1)


    def map_received(self, map_message):
        self.map_acquired = True

    def precompute_particles(self, pose):
        if self.map_acquired:
            # self.lock.acquire()
            self.particles = np.zeros((self.numparticles, 3))
            self.particles[:, 0] = pose.pose.pose.position.x
            self.particles[:, 1] = pose.pose.pose.position.y
            print(pose.pose.pose.orientation)
            quat= [pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w]
            # print(quat)
            self.particles[:, 2] = tf.transformations.euler_from_quaternion(quat)[2]
            self.particles += np.random.normal(scale=0.01, size=self.particles.shape)
            # self.lock.release()


        
    def lidar_callback(self, lidarscan):
        
        if self.map_acquired:
            self.lock.acquire()
            # with self.lock:
                #Whenever you get sensor data use the sensor model to 
                #compute the particle probabilities. Then resample the
                #particles based on these probabilities
                
            # print(self.particles)
            probs=self.sensor_model.evaluate(self.particles,np.array(lidarscan.ranges))
        
            summed = np.sum(probs)
            probs = probs/summed
            self.weights=probs
            ranges=np.arange(self.numparticles)            
            new_particles=np.random.choice(ranges,self.numparticles,p=probs)
        
            # print(np.shape(new_particles))
            # self.particles= np.zeros((self.numparticles, 3))
            self.particles=self.particles[new_particles]
            # print(np.shape(self.particles))
                

                
                # #return mean of particles
                # self.x_mean=np.mean(self.particles[:,0])
                # self.y_mean=np.mean(self.particles[:,1])
                # self.theta_mean=np.arctan2(np.sin(self.particles[:,2]),np.cos(self.particles[:,2]))
            
            self.lock.release()
            
            if self.weights is not None:
                self.estimate_pose()


        
    def odom_callback(self, odometry):
        if self.map_acquired:
            self.lock.acquire()
            # with self.lock:
                # Whenever you get odometry data use the motion model 
                # to update the particle positions
            x=odometry.twist.twist.linear.x
            y=odometry.twist.twist.linear.y
            theta=odometry.twist.twist.angular.z
            

            if self.lastmsg==None:
                dt=0
                self.lastmsg=odometry
            else:
                last_time = self.lastmsg.header.stamp.to_sec()
                dt = odometry.header.stamp.to_sec()-last_time
                self.lastmsg = odometry
                
            # new_odom=[(x+np.random.normal(scale=0.06))*dt,(y+np.random.normal(scale=0.06))*dt,(theta+np.random.normal(scale=0.06))*dt]
            new_odom=[x*dt,y*dt,theta*dt]
            # new_odom=[x,y,theta

            self.particles=self.motion_model.evaluate(self.particles, new_odom)
            
            
          
            # self.prev_time=self.cur_time
            
            # #return mean of particles
            # self.x_mean=np.mean(self.particles[:,0])
            # self.y_mean=np.mean(self.particles[:,1])
            # self.theta_mean=np.arctan2(np.sin(self.particles[:,2]),np.cos(self.particles[:,2]))

            self.lock.release()
            
            if self.weights is not None:
                self.estimate_pose()


    def estimate_pose(self):
        if self.map_acquired:
            self.lock.acquire()
            # with self.lock:
                #new odometry message
            new_pose=Odometry()

            new_pose.header.frame_id = "/map"
            new_pose.header.stamp = rospy.Time.now()
            
            #calculate mean of new pose
            x_mean=np.average(self.particles[:,0],weights=self.weights)
            y_mean=np.average(self.particles[:,1],weights=self.weights)
            theta_mean=np.arctan2(np.sum(np.sin(self.particles[:,2])),np.sum(np.cos(self.particles[:,2])))
            
            #assign x y values in odometry message
            new_pose.pose.pose.position.x=x_mean
            new_pose.pose.pose.position.y=y_mean
            
            #get quaternion and assign 
            pose_matrix=np.array([[np.cos(theta_mean),-np.sin(theta_mean), 0,x_mean],
                                    [np.sin(theta_mean),np.cos(theta_mean),0,y_mean],
                                    [0,0,1,0],
                                    [0,0,0,1]])
            
            
            # pose_quat=tf.transformations.quaternion_from_matrix(pose_matrix)

            pose_quat=tf.transformations.quaternion_from_euler(0,0,theta_mean)

            
            # Add the source and target frame
            new_pose.header.frame_id =  "map"
            new_pose.child_frame_id =   self.particle_filter_frame
            new_pose.pose.pose.orientation.x= pose_quat[0]
            new_pose.pose.pose.orientation.y= pose_quat[1]
            new_pose.pose.pose.orientation.z= pose_quat[2]
            new_pose.pose.pose.orientation.w= pose_quat[3]
            
            #publish new odometry
            self.odom_pub.publish(new_pose)
            
            
            #send transform from map to particle_filter_frame
            particletransform= geometry_msgs.msg.TransformStamped()
            
            # Add the source and target frame
            particletransform.header.frame_id =  "map"
            particletransform.child_frame_id =   self.particle_filter_frame
            particletransform.header.stamp = rospy.Time.now()
            
            # # Add the translation
            particletransform.header.stamp=rospy.Time.now()
            particletransform.transform.translation.x = x_mean
            particletransform.transform.translation.y = y_mean
            particletransform.transform.translation.z = 0 
            
            # Add the rotation
            particletransform.transform.rotation.x = pose_quat[0]
            particletransform.transform.rotation.y = pose_quat[1]
            particletransform.transform.rotation.z = pose_quat[2]
            particletransform.transform.rotation.w = pose_quat[3]
            
            self.broadcast.sendTransform(particletransform)

            
             
            self.particle_array=PoseArray()
            
            for i in np.arange(self.numparticles):
                pose = Pose()
               
                pose.position.x = self.particles[i,0]
                pose.position.y = self.particles[i,1]
                pose.orientation.x= self.particles[i,0]
                pose.orientation.y= self.particles[i,1]
                
                self.particle_array.poses.append(pose)
            
            self.particle_array.header.frame_id = "map"
            self.marker_pub.publish(self.particle_array)
            
            self.lock.release()
    
            
            
            
            


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
