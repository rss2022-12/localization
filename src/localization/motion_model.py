import numpy as np
from math import pi, sin, cos, acos

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        pass

        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        
        ####################################
        
        # for r in range(particles.shape[0]):
        #     old_theta = particles[r][2]
        #     R_part = np.array([[np.cos(old_theta), -np.sin(old_theta), 0.0], [np.sin(old_theta), np.cos(old_theta), 0.0], [0.0, 0.0, 1.0]])
        #     particles[r] += np.matmul(R_part, odometry)

        # return particles

        old_thetas = particles[:, 2]
        R_parts = np.zeros((old_thetas.shape[0], 3, 3))
        R_parts[:, 0, 0] = np.cos(old_thetas)
        R_parts[:, 0, 1] = -np.sin(old_thetas)
        R_parts[:, 0, 2] = 0.0
        R_parts[:, 1, 0] = np.sin(old_thetas)
        R_parts[:, 1, 1] = np.cos(old_thetas)
        R_parts[:, 1, 2] = 0.0
        R_parts[:, 2, 0] = 0.0
        R_parts[:, 2, 1] = 0.0
        R_parts[:, 2, 2] = 1.0

        n=len(particles)
<<<<<<< HEAD
        x_n= np.random.normal(odometry[0],scale=0.01,size=n)
        y_n=np.random.normal(odometry[1],scale=0.01,size=n)
        theta_n=np.random.normal(odometry[2],scale=0.01,size=n)
=======
        x_n= np.random.normal(odometry[0]+0.01,scale=0.03,size=n)
        y_n=np.random.normal(odometry[1],scale=0.03,size=n)
        theta_n=np.random.normal(odometry[2],scale=0.03,size=n)
>>>>>>> ff16cd35c64011114325e891655c4c0bef18a504

        odom_noisy=np.reshape(np.vstack((x_n,y_n,theta_n)).T, (n, 3, 1))

        particles+=np.reshape(np.matmul(R_parts,odom_noisy), (n, 3))

       # particles += np.matmul(R_parts, odometry)
        # particles += np.random.normal(scale=0.001, size=particles.shape)
        # particles[:,2]+=np.random.normal(scale=0.03,size=particles[:,2].shape)
        # particles[:,1]+=np.random.normal(scale=0.01,size=particles[:,1].shape)
        # particles[:,0]+=np.random.normal(scale=0.01, loc=0.01,size=particles[:,0].shape)
        return particles

        ####################################
