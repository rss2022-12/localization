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
        particles += np.matmul(R_parts, odometry)
        particles += np.random.normal(scale=0.01, size=particles.shape)

        return particles

        ####################################
