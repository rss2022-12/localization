import numpy as np
from scan_simulator_2d import PyScanSimulator2D


import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
import math as math


class SensorModel:

    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param(
            "~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param(
            "~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.lidar_scale_to_map_scale = 1
        ####################################
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        # self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale")

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization)

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self.map_callback,
            queue_size=1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """

        zmax = 200.0
        self.sensor_model_table = np.zeros((201, 201))

        phit = np.zeros((201, 201))

        for z in range(201):
            for d in range(201):
                # phit
                if 0 <= z <= zmax:
                    phit_i = 1.0/np.sqrt(2.0*math.pi*self.sigma_hit**2.0)*np.exp(
                        (-(float(z)-float(d))**2.0)/(2.0*self.sigma_hit**2))
                else:
                    phit_i = 0
                phit[z, d] = phit_i

        summed = np.sum(phit, axis=0, keepdims=True)
        phit_norm = phit/summed

        for z in range(201):
            for d in range(201):

                # pshort
                if 0 <= z <= d and d != 0:
                    pshort = (2.0/d)*(1-float(z)/float(d))
                else:
                    pshort = 0

                # pmax
                if z == zmax:
                    pmax = float(1.0)
                else:
                    pmax = 0

                # prand
                if 0 <= z <= zmax:
                    prand = 1.0/zmax
                else:
                    prand = 0
                self.sensor_model_table[z, d] = self.alpha_hit*phit_norm[z, d] + \
                    self.alpha_short*pshort + self.alpha_max*pmax + self.alpha_rand*prand
        summed = np.sum(self.sensor_model_table, axis=0, keepdims=True)
        self.sensor_model_table = self.sensor_model_table/summed

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle

        scans = self.scan_sim.scan(particles)

        ####################################
        zmax = 200.0
        scans = scans/(self.map_resolution*self.lidar_scale_to_map_scale)
        scans = np.rint(scans).astype(np.uint16)
        scans = np.clip(scans, 0, zmax)

        pix = observation/(self.map_resolution*self.lidar_scale_to_map_scale)
        pix = np.rint(pix).astype(np.uint16)

        pix = np.clip(pix, 0, zmax)

        probs_whole = np.ones(np.size(scans, 0))

        for i in range(np.size(scans, 0)):
            for j in range(np.size(scans, 1)):
                aa = int(pix[j])
                bb = int(scans[i, j])
                dd = self.sensor_model_table[aa, bb]
                probs_whole[i] *= (dd)**(1.0/2.2)

        rospy.loginfo(np.shape(scans))
        return probs_whole

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)
        self.map_resolution = map_msg.info.resolution
        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
            origin_o.x,
            origin_o.y,
            origin_o.z,
            origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
