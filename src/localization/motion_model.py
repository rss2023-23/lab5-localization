import numpy as np
import rospy

class MotionModel:

    def __init__(self):
        self.deterministic  = rospy.get_param("~deterministic")
        self.num_particles = rospy.get_param("~num_particles")

        # Tuneable Parameters
        self.noise_scale = 0.02

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
        # Derivation of Matrix Multiplication
        """
        #Y_k1: = cos(Theta_(k-1)1)*dX1 - sin(Theta_(k-1)1) + X_{k-1}1
        #X_k1: sin(Theta_(k-1)1)*dX1 + cos(Theta_(k-1)1) + Y_{k-1}1
        #Theta_k1: acos(cos(Theta_(k-1)1)*cos(dX1) - sin(Theta_(k-1)1)*sin(dX1)) = acos(cos(Theta_(k-1)1  + dX1)) = Theta_(k-1)1  + dX1
        """
        if not self.deterministic:
            # Apply noise to each particle
            noise = np.random.normal(loc=0.0,scale=self.noise_scale, size=(self.num_particles,3))
            odometry = np.tile(odometry, (self.num_particles,1)) + noise

            # Evaluate above Derivation
            particles[:,0] = np.multiply(np.cos(particles[:,2]),odometry[:,0]) - np.multiply(np.sin(particles[:,2]), odometry[:,1]) + particles[:,0]
            particles[:,1] = np.multiply(np.sin(particles[:,2]),odometry[:,0]) + np.multiply(np.cos(particles[:,2]), odometry[:,1]) + particles[:,1]
            particles[:,2] = particles[:,2] + odometry[:,2]
        else:
            # Evaluate above Derivation
            particles[:,0] = np.cos(particles[:,2])*odometry[0] - np.sin(particles[:,2])*odometry[1] + particles[:,0]
            particles[:,1] = np.sin(particles[:,2])*odometry[0] + np.cos(particles[:,2])*odometry[1] + particles[:,1]
            particles[:,2] = particles[:,2] + odometry[2]
        
        return particles
        
