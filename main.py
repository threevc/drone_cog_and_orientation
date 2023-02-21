import itertools
import numpy as np


def Rx(theta):
    theta = np.radians(theta)
    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])


def Ry(theta):
    theta = np.radians(theta)
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])


def Rz(theta):
    theta = np.radians(theta)
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])


class LEDArray:
    def __init__(self, led_num, initial_positions, initial_cog):
        self.led_num = led_num
        self.initial_positions = initial_positions
        self.initial_cog = initial_cog
        self.initial_normal = np.array([])
        self.normal = np.array([])
        self.cog = initial_cog
        self.K = np.array([])
        self.R = np.array([])
        self.mu = np.NAN

        self.initialize_gains()

    def initialize_gains(self):
        A = self.initial_positions
        cog = self.initial_cog
        n_cap = np.cross((A[1] - A[0]), (A[2] - A[0]))
        n_cap = n_cap / np.linalg.norm(n_cap)
        self.initial_normal = n_cap

        mu = np.dot((cog - A[0]), n_cap)
        P = cog - mu * n_cap
        K = P @ np.linalg.inv(A)
        self.K = K
        self.mu = mu
        return [K, mu]

    def update_cog_and_norm(self, led_positions):
        A = led_positions
        P = self.K @ A
        n_cap = np.cross((A[1] - A[0]), (A[2] - A[0]))
        n_cap = n_cap / np.linalg.norm(n_cap)
        self.normal = n_cap
        cog = P + self.mu * n_cap
        self.cog = cog
        return [cog, n_cap]

    def find_rotation_matrix(self, led_positions):
        cog_dash = self.update_cog_and_norm(led_positions)[0]
        A_dash = led_positions.T
        A = self.initial_positions
        A_ddash = np.subtract(A_dash, cog_dash)  # Row matrix
        R = A_ddash.T @ np.linalg.inv(A.T)
        self.R = R
        return R

    def find_euler_radian(self, led_positions):
        # Returns the euler angles (in radians) given the positions of the leds
        R = self.find_rotation_matrix(led_positions)
        alpha = np.arctan2(R[1, 0], R[0, 0])
        beta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        gamma = np.arctan2(R[2, 1], R[2, 2])

        return np.array([alpha, beta, gamma])

    def find_euler_degrees(self, led_positions):
        # Returns the euler angles (in degrees) given the positions of the leds
        R = self.find_rotation_matrix(led_positions)
        alpha = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        beta = np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        gamma = np.degrees(np.arctan2(R[2, 1], R[2, 2]))

        return np.array([alpha, beta, gamma])


class DroneCog:

    def __init__(self, cog, led_positions):
        self.initial_cog = cog
        self.cog = cog
        self.combinations = list(itertools.combinations([0, 1, 2, 3], 3))
        # self.combinations.append([0, 1, 2, 3])
        self.led_array_list = []
        self.led_positions = led_positions
        # for combination in self.combinations:
        #     vector1 = self.led_positions[combination]
        #     array1 = LED_Array(self.led_positions[combination], self.cog )

    def find_euler_angles_radian(self, led_positions):
        a = 1+1
        return a
        # find the list of valid vector
        # from this list, find the first valid combination
        # led_pos = led_positions[valid_comb]
        # Find the object with valid combs
        # [alpha, beta, gamma] = object1.find_euler_degrees(led_pos)
        # cog = object1.find_cog_and_norm(led_pos)[0]


def trial_run(alpha, beta, gamma):
    # Creating a dummy scenario
    # Inputs
    vector1 = np.array([0, 0, 1])                 # Vector Positions of the LEDs at startup
    vector2 = np.array([-1, 0, 0])
    vector3 = np.array([0, -1, 0])
    cog = np.array([0, 0, 0])                     # Vector position of the COG at startup
    R_given = (Rz(alpha) @ (Ry(beta) @ (Rx(gamma))))     # Finding the Rotation matrix from user-defined angles

    # Rotating the bot and then translating by (1,1,1) to generate led positions
    A = np.array([vector1, vector2, vector3])     # Converting the vectors to a row matrix
    A_dash = (R_given @ A.T).T                    # Rotating the column matrix, and transposing to get a list of row vectors
    A_dash += 1                                   # Translating by [1,1,1] to get the matrix of LED positions at time T

    # Initialization calculations
    n_cap = np.cross((vector2 - vector1), (vector3 - vector1))
    n_cap = n_cap / np.linalg.norm(n_cap)         # Finding the normalized normal to the plane

    mu = np.dot((cog - vector1), n_cap)           # Finding the distance of the COG from the projection point P
    P = cog - mu * n_cap                          # P is mu distance away from COG along the anti-normal
    K = P @ np.linalg.inv(A)                      # P = K@A => K = P@inv(A) (where K,P,A all are rows/row matrices)
    parameters = np.append(K, mu)                 # Storing the params in an array

    # print("Original n_cap is {}".format(n_cap))
    # print("Gamma is {}".format(mu))
    # print("P = {}".format(P))
    # print("K = {}".format(K))
    # print("Mu = {}".format(mu))

    # Recurring calculations
    # Recalculating the Cog and Norm

    K = parameters[0:3]                          # Unpacking the params
    mu = parameters[3]
    P_dash = K @ A_dash                          # P = KA' (row matrices/vectors)

    n_cap = np.cross((A_dash[1] - A_dash[0]), (A_dash[2] - A_dash[0]))  # Calculating n_cap
    n_cap = n_cap / np.linalg.norm(n_cap)
    cog_dash = P_dash + mu * n_cap               # Calculating the new cog from P'
    # print("New n_cap is {}".format(n_cap))
    # print("Calculated cog is {}".format(cog_dash))

    # list1 = list(itertools.combinations([1,2,3,4],3))
    # list1.append((1,2,3,4))
    # print(list1)

    A_ddash = np.subtract(A_dash, cog_dash)      # Row matrices, A'' = A' - COG'
    R = A_ddash.T @ np.linalg.inv(A.T)           # A'' = R@A => R = A''@inv(A) (where  A, A'' are column matrices)

    alpha = np.arctan2(R[1, 0], R[0, 0])         # Calculating the Euler angles from elements of R
    beta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    gamma = np.arctan2(R[2, 1], R[2, 2])

    alpha1 = np.degrees(alpha)                   # Alpha is rotation about Z axis
    beta1  = np.degrees(beta)                    # Beta  is rotation about Y axis
    gamma1 = np.degrees(gamma)                   # Gamma is rotation about X axis

    R_calculated = Rz(alpha1) @ (Ry(beta1) @ Rx(gamma1))  # Comparing actual R to Euler R

    print("R original is \n{}".format(np.around(R_given, decimals=2)))
    print("R calculated from linalg is \n{}".format(np.around(R, decimals=2)))
    print("R calculated from euler angles is \n{}".format(np.around(R_calculated, decimals=2)))

    print("The original vectors are \n{}".format(np.around(A.T)))
    print("The original rot mat * The og vectors = \n{}".format(np.around(R_given @ A.T, decimals=2)))
    print("A'' (which should be the same as above) is \n{}".format(np.around(A_ddash.T, decimals=2)))
    print("The vectors which should have come out from inverting are \n{}".format(
        np.around(((R_given @ A.T) @ np.linalg.inv(A.T))), decimals=2))

    print(" The alpha beta and gamma are : {}   {}   {} ".format(alpha1, beta1, gamma1))

if __name__ == '__main__':

    trial_run(20, 72, 105)  # alpha beta gamma
    # drone1 = drone_cog(cog , led_positions )
    # drone1.find_euler_angles_radian(led_positions)
