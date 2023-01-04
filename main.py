# This is a sample Python script.
import itertools
import numpy as np


def no_nans(array):
    if np.isnan(array).any():
        return False
    return True


class drone_cog:

    def __init__(self, cog, max_speed=10, frame_rate=960):
        self.cog = cog
        self.combinations = np.array(list(itertools.combinations([1, 2, 3, 4],
                                                                 3)))  ## Each LED is labelled 1 to 4, possible combinations of visible LEDs is listed
        self.combinations = np.append(self.combinations, [1, 2, 3, 4])  ## Case where all LEDs are visible
        self.vector_array = self.acquire_data()
        self.weights = self.find_weights()
        self.frame_rate = frame_rate
        self.max_speed = max_speed

    def acquire_data(self):
        return data_input()

    def retrieve_weights(self, index):
        position = np.where(self.combinations == index)[0]
        weights = self.weights[position]
        return weights

    def find_cog(self):
        self.vector_array = self.acquire_data()
        valid_vectors = np.array(no_nans(self.vector_array))
        count = np.count_nonzero(valid_vectors)
        cog = np.array([0, 0, 0])
        cog_list = np.array()
        points = np.array()

        if count < 3:
            raise Exception("Too few visible LEDs")

        if count > 4:
            raise Exception("Too many inputs")

        elif count == 3:
            index = np.argwhere(valid_vectors == True).flatten()
            for i in index:
                points = np.append(points, self.vector_array[i])
            weights = self.retrieve_weights(index)
            return self.find_cog_3(points, weights)

        elif count == 4:
            for index in self.combinations:
                for i in index:
                    points = np.append(points, self.vector_array[i])
                weights = self.retrieve_weights(index)
                if len(index) < 4:
                    cog_list = np.append(cog_list, self.find_cog_3(points, weights))
                else:
                    for i in range(4):
                        cog = cog + weights(i) * points[i]
                    cog_list = np.append(cog_list, cog)
            cog = np.mean(cog_list, axis=0)
            return cog

    def find_cog_3(self, points, weights):
        P = np.array([0, 0, 0])
        for i in range(0, 3):
            P = P + weights[i] * points[i]
        [A, B, C] = points
        n_cap = np.cross((B - A), (C - B))
        n_cap = n_cap / magnitude(n_cap)
        x1 = P + weights[3] * n_cap
        x2 = P - weights[3] * n_cap

        if (magnitude(X1 - self.cog) < (self.max_speed / self.frame_rate)) and (
                magnitude(X2 - self.cog) > (self.max_speed / self.frame_rate)):
            cog = X1
        elif (magnitude(X1 - self.cog) < (self.max_speed / self.frame_rate)) and (
                magnitude(X2 - self.cog) > (self.max_speed / self.frame_rate)):
            cog = X2
        else:
            raise Exception("Cant uniquely define cog with poitns {} , {}, and {} ".format(A, B, C))
        return cog

    def find_weights(self):
        points = np.array()
        weights = np.array()
        for combination in self.combinations:
            if len(combination) == 3:
                for i in combination:
                    points = np.append(points, self.vector_array[i])
                [A, B, C] = points
                n_cap = np.cross((B - A), (C - B))
                n_cap = n_cap / magnitude(n_cap)
                gamma = np.dot((self.cog - self.vector_array[0]), n_cap)
                P = self.cog - gamma * n_cap
                k = np.matmul(P, np.linalg.inv(points))
                k = np.append(k, gamma)
                weights = np.append(weights, k)
            if len(combination) == 4:
                points = self.vector_array
                k = np.matmul(self.cog, np.linalg.inv(self.vector_array))
                weights = np.append(weights, k)

        return weights


if __name__ == '__main__':
    cog = np.array([1, 1, 1])
    drone1 = drone_cog(cog)
    while 1:
        cog = drone1.find_cog()
