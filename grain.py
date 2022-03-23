from cmath import sqrt
import random
import operator
import numpy as np
import helpers as hlp

class Grain2D:
    ex = np.array([1, 0], dtype=float)  # Unit vector in x-direction
    ey = np.array([0, 1], dtype=float)  # Unit vector in y-direction
    tensile_strength = 300


    def __init__(self, vertices: np.ndarray) -> None:
        # Validation of the received arguments
        assert type(vertices) == np.ndarray, f"Vertices {vertices} is not of type: numpy.ndarray!"
        assert vertices.shape[1] == 2, f"The Vertices are not two-dimensional!"
        assert len(vertices) > 2, f"There are not enough point to create a grain. Does have at least three!"
        
        # Assigning default variables
        self.vertices = np.array([vertices])
        self.cutting_edge = hlp.get_vector_miny(self.vertices[-1])
        self.uppercrack = np.array([])
        self.penetration_depth = 0
        self.fractured = False
        self.fractures = 0
        self.finish = False
    

    def initializeValues(self, F_c, F_n, d) -> None:
        self.penetration_depth = d
        self.rake_face()
        self.trailingFace()


    def rake_face(self) -> None:
        '''Input: penetration depth
            Outut: rake face start and end point'''
        max_vector = self.cutting_edge + self.penetration_depth * self.ey
        y = []
        for ind, val in enumerate(self.vertices[-1]): # list of all points that penetrate the workpiece
            if val[1] <= max_vector[1]:
                y.append(val[1])
            else:
                y.append(self.cutting_edge[1])
        y_ind, y_val = max(enumerate(y), key=operator.itemgetter(1)) # index and y-value of point with max y-value
        self.rake_face = np.array([self.vertices[-1][y_ind], self.vertices[-1][y_ind + 1]]) # start and end point of the rake face
        self.rake_face_length = np.linalg.norm(self.rake_face_vector()) # sets rake face length of the grain
    

    def rake_face_vector(self) -> np.ndarray:
        """" Returns unnormalized direction vector of the rake face"""
        return hlp.get_vector(self.rake_face[0], self.rake_face[1]) 
    

    def trailingFace(self) -> None:
        """Start and end point of the trailing face"""
        end = hlp.get_vector_maxx(self.vertices[0])
        self.trailing_face = np.array([self.cutting_edge, end])
        


    def find_contact_length(self) -> float:
        """Returns the grain-chip contact length depending on the penetration depth"""
        a = (self.penetration_depth + self.cutting_edge[1] - self.rake_face[0][1]) / (self.rake_face[1][1] - self.rake_face[0][1])
        pd = self.rake_face[0] + a * hlp.get_vector(self.rake_face[0], self.rake_face[1])
        contact_length_vector = hlp.get_vector(self.cutting_edge, pd)
        return np.linalg.norm(contact_length_vector)


    def cuttingRectangle(self) -> np.ndarray:
        """Returns a rectangle to be intersected with the grain to get the cutting area"""
        origin = self.cutting_edge + self.penetration_depth * self.ey
        left_upper = origin - 10 * self.ex
        right_upper = origin + 10 * self.ex
        right_lower = right_upper - 2 * self.penetration_depth * self.ey
        left_lower = left_upper - 2 * self.penetration_depth * self.ey
        return np.array([left_upper, right_upper, right_lower, left_lower])


    def rankineCriterion(self, F_c : float, F_n : float, W_t : float) -> float:
        """Returns rankine stress"""
        intersection, area = hlp.polygonIntersection(self.vertices[-1], self.cuttingRectangle())  # Intersection between grain and workpiece
        b = np.linalg.norm(hlp.get_vector(intersection[0], intersection[1]))  # grain width
        A_cut = (self.penetration_depth - W_t) * b  # cutting area
        A_orto = b ** 2  # area orthogonal to penetration depth
        sigma_c = F_c / A_cut  # cutting stress
        sigma_n = F_n / A_orto  # normal stress
        tau_s = F_c / A_orto  # shear stress
        rankine_stress = ((sigma_c + sigma_n) + sqrt((sigma_c - sigma_n)**2 + 4 * tau_s**2)) / 2
        return rankine_stress
    
    
    def crack_propagation(self) -> None:
        """Calculates new crack point depending on the existing crack"""
        crack_index = int(self.getCrackStart())
        crack_base = hlp.get_vector(self.uppercrack[crack_index], self.uppercrack[crack_index + 1])
        crack_base_length = np.linalg.norm(crack_base)
        crack_direction = hlp.unit_vector(np.array([crack_base[1], -crack_base[0]]))
        crack_length = random.uniform(0.5 * crack_base_length, crack_base_length)
        crack_point = self.uppercrack[crack_index] + random.uniform(0.1*crack_base_length, 0.8*crack_base_length)* crack_base + crack_length * crack_direction
        crack_point, finish = self.checkCrackFinish(crack_point)
        self.uppercrack = hlp.numpyInsertPoint(self.uppercrack, crack_point, crack_index + 1)
        new_vertices = hlp.numpyInsertArrayOfPoints(self.vertices[0], self.uppercrack, self.crack_start)
        if self.finish:
            crit_index = hlp.getPointIndex(new_vertices, crack_point)
            new_vertices = new_vertices[crit_index:]
        self.appendVertices(new_vertices)
        self.fractures += 1
    
    def initialCrack(self) -> None:
        """Creates the first crack in the grain"""
        pod_from_rf = random.uniform(2, 3) * self.find_contact_length() * hlp.unit_vector(self.rake_face_vector())
        pod = self.rake_face[0] + pod_from_rf
        self.pod = pod
        crack_length = random.uniform(0.025*self.rake_face_length, 0.075*self.rake_face_length)
        crack_direction = np.array([pod_from_rf[1], -pod_from_rf[0]])
        crack_direction_unit = hlp.unit_vector(crack_direction)
        crack = pod + crack_length * crack_direction_unit
        rf_crack_lower = pod - \
        random.uniform(0.5*crack_length, crack_length) * hlp.unit_vector(pod_from_rf)
        rf_crack_upper = pod + \
        random.uniform(0.5*crack_length, crack_length) * hlp.unit_vector(pod_from_rf)
        crack = np.array([rf_crack_lower, crack, rf_crack_upper])
        for index, value in enumerate(self.vertices[0]):
            if (value-self.rake_face[0]).all:
                self.crack_start = index + 1
                break
        temp_vertices = self.vertices[-1].tolist()
        temp_crack = crack.tolist()
        for ind, point in enumerate(temp_crack):
            temp_vertices.insert(int(self.crack_start + ind), point)
        cracks_temp = self.uppercrack.tolist()
        cracks_temp.append(temp_crack)
        uppercrack = np.array([np.array(v) for v in cracks_temp], dtype=object)
        self.uppercrack = uppercrack[0]
        old_vertices = self.vertices.tolist()
        old_vertices.append(temp_vertices)
        self.vertices = np.array([np.array(w) for w in old_vertices], dtype=object)
        self.fractured = True
        self.fractures += 1
    
    def make_crack(self, F_c : float, F_n : float) -> None:
        """Checks fracture criteria and invokes function rankine stress exceeds tensile strength"""
        if self.rankineCriterion(F_c, F_n, 0) > self.tensile_strength:
            if self.fractured:
                self.crack_propagation()
            else:
                self.initialCrack()
    

    def getCrackStart(self) -> int:
        """Returns the index of the start point for the crack propagation"""
        if self.fractures <= 6:
            if (self.fractures % 2) == 0:
                return self.fractures / 2
            else:
                return self.fractures / 2 + 0.5
        else:
            return self.fractures -3
    
    
    def appendVertices(self, vertices : np.ndarray) -> None:
        """Appends list of updated vertices to the list of existing vertices"""
        old_vertices = self.vertices.tolist()
        new_vertices = vertices.tolist()
        old_vertices.append(new_vertices)
        self.vertices = np.array([np.array(x) for x in old_vertices], dtype=object)


    def checkCrackFinish(self, crack_point) -> np.ndarray:
        """Checks if new crack point lies outside the grain.
        cuts the grain if the condition is true"""
        trailing_face_vector = hlp.get_vector(self.trailing_face[0], self.trailing_face[1])
        a = (crack_point[1] - self.trailing_face[0][1]) / trailing_face_vector[1]
        crit_point = self.trailing_face[0] + a * trailing_face_vector
        if crack_point[0] > crit_point[0]:
            self.finish = True
            return crit_point, True
        else:
            return crack_point, False
        