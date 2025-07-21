import math
import numpy as np
import transforms3d as tf3d

# Geometric constants
aLp = -0.55     # a_{L'}
dLp = 0.6       # d_{L'}
aLF = 0.43      # a_{LF}
d1 = 0.78       # d_1
a2 = 0.32       # a_2
a3 = 1.28       # a_3
a4 = 0.2        # a_4
d4p = 1.5925    # d_{4'}
d6 = 0.2        # d_6

# Joint limits
joint_limits = [[-0.41, 7.3], [-2.967, 2.967], [-1.134, 1.484], [-3.142, 1.222], [-5.236, 5.236], [-2.269, 2.269], [-6.283, 6.283]]

class InverseKinematicsSolver(object):
    """
    An Inverse Kinematics Solver for robot composed of a linear axis and a 6-DoF ABB robotic arm.

    Inverse Kinematics Solver is ROS free. It takes the desired end-effector pose as input, computes the candidate joint positions,
    and output the best candidate which is closest to the current joint configuration.
    """
    T_EE = np.eye(4) # end-effector pose
    T_6_EE = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # end-effector pose in frame 6
    Rot_6_EE = T_6_EE[0:3,0:3]

    def rotMat_b_3(self, theta1, theta2, theta3):
        # rotation matrix from frame b to frame 3: R_b^3
        R_b3 = np.zeros((3,3))
        s1, c1 = math.sin(theta1), math.cos(theta1)
        s2, c2 = math.sin(theta2), math.cos(theta2)
        s3, c3 = math.sin(theta3), math.cos(theta3)
        R_b3[0,0] = c1*s2*c3 + c1*c2*s3
        R_b3[0,1] = -c1*s2*s3 + c1*c2*c3
        R_b3[0,2] = -s1
        R_b3[1,0] = s1*s2*c3 + s1*c2*s3
        R_b3[1,1] = -s1*s2*s3 + s1*c2*c3
        R_b3[1,2] = c1
        R_b3[2,0] = -s2*s3 + c2*c3
        R_b3[2,1] = -s2*c3 - c2*c3
        R_b3[2,2] = 0
        return R_b3

    def compute_distance(self, q_prev, q_candidate):
        def wrap_angle(angle):
            return angle % (2*np.pi)
        q_diff = q_prev - q_candidate
        q_diff[0] = abs(q_diff[0])
        for i in range(1,7):
            q_diff[i] = wrap_angle(q_diff[i])
        return np.linalg.norm(q_diff)

    def solve(self, q_prev: np.array, pos: np.array, orient: np.array):
        px, py, pz = pos.tolist()
        self.T_EE[0:3,3] = pos
        Rot = tf3d.quaternions.quat2mat(orient)
        _, _, t13, _, _, t23, _, _, t33 = Rot.flatten().tolist()
        self.T_EE[0:3,0:3] = Rot

        q_candidates = []  # candidate velocities

        # solve theta_1
        if py >= 0:
            theta1 = np.pi/2
        else:
            theta1 = -np.pi/2

        # solve q_1
        q1 = px - d6*t13 - dLp

        # solve theta_2 and theta_3
        z1 = (py - aLF - d6*t23)/math.sin(theta1) - a2
        z2 = aLp - d6*t33 + pz - d1
        x = 2*(-a4*z1 + d4p*z2)
        y = -2*(a4*z2 + d4p*z1)
        z = a3**2 - a4**2 - d4p**2 - z1**2 - z2**2
        sol_theta23 = []  # potential solutions of theta23
        if x==0 and y!=0:
            t = math.acos(z/y)
            sol_theta23 = [t, -t]
        elif y==0 and x!=0:
            t = math.asin(z/x)
            sol_theta23 = [t, math.pi-t]
        elif z==0:
            t = math.atan2(-y, x)
            sol_theta23 = [t, math.pi+t]
        else:
            d = x**2 + y**2 - z**2
            if d < 0:
                return None, "No solution: theta23 is unsolvable."
            for e in {1,-1}:
                s_theta23 = (x*z + e*y*math.sqrt(d))/(x**2 + y**2)
                c_theta23 = (y*z - e*x*math.sqrt(d))/(x**2 + y**2)
                sol_theta23.append(math.atan2(s_theta23, c_theta23))
        for theta23 in sol_theta23:
            s_theta23 = math.sin(theta23)
            c_theta23 = math.cos(theta23)
            s_theta2 = (-a4*s_theta23 - d4p*c_theta23 + z1)/a3
            c_theta2 = (-a4*c_theta23 + d4p*s_theta23 + z2)/a3
            theta2 = math.atan2(s_theta2, c_theta2)
            theta3 = theta23 - theta2

            # compute the orientation matrix Rot_3^6
            Rot_b_3 = self.rotMat_b_3(theta1, theta2, theta3)
            Rot_3_6 = Rot_b_3.transpose() @ Rot @ self.Rot_6_EE.transpose()
            r11, _, r13, r21, r22, r23, r31, _, r33 = Rot_3_6.flatten().tolist()
            
            # solve theta5
            t = math.acos(max(min(r23, 1.0),-1.0)) # avoid domain error (|r23|>1) because of computation inaccuracy
            sol_theta5 = [t, -t]
            for theta5 in sol_theta5:
                s_theta5 = math.sin(theta5)
                if s_theta5 != 0:
                    theta4 = math.atan2(r33/s_theta5, -r13/s_theta5)
                    theta6 = math.atan2(-r22/s_theta5, r21/s_theta5)
                else:
                    theta4 = 0.
                    theta6 = math.atan2(-r31, r11)
                q_candidates.append([q1, theta1, theta2, theta3, theta4, theta5, theta6])

        # check validity of candidate velocities and output the optimal one (closest to the current joint configuration)
        dist = []
        for q in q_candidates:
            valid = True
            for i, qi in enumerate(q):
                qmin, qmax = joint_limits[i][0], joint_limits[i][1]
                if qi < qmin or qi > qmax:
                    valid = False
                    break
            if valid:
                dist.append(self.compute_distance(q_prev, q))
            else:
                dist.append(np.Inf)
        idx = np.argmin(dist)
        return q_candidates[idx], ""