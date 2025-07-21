import sympy as sp

# Define symbolic variables
q1, theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('q1 θ1 θ2 θ3 θ4 θ5 θ6')
a_Lp, a_LF, a2, a3, a4 = sp.symbols('a_Lp a_LF a2 a3 a4')
d_Lp, d1, d_4p, d6 = sp.symbols('d_Lp d1 d_4p d6')

t11, t12, t13, t21, t22, t23, t31, t32, t33, px, py, pz = sp.symbols('t11, t12, t13, t21, t22, t23, t31, t32, t33, px, py, pz')

# Modified DH table: [alpha_{i-1}, a_{i-1}, theta_i, d_i]
dh_params = [
    (0,        0,         0,                q1),
    (0,        a_Lp,      sp.pi/2,          d_Lp),
    (-sp.pi/2, a_LF,      0,                0),
    (0,        0,         theta1 - sp.pi/2, d1),
    (-sp.pi/2, a2,        theta2 - sp.pi/2, 0),
    (0,        a3,        theta3,           0),
    (-sp.pi/2, a4,        theta4,           0),
    (0,        0,         0,                d_4p),
    (sp.pi/2,  0,         theta5,           0),
    (-sp.pi/2, 0,         theta6,           d6),
    (0,        0,         sp.pi,            0),
]

# Generate Modified DH transformation matrix
def modified_dh_matrix(alpha, a, theta, d):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a],
        [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha)*d],
        [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha), sp.cos(alpha), sp.cos(alpha)*d],
        [0, 0, 0, 1]
    ])

def desired_matrix():
    return sp.Matrix([
        [t11, t12, t13, px],
        [t21, t22, t23, py],
        [t31, t32, t33, pz],
        [  0,   0,   0,  1],
    ])

# Define transformation matrix from 0 to base frame 
def transform_b_0():
    return sp.Matrix([
        [ 0, 0, 1, 0],
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 0, 1]
    ])

# Compute each transformation and the full chain
T = sp.eye(4)
transforms = []
for i, (alpha, a, theta, d) in enumerate(dh_params):
    T_i = modified_dh_matrix(alpha, a, theta, d)
    transforms.append(T_i)
    T = T * T_i

# Compute the fixed transformation from 0 to b
T_b_0 = transform_b_0()

# Print each intermediate transformation matrix
for i, Ti in enumerate(transforms):
    print(f"T_{i}^{i+1} = ")
    sp.pprint(Ti)
    print("\n")

# Compute end-effector pose in reference frame
T_0EE = sp.simplify(T)
T_bEE = sp.simplify(T_b_0*T)
print("Final transformation T_b^EE = ")
sp.pprint(T_bEE)

# Get desired transformation matrix
T_des = desired_matrix()

## solving q1
# Compute transformation matrix T_5_EE
T_5_EE = sp.eye(4)
for i in range(2):
    T_5_EE = transforms[-1-i]*T_5_EE
T_5_EE = sp.simplify(T_5_EE)

# Compute transformation matrix T_b_1
T_0_1 = sp.eye(4)
for i in range(4):
    T_0_1 = T_0_1 * transforms[i]
T_b_1 = T_b_0*T_0_1

# Compute transformation matrix T_0_5
T_EE_5 = sp.simplify(T_5_EE.inv())
T_b_5 = sp.simplify(T_bEE*T_EE_5)
T_0_5 = T_b_0.inv()*T_b_5
print("Transformation T_0^5 = ")
sp.pprint(sp.simplify(T_0_5))

# Compute desired T_0_5
print("Desired transformation T_0^5 = ")
sp.pprint(sp.simplify(T_b_0.inv()*T_des*T_5_EE.inv()))

## solving theta_2 and theta_3
# Compute transformation matrix involving linear axis
T_0_LF = sp.eye(4)
for i in range(3):
    T_0_LF = T_0_LF * transforms[i]
T_b_LF = T_b_0*T_0_LF

T_05 = T_0EE*T_EE_5
T_05 = sp.simplify(T_05)
print("Transformation T_LF^5 = ")
sp.pprint(sp.simplify(T_0_LF.inv()*T_05))

print("Desired transformation T_LF^5 = ")
sp.pprint(sp.simplify(T_b_LF.inv()*T_des*T_5_EE.inv()))

## solving theta_4, theta_5, theta_6
# Compute transformation matrix from 0 to 3
T_0_3 = sp.eye(4)
for i in range(6):
    T_0_3 = T_0_3 * transforms[i]
T_b_3 = T_b_0*T_0_3

print("Transformation T_b^3 = ")
sp.pprint(T_b_3)

print("Transformation T_6^EE = ")
sp.pprint(transforms[-1])

T_EE_6 = sp.simplify(transforms[-1].inv())
T_3_6 = T_0_3.inv()*T_0EE*T_EE_6
T_3_6 = sp.simplify(T_3_6)
print("Transformation T_3^6 = ")
sp.pprint(T_3_6)

print("Desired transformation T_3^6 = ")
sp.pprint(sp.simplify(T_b_3.inv()*T_des*T_EE_6))
