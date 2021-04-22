import numpy as np
from scipy.optimize import least_squares

def D(S, V, assignment):
    # From a flattened S to a mapped S (according to assignment)
    mappedS = np.zeros((V.shape[0], V.shape[1]+1))
    vert_num = 0
    for sphere_num in assignment:
        mappedS[vert_num, :] = S[4*sphere_num:4*sphere_num+4]
        vert_num += 1

    return  (np.sqrt( (mappedS[:, 0] - V[:, 0])**2 
                    + (mappedS[:, 1] - V[:, 1])**2
                    + (mappedS[:, 2] - V[:, 2])**2
                    ) - mappedS[:, 3]
            ) # using least square optimizer, no abs value needed

def J(S, V, assignment):
    J = np.zeros((V.shape[0], len(S)))

    mappedS = np.zeros((V.shape[0], V.shape[1]+1))
    vert_num = 0
    for sphere_num in assignment:
        mappedS[vert_num, :] = S[4*sphere_num:4*sphere_num+4]
        vert_num += 1

    # func_val = np.sqrt( (mappedS[:, 0] - V[:, 0])**2 
    #                 + (mappedS[:, 1] - V[:, 1])**2
    #                 + (mappedS[:, 2] - V[:, 2])**2
    #               ) - mappedS[:, 3]
    # sign = np.empty(V.shape[0])
    # for i in range(len(sign)):
    #    sign[i] = 1.0 if func_val[i] >= 0 else -1.0

    deriv = 0.5 * (
                (mappedS[:, 0] - V[:, 0])**2
                + (mappedS[:, 1] - V[:, 1])**2
                + (mappedS[:, 2] - V[:, 2])**2
            )**(-0.5) 
    
    vert_num = 0
    for sphere_num in assignment:
        J[vert_num, 4*sphere_num] = deriv[vert_num] * 2 * (mappedS[vert_num, 0] - V[vert_num, 0]) # * sign[vert_num]
        J[vert_num, 4*sphere_num+1] = deriv[vert_num] * 2 * (mappedS[vert_num, 1] - V[vert_num, 1]) # * sign[vert_num]
        J[vert_num, 4*sphere_num+2] = deriv[vert_num] * 2 * (mappedS[vert_num, 2] - V[vert_num, 2]) # * sign[vert_num]
        J[vert_num, 4*sphere_num+3] = -1.0 # * sign[vert_num]
        vert_num += 1

    return J


def optimizeAsgdSpheresFromVert(verts, spheres, assignment):
    S = spheres.flatten()
    V = np.empty((len(verts), 3))
    for i in range(len(verts)):
        V[i, :] = np.array(verts[i])

    optRes = least_squares(D, S, jac=J, args=(V, assignment), verbose=1)
    optS = np.reshape(optRes.x, (-1, 4))
    return optS


