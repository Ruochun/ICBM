import numpy as np

def sphereFrom4Points(v):
    M = np.array(
                [[v[0][0]**2 + v[0][1]**2 + v[0][2]**2, v[0][0], v[0][1], v[0][2], 1.0],
                 [v[1][0]**2 + v[1][1]**2 + v[1][2]**2, v[1][0], v[1][1], v[1][2], 1.0],
                 [v[2][0]**2 + v[2][1]**2 + v[2][2]**2, v[2][0], v[2][1], v[2][2], 1.0],
                 [v[3][0]**2 + v[3][1]**2 + v[3][2]**2, v[3][0], v[3][1], v[3][2], 1.0]]
                )
    M11 = np.linalg.det(M[:, 1:])
    M12 = np.linalg.det(M[:, [0, 2, 3, 4]])
    M13 = np.linalg.det(M[:, [0, 1, 3, 4]])
    M14 = np.linalg.det(M[:, [0, 1, 2, 4]])
    M15 = np.linalg.det(M[:, :-1])

    np.seterr(divide='ignore', invalid='ignore')
    sphere = np.empty(4)
    sphere[0] = M12 / (2 * M11)
    sphere[1] = -M13 / (2 * M11)
    sphere[2] = M14 / (2 * M11)
    sphere[3] = np.sqrt(sphere[0]**2 + sphere[1]**2 + sphere[2]**2 - M15 / M11)
    return sphere

