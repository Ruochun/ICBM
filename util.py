import numpy as np

class sphere:
    def __init__(self, center = [0.0,0.0,0.0], radius=1.0):
        self.center = np.array(center)
        self.radius = radius

def EucDistSq(a, b):
    return (a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2

def getNormal(P,Q,R):
    V = Q-P
    W = R-P
    N = np.cross(V,W)
    return N/np.linalg.norm(N)

def getTri(v):
    return np.array([ v[0][0], v[1][0], v[2][0] ])

def getPointPlaneDist(point, planePoint, normal):
    return abs(np.dot(point-planePoint, normal))

def inscribedSphereViaPointMesh(center, mesh):
    radius = 1e10
    for v_vt_vn in mesh.polygons:
        tri = getTri(v_vt_vn)
        P = np.array(mesh.vertices[tri[0]])
        Q = np.array(mesh.vertices[tri[1]])
        R = np.array(mesh.vertices[tri[2]])
        normal = getNormal(P,Q,R)
        distance = getPointPlaneDist(center, P, normal)
        if distance<radius:
            radius = distance

    return sphere( center = center, radius = radius)
       
def interpolateSphere(bigone, smallone, ratio):
    if ratio>1.0 or ratio<0.0:
        raise Exception("The spherical interpolation ration has to be between 0 and 1")
    C2c = smallone.center - bigone.center
    distC2c = np.linalg.norm(C2c)
    if distC2c > 1e-6:
        C2c_unit = C2c/distC2c
        c2b = C2c_unit*smallone.radius
        C2B = C2c_unit*bigone.radius
        vec1 = (C2c + c2b)*(1.0-ratio) + C2B*ratio
        vec2 = (C2c - c2b)*(1.0-ratio) - C2B*ratio
        C2new = (vec1 + vec2)/2.0
        radius = np.linalg.norm(vec1-vec2)/2.0
        center = bigone.center + C2new
    else:
        center = bigone.center
        radius = smallone.radius*(1.0-ratio) + bigone.radius*ratio
    return sphere( center = center, radius = radius)

def box2ball(box):
    ball = sphere(  
                    center = [(box[0,0]+box[1,0])/2.0, (box[0,1]+box[1,1])/2.0, (box[0,2]+box[1,2])/2.0],
                    radius = np.sqrt( (box[1,0]-box[0,0])**2 + (box[1,1]-box[0,1])**2 + (box[1,2]-box[0,2])**2)/2.0
                 )
    return ball
                    

def bbox(verts):
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    box = np.array([ [min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]])
    return box

def coord_avg(verts):
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    center = np.array([float(sum(xs)/len(xs)), float(sum(ys)/len(ys)), float(sum(zs)/len(zs))])
    return center

def findClosestSphere(verts, spheres):
    num_list = np.zeros(len(verts)).astype(int)
    for i in range(len(verts)):
        min_dist = 1e30
        for j in range(len(spheres)):
            dist = EucDistSq(verts[i], spheres[j, :3])
            if dist < min_dist:
                min_dist = dist
                num_list[i] = j

    return num_list


