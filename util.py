import numpy as np

class sphere:
    def __init__(self, center = [0.0,0.0,0.0], radius=1.0):
        self.center = np.array(center)
        self.radius = radius

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



