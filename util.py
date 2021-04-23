import numpy as np
import trimesh as tm
import wavefront as wf

class sphere:
    def __init__(self, center = [0.0,0.0,0.0], radius=1.0):
        self.center = np.array(center)
        self.radius = radius

def EucDist(a, b):
    return np.sqrt( (a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2 )

def getNormal(P,Q,R):
    V = Q-P
    W = R-P
    N = np.cross(V,W)
    return N/np.linalg.norm(N)

def getTri(v):
    return np.array([ v[0][0], v[1][0], v[2][0] ])

# it is signed distance: postive = from inside mesh
def getPointPlaneDist(point, planePoint, normal):
    return np.dot(planePoint - point, normal)

def getBoxDim(corner1, corner2):
    return np.array([abs(corner1[0] - corner2[0]), abs(corner1[1] - corner2[1]), abs(corner1[2] - corner2[2])])

def pointInBox(pnt, corner1, corner2):
    mid = [(corner1[0] + corner2[0]) / 2.0, (corner1[1] + corner2[1]) / 2.0, (corner1[2] + corner2[2]) / 2.0]
    hdim = [abs(corner1[0] - corner2[0]) / 2.0, abs(corner1[1] - corner2[1]) / 2.0, abs(corner1[2] - corner2[2]) / 2.0]
    return (abs(pnt[0] - mid[0]) <= hdim[0]) and (abs(pnt[1] - mid[1]) <= hdim[1]) and (abs(pnt[2] - mid[2]) <= hdim[2])

def inscribedSphereViaPointMesh(center, mesh):
    radius = 1e10
    for v_vt_vn in mesh.polygons:
        tri = getTri(v_vt_vn)
        P = np.array(mesh.vertices[tri[0]])
        Q = np.array(mesh.vertices[tri[1]])
        R = np.array(mesh.vertices[tri[2]])
        normal = getNormal(P,Q,R)
        distance = abs(getPointPlaneDist(center, P, normal))
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

# find closest facet in mesh, and then see if the signed distance is positive.
# this should work for not-too-crazy shapes, not necessarily convex.
# If the mesh is convex, it can be easier, all signed_dist should be positive.
def pointInMesh(point, mesh):
    smallest_dist = 1e30
    isPositive = False
    for v_vt_vn in mesh.polygons:
        tri = getTri(v_vt_vn)
        P = np.array(mesh.vertices[tri[0]])
        Q = np.array(mesh.vertices[tri[1]])
        R = np.array(mesh.vertices[tri[2]])
        pTriDist = min([EucDist(point, P), EucDist(point, Q), EucDist(point, R)])
        if pTriDist < smallest_dist:
            smallest_dist = pTriDist
            normal = getNormal(P,Q,R)
            signed_dist = getPointPlaneDist(point, P, normal)
            isPositive = True if signed_dist > 0.0 else False
    
    return isPositive

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
            dist = abs(EucDist(verts[i], spheres[j, :3]) - spheres[j, -1])
            if dist < min_dist:
                min_dist = dist
                num_list[i] = j

    return num_list

def sliceMesh(mesh, normal, origin):
    verts = np.array(mesh.vertices)
    faces = np.empty((len(verts), 3), dtype=int)
    for i in range(len(faces)):
        faces[i, :] = [mesh.polygons[i][0][0], mesh.polygons[i][1][0], mesh.polygons[i][2][0]]
    
    new_v, new_f = tm.intersections.slice_faces_plane(verts, faces, normal, origin)
    new_mesh = wf.WavefrontOBJ()
    for i in range(len(new_v)):
        new_mesh.vertices.append(new_v[i, :])
    for i in range(len(new_f)):
        new_mesh.polygons.append([[new_f[i,0], -1], [new_f[i,1], -1], [new_f[i,2], -1]])

    return new_mesh


