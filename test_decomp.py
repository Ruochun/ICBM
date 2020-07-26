import pybullet as p
import pybullet_data as pd
import os
import wavefront as wf
import util
import numpy as np

csv_out = "result.csv"

p.connect(p.DIRECT)
name_in = "bear.obj"#os.path.join(pd.getDataPath(), "part.obj")
name_out = "part_vhacd2.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log, concavity=0.0000,alpha=0.04,resolution=500000 )

parts = wf.load_obj(name_out)#, triangulate=True)

xyzr = np.zeros((len(parts), 4))
part_id = 0
for part in parts:
    bounding_box = util.bbox(part.vertices)
    big_sphere = util.box2ball(bounding_box)

    mesh_center = util.coord_avg(part.vertices)
    small_sphere = util.inscribedSphereViaPointMesh(mesh_center, part)

    decomp_sphere = util.interpolateSphere(big_sphere, small_sphere, 0.5)
    xyzr[part_id,:3] = decomp_sphere.center 
    xyzr[part_id,-1] = decomp_sphere.radius

    part_id += 1

np.savetxt(csv_out, xyzr, header = "x,y,z,r", delimiter=",")



