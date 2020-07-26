import pybullet as p
import pybullet_data as pd
import os
import wavefront as wf
import util
import numpy as np

csv_out = "result.csv"

p.connect(p.DIRECT)
name_in = "part.obj"#os.path.join(pd.getDataPath(), "part.obj")
name_out = "part_vhacd2.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=50000 )

parts = wf.load_obj(name_out)

xyzr = np.zeros((len(parts), 4))
part_id = 0
for part in parts:
    bounding_box = util.bbox(part.vertices)
    big_sphere = util.box2ball(bounding_box)

    mesh_center = util.coord_avg(part.vertices)
    
    xyzr[part_id,:3] = big_sphere.center 
    xyzr[part_id,-1] = big_sphere.radius

    part_id += 1

np.savetxt(csv_out, xyzr, header = "x,y,z,r", delimiter=",")



