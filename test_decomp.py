import pybullet as p
#import pybullet_data as pd
import os
import wavefront as wf
import util
import numpy as np
import argparse, sys

parser = argparse.ArgumentParser(description=__doc__, formatter_class=
        argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("--input_obj", "-i", type=str, dest="input_obj", default="mesh.obj", help="File name of the .obj input")
parser.add_argument("--output_csv", "-o", type=str, dest="output_csv", default="result.csv", help="File name of the .csv output")
parser.add_argument("--convex_obj", "-c", type=str, dest="convex_obj", default="parts.obj", help="File name of the intermediate .obj convex shapes")
parser.add_argument("--convex_log", type=str, dest="convex_log", default="log.txt", help="File name of the intermediate convex decomposition logs")
parser.add_argument("--interpolate_ratio", "-r", type=float, dest="ratio", default=0.5,  
			help="Must be in [0,1]; set it close to 1 to make the final output spheres large, close to 0 to make them small")
parser.add_argument("--voxel_resolution", "-v", type=int, dest="voxel_resolution", default=50000,
			help="The resolution at which the convex decomposition takes place; larger number usually leads to more final spheres")
args = parser.parse_args(sys.argv[1:])

p.connect(p.DIRECT)

############## IF OTHER CONVEX DECOMPOSITION PARAM NEEDED TO CHANGE: PLEASE DIRECTLY CHANGE THEM HERE #######################
p.vhacd(args.input_obj, args.convex_obj, args.convex_log, concavity=0.0025, alpha=0.04, resolution=args.voxel_resolution)
#############################################################################################################################

parts = wf.load_obj(args.convex_obj)#, triangulate=True)

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

np.savetxt(args.output_csv, xyzr, header = "x,y,z,r", delimiter=",")



