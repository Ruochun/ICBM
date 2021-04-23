import pybullet as p
#import pybullet_data as pd
import os
import wavefront as wf
import util
import optimization as opt
import numpy as np
import argparse, sys

parser = argparse.ArgumentParser(description=__doc__, formatter_class=
        argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("--input_obj", "-i", type=str, dest="input_obj", default="mesh.obj", help="File name of the .obj input")
parser.add_argument("--output_csv", "-o", type=str, dest="output_csv", default="result.csv", help="File name of the .csv output (after optimization)")
parser.add_argument("--approx_csv", "-a", type=str, dest="approx_csv", default="quick_approx.csv", help="File name of a quick approximation of the decomposition (before optimization)")
parser.add_argument("--convex_obj", "-c", type=str, dest="convex_obj", default="parts.obj", help="File name of the intermediate .obj convex shapes")
parser.add_argument("--convex_log", type=str, dest="convex_log", default="log.txt", help="File name of the intermediate convex decomposition logs")
parser.add_argument("--interpolate_ratio", "-r", type=float, dest="ratio", default=0.5,  
			help="Must be in [0,1]; set it close to 1 to make the final output spheres large, close to 0 to make them small")
parser.add_argument("--voxel_resolution", "-v", type=int, dest="voxel_resolution", default=50000,
			help="The resolution at which the convex decomposition takes place; larger number usually leads to more final spheres")
parser.add_argument("--max_hull_verts", type=int, dest="max_hull_verts", default=64, help="The max number of vertices of one convex hull")
parser.add_argument("--budget", "-b", type=int, dest="budget", default=5, help="The target number of spheres we want")
args = parser.parse_args(sys.argv[1:])

original_meshes = wf.load_obj(args.input_obj)
assert len(original_meshes) == 1, "This script handles OBJ with one mesh group only."
mesh = original_meshes[0]

p.connect(p.DIRECT)

############## IF OTHER CONVEX DECOMPOSITION PARAM NEEDED TO CHANGE: PLEASE DIRECTLY CHANGE THEM HERE #######################
p.vhacd(args.input_obj, args.convex_obj, args.convex_log, concavity=0.0025, alpha=0.04, 
        resolution=args.voxel_resolution, maxNumVerticesPerCH=args.max_hull_verts)
#############################################################################################################################

parts = wf.load_obj(args.convex_obj)#, triangulate=True)
parts.append(util.sliceMesh(parts[0], [0,0,1], [0,0,40]))
parts[0] = util.sliceMesh(parts[0], [0,0,-1], [0,0,40])
parts.append(util.sliceMesh(parts[0], [0,0,-1], [0,0,-40]))
parts[0] = util.sliceMesh(parts[0], [0,0,1], [0,0,-40])

xyzr = np.zeros((len(parts), 4))
part_id = 0
for part in parts:

    bounding_box = util.bbox(part.vertices)
    big_sphere = util.box2ball(bounding_box)

    mesh_center = util.coord_avg(part.vertices)
    small_sphere = util.inscribedSphereViaPointMesh(mesh_center, part)

    decomp_sphere = util.interpolateSphere(big_sphere, small_sphere, args.ratio)
    xyzr[part_id,:3] = decomp_sphere.center 
    xyzr[part_id,-1] = decomp_sphere.radius

    part_id += 1

np.savetxt(args.approx_csv, xyzr, header = "x,y,z,r", delimiter=",")

# The ball number which this vertex got assigned to.
assign_list = util.findClosestSphere(mesh.vertices, xyzr)
opt_spheres = opt.optimizeAsgdSpheresFromVert(mesh.vertices, xyzr, assign_list)
np.savetxt(args.output_csv, opt_spheres, header = "x,y,z,r", delimiter=",")


