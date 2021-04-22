import os
import wavefront as wf
import util
import optimization as opt
import sampling as sp
import numpy as np
import argparse, sys
import random

parser = argparse.ArgumentParser(description=__doc__, formatter_class=
        argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("--input_obj", "-i", type=str, dest="input_obj", default="mesh.obj", help="File name of the .obj input")
parser.add_argument("--output_csv", "-o", type=str, dest="output_csv", default="result.csv", help="File name of the .csv output (after optimization)")
parser.add_argument("--approx_csv", "-a", type=str, dest="approx_csv", default="quick_approx.csv", help="File name of a quick approximation of the decomposition (before optimization)")
parser.add_argument("--number_sample", "-n", type=int, dest="number_sample", default=50000, help="Number of the initial random sphere samples")
parser.add_argument("--budget", "-b", type=int, dest="budget", default=5, help="The target number of spheres we want")
args = parser.parse_args(sys.argv[1:])

original_meshes = wf.load_obj(args.input_obj)
assert len(original_meshes) == 1, "This script handles OBJ with one mesh group only."
mesh = original_meshes[0]

# randomly sample many spheres
initial_sample = np.empty((args.number_sample, 4))
tot_v = len(mesh.vertices)
for i in range(args.number_sample):
    verts = random.sample(range(tot_v), 4) # selected 4 random vertices from mesh
    initial_sample[i, :] = sp.sphereFrom4Points([mesh.vertices[j] for j in verts])

# pick the good spheres
mesh_bbox = util.bbox(mesh.vertices)
max_radius = np.sqrt(util.EucDistSq(mesh_bbox[0], mesh_bbox[1])) / 2.0
retain_list = []
for i in range(initial_sample.shape[0]):
    if not(np.isfinite(initial_sample[i, :]).all()): continue # git rid of nan
    if (initial_sample[i, 3] > max_radius): continue # get rid of large spheres
    if not(util.pointInBox(initial_sample[i, :3], mesh_bbox[0], mesh_bbox[1])): continue # get rid of out-of-bound spheres
    retain_list.append(i)
good_sample = initial_sample[retain_list, :]
del initial_sample

# merge similar spheres 
# do we do anything here???
xyzr = good_sample
del good_sample

# The ball number which this vertex got assigned to.
assign_list = util.findClosestSphere(mesh.vertices, xyzr)
unique, counts = np.unique(assign_list, return_counts=True)
if len(unique) > args.budget:
    sphere_popularity = dict(zip(unique, counts))
    sphere_popularity = sorted(sphere_popularity.items(), key=lambda item: item[1], reverse=True)
    retained = sphere_popularity[:args.budget]
    xyzr = xyzr[[item[0] for item in retained], :]
    assign_list = util.findClosestSphere(mesh.vertices, xyzr)
elif len(unique) < args.budget:
    print("WARNING! We just got "+str(len(unique))+" spheres in the clump and it is lower than the target amount.\n")
np.savetxt(args.approx_csv, xyzr, header = "x,y,z,r", delimiter=",")

opt_spheres = opt.optimizeAsgdSpheresFromVert(mesh.vertices, xyzr, assign_list)
np.savetxt(args.output_csv, opt_spheres, header = "x,y,z,r", delimiter=",")


