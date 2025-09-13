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
parser.add_argument("--largest_sphere", "-l", type=float, dest="largest_sphere", default=1e9, help="The radius of the largest sphere allowed")
parser.add_argument("--smallest_sphere", "-s", type=float, dest="smallest_sphere", default=1e-9, help="The radius of the smallest sphere allowed")
args = parser.parse_args(sys.argv[1:])

if args.ratio>1.0 or args.ratio<0.0:
    raise Exception("The spherical interpolation ration has to be between 0 and 1")

original_meshes = wf.load_obj(args.input_obj)
assert len(original_meshes) == 1, "This script handles OBJ with one mesh group only."
mesh = original_meshes[0]

p.connect(p.DIRECT)

############## IF OTHER CONVEX DECOMPOSITION PARAM NEEDED TO CHANGE: PLEASE DIRECTLY CHANGE THEM HERE #######################
p.vhacd(args.input_obj, args.convex_obj, args.convex_log, concavity=0.0025, alpha=0.04, 
        resolution=args.voxel_resolution, maxNumVerticesPerCH=args.max_hull_verts)
#############################################################################################################################
parts = wf.load_obj(args.convex_obj)#, triangulate=True)

# preprocess and sort parts by quality
for part in parts:
    # quality is defined by the ratio between the small_sphere and big_sphere
    util.updatePartQuality(part, args.largest_sphere, args.smallest_sphere, args.ratio)

# Now, subdivide bad-quality parts, until the number of parts reaches budget
while len(parts) < args.budget:
    # resort
    parts.sort(key=lambda x: x.quality)
    # find the worst-quality part
    worst_id = 0
    found = False
    # but if the expected size is already small enough, we move on to the next worst
    for i in range(len(parts)):
        big_sphere, small_sphere = util.getLargeSmallSphere(parts[i])
        expected_size = util.interpolateSphere(big_sphere, small_sphere, args.ratio).radius
        if expected_size > args.smallest_sphere:
            worst_id = i
            found = True
            print("Current number of parts: %d. Worst part is %d with quality %.4f and expected size %.4f" % (len(parts), worst_id, parts[worst_id].quality, expected_size))
            break
    if not found:
        print("All parts are small enough. Stopping at %d parts." % len(parts))
        break

    worst_part = parts[worst_id]
    # subdivide it into two parts, by cutting along the longest axis of this mesh
    bounding_box = util.bbox(worst_part.vertices)
    longest_axis = np.argmax(bounding_box[1,:] - bounding_box[0,:])
    axis_center = (bounding_box[1,longest_axis] + bounding_box[0,longest_axis]) / 2.0
    # cutting plane's center is the center of the bounding box
    plane_center = (bounding_box[1,:] - bounding_box[0,:]) / 2.0 + bounding_box[0,:]
    # cutting normal direction is along the longest axis
    plane_normal = np.array([0.0,0.0,0.0])
    plane_normal[longest_axis] = 1.0

    cutted_parts = []
    cutted_parts.append(worst_part)

    planes = [plane_center]
    normals = [plane_normal]
    # print("Cutting part %d along axis %d at %.4f" % (worst_id, longest_axis, axis_center))

    for i in range(len(planes)):
        plane = np.array(planes[i])
        normal = np.array(normals[i])
        this_cut_result = []
        for part in cutted_parts:
            halfA = util.sliceMesh(part, normal, plane)
            halfB = util.sliceMesh(part, -normal, plane)
            if len(halfA.vertices) > 0:
                this_cut_result.append(halfA)
            if len(halfB.vertices) > 0:
                this_cut_result.append(halfB)
        cutted_parts = this_cut_result

    # newly generated parts quality update
    for part in cutted_parts:
        util.updatePartQuality(part, args.largest_sphere, args.smallest_sphere, args.ratio)

    # replace the worst part by the two new parts
    parts.pop(worst_id)
    for part in cutted_parts:
        parts.append(part)
    

# assign spheres to each part
part_id = 0
xyzr = np.zeros((len(parts), 4))
for part in parts:

    big_sphere, small_sphere = util.getLargeSmallSphere(part)

    decomp_sphere = util.interpolateSphere(big_sphere, small_sphere, args.ratio)
    xyzr[part_id,:3] = decomp_sphere.center 
    xyzr[part_id,-1] = decomp_sphere.radius

    part_id += 1

np.savetxt(args.approx_csv, xyzr, header = "x,y,z,r", delimiter=",")

# The ball number which this vertex got assigned to.
# TODO: Temporarily disable optimization for better stability

# assign_list = util.findClosestSphere(mesh.vertices, xyzr)
# opt_spheres = opt.optimizeAsgdSpheresFromVert(mesh.vertices, xyzr, assign_list)
# np.savetxt(args.output_csv, opt_spheres, header = "x,y,z,r", delimiter=",")


