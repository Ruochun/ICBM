 # wavefront.py
import numpy as np

class WavefrontOBJ:
    def __init__( self, default_mtl='default_mtl' ):
        self.path      = None               # path of loaded object
        self.mtllibs   = []                 # .mtl files references via mtllib
        self.mtls      = [ default_mtl ]    # materials referenced
        self.mtlid     = []                 # indices into self.mtls for each polygon
        self.vertices  = []                 # vertices as an Nx3 or Nx6 array (per vtx colors)
        self.normals   = []                 # normals
        self.texcoords = []                 # texture coordinates
        self.polygons  = []                 # M*Nv*3 array, Nv=# of vertices, stored as vid,tid,nid (-1 for N/A)

def load_obj( filename: str, default_mtl='default_mtl', triangulate=False ) -> WavefrontOBJ:
    """Reads a .obj file from disk and returns a WavefrontOBJ instance

    Handles only very rudimentary reading and contains no error handling!

    Does not handle:
        - relative indexing
        - lines, splines, beziers, etc.
    """
    # parses a vertex record as either vid, vid/tid, vid//nid or vid/tid/nid
    # and returns a 3-tuple where unparsed values are replaced with -1
    def parse_vertex( vstr ,v,vn,vt):
        vals = vstr.split('/')
        vid = int(vals[0])-1-v
        tid = int(vals[1])-1-vt if len(vals) > 1 and vals[1] else -1
        nid = int(vals[2])-1-vn if len(vals) > 2 else -1
        return (vid,tid,nid)

    obj = []
    obj_ind = 0
    v_count = 0
    vn_count = 0
    vt_count = 0
    v_offset = 0
    vn_offset = 0
    vt_offset = 0

    with open( filename, 'r' ) as objf:
        obj.append(WavefrontOBJ(default_mtl=default_mtl))
        obj[obj_ind].path = filename
        cur_mat = obj[obj_ind].mtls.index(default_mtl)
        for line in objf:
            toks = line.split()
            if not toks:
                continue
            if (toks[0] == 'o') and (v_count+vn_count+vt_count>0):
                obj.append(WavefrontOBJ(default_mtl=default_mtl))
                obj_ind += 1
                obj[obj_ind].path = filename
                cur_mat = obj[obj_ind].mtls.index(default_mtl)
                v_offset = v_count
                vn_offset = vn_count
                vt_offset = vt_count
            elif toks[0] == 'v':
                v_count += 1
                obj[obj_ind].vertices.append( [ float(v) for v in toks[1:]] )
            elif toks[0] == 'vn':
                vn_count += 1
                obj[obj_ind].normals.append( [ float(v) for v in toks[1:]] )
            elif toks[0] == 'vt':
                vt_count += 1
                obj[obj_ind].texcoords.append( [ float(v) for v in toks[1:]] )
            elif toks[0] == 'f':
                poly = [ parse_vertex(vstr,v_offset,vn_offset,vt_offset) for vstr in toks[1:] ]
                if triangulate:
                    for i in range(2,len(poly)):
                        obj[obj_ind].mtlid.append( cur_mat )
                        obj[obj_ind].polygons.append( (poly[0], poly[i-1], poly[i] ) )
                else:
                    obj[obj_ind].mtlid.append(cur_mat)
                    obj[obj_ind].polygons.append( poly )
            elif toks[0] == 'mtllib':
                obj[obj_ind].mtllibs.append( toks[1] )
            elif toks[0] == 'usemtl':
                if toks[1] not in obj[obj_ind].mtls:
                    obj[obj_ind].mtls.append(toks[1])
                cur_mat = obj[obj_ind].mtls.index( toks[1] )
        return obj

# save function disabled due to that file may contain multiple objects
"""
def save_obj( obj: WavefrontOBJ, filename: str ):
    with open( filename, 'w' ) as ofile:
        for mlib in obj.mtllibs:
            ofile.write('mtllib {}\n'.format(mlib))
        for vtx in obj.vertices:
            ofile.write('v '+' '.join(['{}'.format(v) for v in vtx])+'\n')
        for tex in obj.texcoords:
            ofile.write('vt '+' '.join(['{}'.format(vt) for vt in tex])+'\n')
        for nrm in obj.normals:
            ofile.write('vn '+' '.join(['{}'.format(vn) for vn in nrm])+'\n')
        if not obj.mtlid:
            obj.mtlid = [-1] * len(obj.polygons)
        poly_idx = np.argsort( np.array( obj.mtlid ) )
        cur_mat = -1
        for pid in poly_idx:
            if obj.mtlid[pid] != cur_mat:
                cur_mat = obj.mtlid[pid]
                ofile.write('usemtl {}\n'.format(obj.mtls[cur_mat]))
            pstr = 'f '
            for v in obj.polygons[pid]:
                # UGLY!
                vstr = '{}/{}/{} '.format(v[0]+1,v[1]+1 if v[1] >= 0 else 'X', v[2]+1 if v[2] >= 0 else 'X' )
                vstr = vstr.replace('/X/','//').replace('/X ', ' ')
                pstr += vstr
            ofile.write( pstr+'\n')
"""


