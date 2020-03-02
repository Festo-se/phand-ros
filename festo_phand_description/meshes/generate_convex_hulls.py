#!/usr/bin/env python

import sys
import os
import subprocess

# Script taken from doing the needed operation
# (Filters > Remeshing, Simplification and Reconstruction >
# Quadric Edge Collapse Decimation, with parameters:
# 0.9 percentage reduction (10%), 0.3 Quality threshold (70%)
# Target number of faces is ignored with those parameters
# conserving face normals, planar simplification and
# post-simplimfication cleaning)
# And going to Filter > Show current filter script

# To genrete all collision meshes use :  find . | grep .STL | xargs  ./generate_convex_hulls.py remove_old


filter_script_mlx = """<!DOCTYPE FilterScript>
<FilterScript>
 <filter name="Convex Hull">
  <Param type="RichBool" value="true" name="reorient"/>
 </filter>
</FilterScript>
"""


def create_tmp_filter_file(filename='filter_file_tmp.mlx'):
    with open('/tmp/' + filename, 'w') as f:
        f.write(filter_script_mlx)
    return '/tmp/' + filename


def apply_meshlab_filter(in_file, out_file,
                         filter_script_path=create_tmp_filter_file()):
    # Add input mesh
    command = "meshlabserver -i " + in_file
    # Add the filter script
    command += " -s " + filter_script_path
    # Add the output filename and output flags
    command += " -o " + out_file + " -om vn fn"
    # Execute command
    print "Going to execute: " + command
    output = subprocess.check_output(command, shell=True)
    last_line = output.splitlines()[-1]
    print
    print "Done:"
    print in_file + " > " + out_file + ": " + last_line


if __name__ == '__main__':

    print sys.argv[1:]

    for mesh in sys.argv[2:]:

        in_mesh = mesh
        filename = in_mesh.split('/')[-1]
        print "Input mesh: " + in_mesh + " (filename: " + filename + ")"

        out_mesh = in_mesh[:-4] + "_convex.STL"

        if os.path.isfile(out_mesh) and sys.argv[2] == "remove_old":
            os.remove(out_mesh)

        apply_meshlab_filter(in_mesh, out_mesh)

        print "Done reducing" + out_mesh