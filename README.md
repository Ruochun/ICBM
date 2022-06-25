# ICBM
Ice Cream Ball Maker

It makes overlapping icecream balls that fill up a mesh.

To use it, first install Pybullet and other utilities if going the convex_then_clump path:

pip3 install pybullet --upgrade --user

pip3 install Pillow

pip3 install trimesh

Then call the script, something like this (a bear.obj demo is included):

python3 convex_then_clump.py -i bear.obj -o result.csv -c parts.obj -v 1000000 -r 0.6

python3 convex_then_clump.py -i football_refined.obj -o result.csv -c parts.obj -v 1000000 -r 0.1

python3 convex_then_clump.py -i football_5_2.obj -o result.csv -c parts.obj -v 1000000 -r 0.1

There is an old implementation but I do not think it works well...:

python3 sample_then_clump.py -i football.obj -o result.csv -n 10000 -b 3

