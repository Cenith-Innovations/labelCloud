import numpy as np
import open3d as o3d
import os

import json
import os
import math as m

import argparse



def files(path):
    # simple generator to iterate over only files in a folder
    for fname in os.listdir(path):
        if os.path.isfile(os.path.join(path, fname)):
            yield fname


def clouds_to_bins(cloud_dir, out_dir):

    os.makedirs(out_dir, exist_ok=True)
    for fname in files(cloud_dir):

        pcd_path = os.path.join(cloud_dir, fname)
        #out_path = os.path.join(out_dir, f'{idx:06d}.bin')
        out_path = os.path.join(out_dir, os.path.splitext(fname)[0] + '.bin')

        pcd = o3d.io.read_point_cloud(pcd_path)
        cloud = np.asarray(pcd.points, dtype='float32')
        colors = np.asarray(pcd.colors, dtype='float32')

        vel_cloud = np.hstack((cloud, colors))
        vel_cloud = vel_cloud[:, :4]

        out = open(out_path, 'wb')
        out.write(vel_cloud.tobytes())
        out.close()

        #idx += 1


def conv_labels(label_dir, out_dir):

    os.makedirs(out_dir, exist_ok=True)
    for fname in files(label_dir):

        f = open(os.path.join(label_dir, fname))
        label = json.load(f)
        drone = label['objects'][0]
        
        name = drone['name']
        height = drone['dimensions']['height']
        width = drone['dimensions']['width']
        length = drone['dimensions']['length']
        y = drone['centroid']['y']
        z = drone['centroid']['z']
        x = drone['centroid']['x']
        rz = drone['rotations']['z']
        # fix the height since no y rotation
        height = height + (length * m.sin(drone['rotations']['y']))
        
        obj_str = f'{name} 0 0 0 0 0 0 0 {height} {width} {length} {-1* y} {-1* z + (height/2)} {x} {-rz}'

        kitti_name = os.path.splitext(fname)[0] + '.txt'
        kitti_name = os.path.join(out_dir, kitti_name)
        kit_lab = open(kitti_name, 'w')
        kit_lab.write(obj_str)
        kit_lab.close()

def add_dep_args(args):
    if args.labels is None:
        args.labels = os.path.join(args.clouds, 'labels')



if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Convert from labeled Ouster files to KITTI clouds and labels')

    parser.add_argument('clouds', metavar='pcd_dir', type=str,
                        help='The directory with the *.pcd files')
    parser.add_argument('--labels', default=None,
                        help='The directory with centroid_rel format labels')
    parser.add_argument('-o', '--out_dir', default='./kitti',
                        help='directory for all the output data')

    #parser.add_argument('-s', '--start', default=0, help='number to start the file names at (for combining clips)')



    args = parser.parse_args()
    add_dep_args(args)

    # first convert the clouds to *.bins
    clouds_to_bins(args.clouds, os.path.join(args.out_dir, 'velodyne'))

    # convert labels
    conv_labels(args.labels, os.path.join(args.out_dir, 'label_2'))