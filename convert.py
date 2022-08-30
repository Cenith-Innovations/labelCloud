#!/usr/bin/env python
# coding: utf-8

from ouster import client, pcap
from ouster.sdk.examples.colormaps import colorize, normalize
import ouster.client._utils as outils

import os

import numpy as np
import matplotlib.pyplot as plt

from more_itertools import nth
from itertools import islice
import open3d as o3d

import argparse

def import_pcap(pcap_path, metadata_path):

    # open the json file first for sensor info
    with open(metadata_path, 'r') as f:
        metadata = client.SensorInfo(f.read())


    return metadata, pcap.Pcap(pcap_path, metadata)


def pcap_to_cloud(source, metadata, num, out_dir, file_base, file_ext, field_idx) -> None:
    
    "Write scans from a pcap to pcd files (one per lidar scan)."

    fields = [client.ChanField.RANGE, client.ChanField.SIGNAL, client.ChanField.REFLECTIVITY, client.ChanField.NEAR_IR]
    field = fields[field_idx]


    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # precompute xyz look up table to save computation in a loop
    xyzlut = client.XYZLut(metadata)

    # create an iterator of LidarScans from pcap and bound it if num is specified
    scans = iter(client.Scans(source))
    if num:
        scans = islice(scans, num)

    for idx, scan in enumerate(scans):
        # project the ranges to cartesian
        xyz = xyzlut(scan)
        # create a point clous object
        pcd = o3d.geometry.PointCloud()
        # populate the clous with projexted points
        pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))

        # add the color information from the selected field
        color_data = scan.field(field).astype(float)

        # These lines are in the docs but dont seem to do anything
        #outils.BeamUniformityCorrector()(color_data) # fix beam 'streak' artifacts
        #outils.AutoExposure()(color_data) # change exposure (probably would be similar to normalize)

        color_data = normalize(color_data, .1) # the second arg is the percent at which clip the data's extremes
        # Chose a pretty cmap to make the channel look nice cmaps here: https://matplotlib.org/stable/tutorials/colors/colormaps.html
        cm = plt.get_cmap('plasma')
        color_img = cm(color_data)[:, :, :3] # drop the a in rgba
        # add the color datas in
        pcd.colors = o3d.utility.Vector3dVector(color_img.reshape(-1, 3))


        out_path = os.path.join(out_dir, f'{file_base}{idx:06d}.{file_ext}')
        print(f'write frame #{idx} to file: {out_path}')

        o3d.io.write_point_cloud(out_path, pcd)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Convert Ouster pcap file')

    parser.add_argument('pcap', metavar='*.pcap', type=str,
                        help='The pcap file')
    parser.add_argument('json', metavar='*.json', type=str,
                        help='The metadata')
    parser.add_argument('-n', '--num', default=0,
                        help='How many frames to export')
    parser.add_argument('-f', '--file_name', default='',
                        help='The file name')
    parser.add_argument('-e', '--ext', default='pcd',
                        help='The file extention')
    parser.add_argument('-d', '--dir', default='../clouds',
                        help='the out directory')
    parser.add_argument('--field', default=2,
                    help='''The field to use in coloring the points:\n
                        0= RANGE\n
                        1= SIGNAL\n
                        2= REFLECTIVITY\n
                        3= NEAR_IR''')


    args = parser.parse_args()

    metadata, source = import_pcap(args.pcap, args.json)
    pcap_to_cloud(source, metadata, args.num, args.dir, args.file_name, args.ext, args.field)