#! /usr/bin/python

# Converts a 2D image to a 3D point cloud
# File formats:
# The depth maps are stored as 640x480 16-bit monochrome images in PNG format.
# The color images are stored as 640x480 8-bit RGB images in PNG format.
# The depth images are scaled by a factor of 5000, i.e. a value of 5000 in the depth image corresponds
# to 1 meter from the camera. A pixel value of 0 means missing value/no data.

# Ground truth trajectories
# Each line of the text file gives the timestamp, tx, ty, tz, qx, qy, qz, qw
# Where timestamp is the time since Unix epoch
# tx, ty, tz (floats) give the position of the optical center of the color camera with respect to the world origin
# as defined by the motion capture system.
# qx, qy, qz, qw (floats) give the orientation of the optical center of the color camera in the form of a unit
# quaternion with respect to the world origin as defined by the motion capture system.


import cv2
import numpy as np
import os
from PIL import Image # pip install pillow.

fx = 525.0  # focal length x
cx = 319.5  # optical center x
cy = 239.5  # optical center y
factor = 5000.0 # for the 16-bit PNG files. This needs to be a float!!!!!!!


def mult(q0, q1, q2, q3, r0, r1, r2, r3):
    return [r0*q0 - r1*q1 - r2*q2 - r3*q3,
            r0*q1 + r1*q0 - r2*q3 + r3*q2,
            r0*q2 + r1*q3 + r2*q0 - r3*q1,
            r0*q3 - r1*q2 + r2*q1 + r3*q0]

def conj(q0, q1, q2, q3):
    return [q0, -q1, -q2, -q3]

def norm(q0, q1, q2, q3):
    return q0**2 + q1**2 + q2**2 + q3**2

def inv(q0, q1, q2, q3):
    n = norm(q0, q1, q2, q3)
    c = conj(q0, q1, q2, q3)
    return [float(x) / n for x in c]

def rotate(x, y, z, w, p, q, r):
    tmp = mult(w, p, q, r, 0, x, y, z)
    i = inv(w, p, q, r)
    res = mult(tmp[0], tmp[1], tmp[2], tmp[3], i[0], i[1], i[2], i[3])
    return res[1:]

def magnitude(x, y, z):
    return (x**2 + y**2 + z**2)**(0.5)

def convert_file(depth_file, rgb_file, output_file, quaternion):
    depth = Image.open(depth_file).rotate(180)
    rgb = Image.open(rgb_file).rotate(180)
    if depth.mode != "I":
        raise Exception("Depth image is not in intensity format")

    points = []
    for v in range(depth.size[1]):
        for u in range(depth.size[0]):
            color = rgb.getpixel((u,v))
            Z = depth.getpixel((u, v)) / factor
            if Z == 0: continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fx
            # test = rotate(X, Y, Z, quaternion[3], quaternion[4], quaternion[5], quaternion[6])
            points.append("%f %f %f %d %d %d 1\n"%(X + quaternion[0],Y+ quaternion[1],Z+ quaternion[2],color[0],color[1],color[2]))

    file = open(output_file, "w")
    file.write("".join(points))
    file.close()


def read_file(name):
    with open(name) as f:
        content = f.readlines()
        return [x.strip().split()[1] for x in content]

def read_quaternion(name):
    with open(name) as f:
        content = f.readlines()
        output = []
        for line in content:
            words = line.split()
            output.append((float(words[1]),float(words[2]),float(words[3]),float(words[4]), float(words[5]), float(words[6]), float(words[7])))
        return output

# depth_files = read_file("data/depth.txt.assoc")
# rgb_files = read_file("data/rgb.txt.assoc")
quaternion = read_quaternion("data/groundtruth.txt.assoc")
#
#
# for idx, val in enumerate(depth_files):
#     print idx
#     convert_file("data/" + val, "data/" + rgb_files[idx], "data/output_q/" + val[6:], quaternion[idx])


print quaternion[0]
convert_file("data/depth/1.png", "data/rgb/1305031102.175304.png", "data/output_q/1.png", (0,0,0,0))

# names = []
# for filename in os.listdir("data/output_q/"):
#     names.append(filename + "\n")
#
#
# file = open("data/names_q.txt", "w")
# file.write("".join(names))
# file.close()

# for filename in os.listdir("data/rgb/"):
#     rgb_files.append(filename)
#
# for i, val in enumerate(depth_files):
#     convert_file("data/depth/" + val, "data/rgb/" + rgb_files[i], "data/output_color/" + val)


