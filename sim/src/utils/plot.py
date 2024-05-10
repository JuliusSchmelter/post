#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.12.23
# Last modified by Tibor Völcker on 10.05.24
# Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
import re
import sys
from pathlib import Path

import numpy as np
import pyvista as pv


def read_stdin():
    print("Reading from STDIN...", end="")
    content = sys.stdin.read()
    print(" Done.")
    return content


def read_file():
    if len(sys.argv) < 2:
        return False

    filename = sys.argv[1]
    print(f"Reading from file {filename}...", end="")
    content = open(filename, "r", encoding="utf-8").read()
    print(" Done.")
    return content


def parse_test_output(content):
    # Define regular expressions for extracting time, position, and velocity
    time_pattern = re.compile(r"Time: (\d+)")
    position_pattern = re.compile(
        r"Position:[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)"
    )
    velocity_pattern = re.compile(
        r"Velocity:[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)"
    )

    # Find all matches in the content
    time_matches = time_pattern.findall(content)
    position_matches = position_pattern.findall(content)
    velocity_matches = velocity_pattern.findall(content)

    time = np.array(time_matches).astype(float)
    position = np.array(position_matches).astype(float)
    velocity = np.array(velocity_matches).astype(float)

    return time, position, velocity


if __name__ == "__main__":
    content = read_file()
    if not content:
        content = read_stdin()

    t, pos, vel = parse_test_output(content)

    earth = pv.examples.planets.load_earth(radius=6378166, lat_resolution=500, lon_resolution=250)
    earth_texture = pv.read_texture(Path(__file__).parent / "8k_earth_daymap.jpg")

    pl = pv.Plotter()

    actor = pl.add_mesh(earth, texture=earth_texture)
    # Rotate earth so X-Y plane goes through Greenwich
    actor.rotate_z(180)

    traj = pv.MultipleLines(pos)
    pl.add_mesh(
        traj,
        color="g",
        line_width=5,
        render_lines_as_tubes=True,
    )
    pl.add_mesh(pos, color="r", point_size=6)

    # Set camera and focus
    pl.camera.position = pos[0] * 2
    pl.set_focus(pos[0])

    pl.show()
