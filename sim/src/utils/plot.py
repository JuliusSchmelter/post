#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.12.23
# Last modified by Tibor Völcker on 11.03.24
# Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
import re
import sys

import numpy as np
import pyvista
from pyvista import examples


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


def plot_earth(axes):
    radius = 6378166

    # Make data
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = radius * np.outer(np.cos(u), np.sin(v))
    y = radius * np.outer(np.sin(u), np.sin(v))
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v))

    # Plot the surface
    axes.plot_surface(x, y, z, color=(1, 1, 1, 0))

    # Plot spin axis
    z = np.linspace(-7000000, 7000000, 100)
    axes.plot(np.zeros_like(z), np.zeros_like(z), z, color="red")


if __name__ == "__main__":
    content = read_file()
    if not content:
        content = read_stdin()

    t, pos, vel = parse_test_output(content)

    traj = pyvista.MultipleLines(pos)

    earth = examples.planets.load_earth(radius=6378166)
    earth_texture = examples.load_globe_texture()

    pl = pyvista.Plotter(lighting="none")

    pl.add_mesh(earth, texture=earth_texture)
    pl.add_mesh(traj, color="r", line_width=10)

    pl.set_focus(pos[0])

    pl.show()
