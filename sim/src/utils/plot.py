#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.12.23
# Last modified by Tibor Völcker on 27.12.23
# Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
import io
import re

import matplotlib.pyplot as plt
import numpy as np


def read_test_output():
    # Read piped test output
    pipe = io.open(0, encoding="utf-8")
    content = pipe.read()

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
    axes.plot_surface(x, y, z, color=(1, 1, 1, 0.5))

    # Plot spin axis
    z = np.linspace(-7000000, 7000000, 100)
    axes.plot(np.zeros_like(z), np.zeros_like(z), z, color="gray")


if __name__ == "__main__":
    t, pos, vel = read_test_output()

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    # Plot earth
    plot_earth(ax)

    # Plot the trajectory
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label="Trajectory")

    # Set an equal aspect ratio
    ax.set_aspect("equal")

    # Show the plot
    plt.show()
