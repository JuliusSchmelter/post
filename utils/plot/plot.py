#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 22.12.23
# Last modified by Tibor Völcker on 12.05.24
# Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)
import re
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

import numpy as np
import pandas as pd
import pyvista as pv
from pyvista import plotting


@contextmanager
def read_file() -> Iterator[str]:
    """Reads simulation output from a file or from STDIN.

    The filename should be the first command line argument. If it is not given,
    STDIN is used.

    Yields:
        Iterator[str]: The content of the file or STDIN.
    """
    # If no cmd line arguments are used, we read from STDIN (filename=0)
    if len(sys.argv) < 2:
        filename = 0
        print("Reading from STDIN...", end=" ")
    else:
        filename = sys.argv[1]
        print(f"Reading from file {filename}...", end=" ")

    try:
        f = open(filename, "r", encoding="utf-8")
        yield f.read()
    finally:
        f.close()
        print("Done")


def parse_test_output(content: str) -> pd.DataFrame:
    """Parse the simulation output.

    Extract the phase, time, position and velocity and write it to a DF.

    Args:
        content (str): The simulation output.

    Returns:
        pd.DataFrame: The data.
    """
    # Define regular expressions for extracting each data point
    phase_pattern = re.compile(r"Starting Phase (\d+)")
    time_pattern = re.compile(r"Time: (\d+)")
    position_pattern = re.compile(
        r"Position:[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)"
    )
    velocity_pattern = re.compile(
        r"Velocity:[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)[\S\s]*?\│\s+(-?\d+)"
    )

    # Split the string into each phase (first item is before Phase 1)
    phases = phase_pattern.split(content)[1:]
    # Rearange to an array of phase index and content
    phases = np.reshape(phases, (-1, 2))

    dfs = []
    for i, phase in phases:
        # Find all matches in each phase
        time_matches = time_pattern.findall(phase)
        position_matches = position_pattern.findall(phase)
        velocity_matches = velocity_pattern.findall(phase)

        # Convert to one DF for this phase
        index = pd.MultiIndex.from_product([[i], time_matches], names=["Phase", "Time"])
        position = pd.DataFrame(position_matches).astype(float)
        velocity = pd.DataFrame(velocity_matches).astype(float)

        dfs.append(
            pd.concat([position, velocity], axis=1, keys=["Position", "Velocity"]).set_index(index)
        )

    # Concat all phases
    return pd.concat(dfs)


def plot(df: pd.DataFrame):
    """Plot the trajectory using PyVista.

    Args:
        df (pd.DataFrame): The simulation data.
    """
    pl = plotting.Plotter(lighting="none")

    # Add space background
    cubemap = pv.examples.download_cubemap_space_16k()
    pl.add_actor(cubemap.to_skybox())

    # Add earth model
    earth = pv.examples.planets.load_earth(radius=6378.166, lat_resolution=500, lon_resolution=250)
    earth_texture = pv.read_texture(Path(__file__).parent / "8k_earth_daymap.jpg")

    actor = pl.add_mesh(earth, texture=earth_texture, specular=0.1, smooth_shading=True)
    # Rotate earth so X-Y plane goes through Greenwich
    actor.rotate_z(180)

    # Add trajectory
    traj = pv.MultipleLines(df["Position"].values)
    colors = np.sqrt(np.square(df["Velocity"]).sum(axis=1)).values
    pl.add_mesh(
        traj,
        scalars=colors,
        line_width=4,
        render_lines_as_tubes=True,
        lighting=False,
        scalar_bar_args={"title": "Velocity in km/s", "fmt": "%.2f"},
        clim=[0, colors.max()],
    )

    # Add Phase labels
    phases = df.groupby("Phase")["Position"].first()
    pl.add_point_labels(
        phases.values,
        phases.index,
        point_color="gray",
        point_size=8,
        shape_opacity=0.5,
    )

    # Set camera above starting point and focus on start of trajectory
    start = df.iloc[0]["Position"].values
    pl.camera.position = start * 2
    pl.set_focus(start)

    # Add sunlight coming from above starting point
    light = plotting.Light()
    light.set_direction_angle(30, np.arctan2(start[1], start[0]) - 90)
    pl.add_light(light)

    pl.show()


if __name__ == "__main__":
    with read_file() as data_str:
        data = parse_test_output(data_str)

    # Convert from m to km
    data["Position"] /= 1e3
    data["Velocity"] /= 1e3

    plot(data)
