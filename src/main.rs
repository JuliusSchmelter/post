// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 24.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

//! # Welcome to the code documentation!
//! This is the main crate. Everything related to the simulation is included in
//! the [`sim`] crate.
//! Later, there will be also an `optimization` crate.
//!
//! A good starting point for discovering the code is the [`sim`] library
//! crate, or the [`sim::phase::Phase`] documentation. Simply click through the
//! links or the left panel to find out more.
//!
//! __Info:__ The references refer to the references in the user manual.
//! Although there should be ony one reference: [3] which is the formulation
//! manual.

use std::error::Error;

use clap::Parser;
use sim::Simulation;

/// Used to parse the CLI options, which is done by the external `clap` crate.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// The configuration file
    #[arg(short, long, value_name = "FILE")]
    config: std::path::PathBuf,
}

/// The main entry point for the CLI.
/// Parses the CLI options and calls the appropiate function
/// from the [`sim`] crate.
///
/// # Errors
/// The function errors if the configuration could not be parsed.
fn main() -> Result<(), Box<dyn Error>> {
    let args = Cli::parse();

    let sim = Simulation::from_file(args.config)?;

    sim.run();

    Ok(())
}
