// Created by Tibor Völcker (tiborvoelcker@hotmail.de) on 12.11.23
// Last modified by Tibor Völcker on 12.05.24
// Copyright (c) 2023 Tibor Völcker (tiborvoelcker@hotmail.de)

use std::error::Error;

use clap::Parser;
use sim::Simulation;

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// The configuration file
    #[arg(short, long, value_name = "FILE")]
    config: std::path::PathBuf,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Cli::parse();

    let sim = Simulation::from_file(args.config)?;

    sim.run();

    Ok(())
}
