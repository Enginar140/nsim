//! `nsim` command-line runner.

mod backends;
mod demo;

use std::process::ExitCode;

const USAGE: &str = "\
nsim — open-source robotics and UAV simulator

Usage:
  nsim demo [--physics <name>] [--seconds <n>]   headless drop test
  nsim backends                                  list physics backends
  nsim --version";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(msg) => {
            eprintln!("nsim: {msg}");
            ExitCode::from(2)
        }
    }
}

fn run(args: &[String]) -> Result<(), String> {
    match args.first().map(String::as_str) {
        None | Some("--help" | "-h") => {
            println!("{USAGE}");
            Ok(())
        }
        Some("--version" | "-V") => {
            println!("nsim {}", env!("CARGO_PKG_VERSION"));
            Ok(())
        }
        Some("backends") => {
            for b in backends::available() {
                println!("{:<10} {}", b.name, b.description);
            }
            Ok(())
        }
        Some("demo") => {
            let mut physics = "rapier".to_owned();
            let mut seconds = 5u32;
            let mut rest = args[1..].iter();
            while let Some(flag) = rest.next() {
                let value = rest.next().ok_or_else(|| format!("{flag} needs a value"))?;
                match flag.as_str() {
                    "--physics" => physics = value.clone(),
                    "--seconds" => {
                        seconds = value.parse().map_err(|_| {
                            format!("--seconds expects a whole number, got '{value}'")
                        })?;
                    }
                    other => return Err(format!("unknown option '{other}'\n\n{USAGE}")),
                }
            }
            demo::run(backends::create(&physics)?, seconds)
        }
        Some(other) => Err(format!("unknown command '{other}'\n\n{USAGE}")),
    }
}
