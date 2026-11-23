//! `nsim` command-line runner. Subcommands (run, replay, …) will be added as the
//! simulator is implemented.

use std::process::ExitCode;

fn main() -> ExitCode {
    let arg = std::env::args().nth(1);
    match arg.as_deref() {
        Some("--version" | "-V") => {
            println!("nsim {}", env!("CARGO_PKG_VERSION"));
            ExitCode::SUCCESS
        }
        None | Some("--help" | "-h") => {
            println!("nsim — open-source robotics and UAV simulator\n\nUsage: nsim --version");
            ExitCode::SUCCESS
        }
        Some(other) => {
            eprintln!("nsim: unknown argument '{other}' (try --help)");
            ExitCode::from(2)
        }
    }
}
