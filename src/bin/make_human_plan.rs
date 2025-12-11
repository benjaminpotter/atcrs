use anyhow::Context;
use anyhow::Result;
use atcrs::aero::airport::AirportsResponse;
use atcrs::aero::track::TrackResponse;
use clap::Parser;
use sguaba::Coordinate;
use sguaba::math::RigidBodyTransform;
use sguaba::system;
use sguaba::systems::Wgs84;
use std::fs::File;
use std::io::BufReader;
use std::io::BufWriter;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use uom::si::angle::radian;
use uom::si::f64::*;
use uom::si::length::kilometer;

system!(struct PlaneFrd using FRD);
system!(struct GroundEnu using ENU);

#[derive(Parser)]
struct Cli {
    track_path: PathBuf,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let airports_response = read_airports_response("airports_response.json")?;
    let max_haversine_distance = Length::new::<kilometer>(75.0);
    let output_path = "human_plan.g";

    let airport = Wgs84::builder()
        .longitude(airports_response.longitude())
        .latitude(airports_response.latitude())
        .expect("latitude is in [-90, 90]")
        .altitude(airports_response.elevation())
        .build();

    let ecef_to_ground_enu =
        unsafe { RigidBodyTransform::<_, GroundEnu>::ecef_to_enu_at(&airport) };

    let track_response = read_track_response(cli.track_path)?;

    let positions: Vec<_> = track_response
        .positions
        .into_iter()
        .map(|position| {
            (
                Wgs84::builder()
                    .longitude(position.longitude())
                    .latitude(position.latitude())
                    .expect("latitude is in [-90, 90]")
                    .altitude(position.altitude())
                    .build(),
                position.heading(),
            )
        })
        .filter(|(wgs84, _)| {
            wgs84.haversine_distance_on_surface(&airport) <= max_haversine_distance
        })
        .map(|(wgs84, heading)| {
            (
                ecef_to_ground_enu.transform(Coordinate::from_wgs84(&wgs84)),
                heading,
            )
        })
        .map(|(position, heading)| {
            [
                position.enu_east().get::<kilometer>(),
                position.enu_north().get::<kilometer>(),
                position.enu_up().get::<kilometer>(),
                heading.unwrap().get::<radian>(),
            ]
        })
        .collect();

    if positions.len() < 3 {
        eprintln!("too small...");
        std::process::exit(1);
    }

    let file = File::create(&output_path).unwrap();
    let mut writer = BufWriter::new(file);

    for (ix, state) in positions.into_iter().enumerate() {
        let _ = writeln!(
            writer,
            "v {} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2} {}",
            ix, state[0], state[1], state[2], state[3], 0.0, 0.0, true,
        );
    }

    Ok(())
}

fn read_airports_response<P: AsRef<Path>>(file_path: P) -> Result<AirportsResponse> {
    let file = File::open(file_path).context("failed to open track response file")?;
    let reader = BufReader::new(file);
    serde_json::from_reader(reader).context("failed to parse track response file")
}

fn read_track_response<P: AsRef<Path>>(file_path: P) -> Result<TrackResponse> {
    let file = File::open(file_path).context("failed to open track response file")?;
    let reader = BufReader::new(file);
    serde_json::from_reader(reader).context("failed to parse track response file")
}

