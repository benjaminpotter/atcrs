use anyhow::Context;
use anyhow::Result;
use atcrs::aero::airport::AirportsResponse;
use atcrs::aero::track::TrackResponse;
use chrono::prelude::*;
use dubins_paths::DubinsPath;
use serde::Serialize;
use serde::Serializer;
use serde::ser::SerializeStruct;
use sguaba::Coordinate;
use sguaba::math::RigidBodyTransform;
use sguaba::system;
use sguaba::systems::Wgs84;
use std::f64::consts::TAU;
use std::fs::File;
use std::io::BufRead;
use std::io::BufReader;
use std::io::BufWriter;
use std::path::Path;
use std::path::PathBuf;
use uom::si::angle::radian;
use uom::si::f64::*;
use uom::si::length::kilometer;

system!(struct PlaneFrd using FRD);
system!(struct GroundEnu using ENU);

#[derive(serde::Serialize)]
struct HumanResult {
    fa_flight_id: String,
    path_length_ecld: f64,
    path_length_dbns: f64,
}

fn main() -> Result<()> {
    let airports_response = read_airports_response("airports_response.json")?;
    let max_haversine_distance = Length::new::<kilometer>(75.0);
    let dataset_path = PathBuf::from("/home/ben/downloads/flight_aware_dataset/flight_track");
    let output_path = "human_results.csv";

    let airport = Wgs84::builder()
        .longitude(airports_response.longitude())
        .latitude(airports_response.latitude())
        .expect("latitude is in [-90, 90]")
        .altitude(airports_response.elevation())
        .build();

    let ecef_to_ground_enu =
        unsafe { RigidBodyTransform::<_, GroundEnu>::ecef_to_enu_at(&airport) };

    let reader = BufReader::new(std::io::stdin());
    let handles: Vec<(String, String)> = reader
        .lines()
        .flat_map(|result| {
            let line = result.ok()?;
            let mut split = line.split_whitespace();
            Some((split.next()?.to_string(), split.next()?.to_string()))
        })
        .collect();

    let mut writer = csv::Writer::from_path(output_path)?;
    for (date, fa_flight_id) in handles {
        let mut data_path = dataset_path.clone();
        data_path.push(&date);
        data_path.push(&fa_flight_id);
        data_path.set_extension("json");

        let Ok(track_response) = read_track_response(&data_path) else {
            eprintln!("skipping: could not read '{}'", data_path.display());
            continue;
        };

        println!("hit: {}", data_path.display());

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
            continue;
        }

        let mut path_length_ecld = 0.0;
        let mut path_length_dbns = 0.0;
        for window in positions.windows(2) {
            let &[s0, s1] = window else {
                eprintln!("windows failed");
                continue;
            };

            path_length_ecld += ecld(&s0, &s1);
            path_length_dbns += dbns(&s0, &s1);
        }

        writer.serialize(&HumanResult {
            fa_flight_id: fa_flight_id.clone(),
            path_length_ecld,
            path_length_dbns,
        })?;
    }

    Ok(())
}

fn angle_diff(a: f64, b: f64) -> f64 {
    // Compute the absolute difference
    let diff = (a - b).abs();

    // Return the smaller of the difference or its complement to a full circle
    diff.min(TAU - diff)
}

fn ecld(s0: &[f64; 4], s1: &[f64; 4]) -> f64 {
    ((s1[0] - s0[0]).powf(2.0)
        + (s1[1] - s0[1]).powf(2.0)
        + (s1[2] - s0[2]).powf(2.0)
        + (angle_diff(s1[3], s0[3])).powf(2.0))
    .sqrt()
}

fn dbns(s0: &[f64; 4], s1: &[f64; 4]) -> f64 {
    let max_turn_rate = 0.025;
    let xy_velocity = 0.1;
    let turn_radius = xy_velocity / max_turn_rate;

    let q0 = [s0[0], s0[1], s0[3]].into();
    let q1 = [s1[0], s1[1], s1[3]].into();
    let shortest_path = DubinsPath::shortest_from(q0, q1, turn_radius).unwrap();
    let xy_dist = shortest_path.length();
    let alt_change = s0[2] - s1[2];

    (xy_dist.powf(2.) + alt_change.powf(2.)).sqrt()
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
