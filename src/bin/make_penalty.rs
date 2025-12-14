use anyhow::Context;
use anyhow::Result;
use atcrs::PenaltyMap;
use atcrs::aero::airport::AirportsResponse;
use atcrs::aero::track::TrackResponse;
use sguaba::Coordinate;
use sguaba::math::RigidBodyTransform;
use sguaba::system;
use sguaba::systems::Wgs84;
use std::fs::File;
use std::io::BufRead;
use std::io::BufReader;
use std::path::Path;
use std::path::PathBuf;
use uom::si::angle::radian;
use uom::si::f64::*;
use uom::si::length::kilometer;

system!(struct PlaneFrd using FRD);
system!(struct GroundEnu using ENU);

fn main() -> Result<()> {
    let airports_response = read_airports_response("airports_response.json")?;
    let max_haversine_distance = Length::new::<kilometer>(75.0);
    let dataset_path = PathBuf::from("dataset/flight_track");
    let output_path = "penalty.csv";

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

    PenaltyMap::from_states(
        handles
            .into_iter()
            .map(|(date, fa_flight_id)| {
                let mut data_path = dataset_path.clone();
                data_path.push(&date);
                data_path.push(&fa_flight_id);
                data_path.set_extension("json");
                println!("hit: {}", data_path.display());

                data_path
            })
            .filter_map(|data_path| read_track_response(&data_path).ok())
            .flat_map(|track_response| {
                track_response
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
                            position.heading().unwrap(),
                        )
                    })
                    .filter(|(wgs84, _)| {
                        wgs84.haversine_distance_on_surface(&airport) <= max_haversine_distance
                    })
                    .map(|(wgs84, heading)| {
                        let coord = ecef_to_ground_enu.transform(Coordinate::from_wgs84(&wgs84));
                        [
                            coord.enu_east().get::<kilometer>(),
                            coord.enu_north().get::<kilometer>(),
                            coord.enu_up().get::<kilometer>(),
                            heading.get::<radian>(),
                        ]
                    })
            }),
    )
    .save(output_path)?;

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
