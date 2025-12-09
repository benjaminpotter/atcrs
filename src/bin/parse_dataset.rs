use anyhow::Context;
use anyhow::Result;
use atcrs::aero::airport::AirportsResponse;
use atcrs::aero::track::TrackResponse;
use chrono::prelude::*;
use serde::Serialize;
use serde::Serializer;
use serde::ser::SerializeStruct;
use sguaba::Coordinate;
use sguaba::math::RigidBodyTransform;
use sguaba::system;
use sguaba::systems::Wgs84;
use std::fs::File;
use std::io::BufRead;
use std::io::BufReader;
use std::path::Path;
use std::path::PathBuf;
use uom::si::f64::*;
use uom::si::length::kilometer;

system!(struct PlaneFrd using FRD);
system!(struct GroundEnu using ENU);

struct State {
    fa_flight_id: String,
    timestamp: DateTime<Utc>,
    position: Coordinate<GroundEnu>,
}

fn main() -> Result<()> {
    let airports_response = read_airports_response("airports_response.json")?;
    let max_haversine_distance = Length::new::<kilometer>(75.0);
    let dataset_path = PathBuf::from("/home/ben/downloads/flight_aware_dataset/flight_track");
    let output_path = "positions.csv";

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

    let mut writer = csv::Writer::from_path(&output_path)?;
    for (date, fa_flight_id) in handles {
        let mut data_path = dataset_path.clone();
        data_path.push(&date);
        data_path.push(&fa_flight_id);
        data_path.set_extension("json");

        println!("hit: {}", data_path.display());

        let track_response = read_track_response(&data_path)?;
        for position in track_response.positions {
            let wgs84 = Wgs84::builder()
                .longitude(position.longitude())
                .latitude(position.latitude())
                .expect("latitude is in [-90, 90]")
                .altitude(position.altitude())
                .build();

            if wgs84.haversine_distance_on_surface(&airport) <= max_haversine_distance {
                writer.serialize(State {
                    fa_flight_id: fa_flight_id.clone(),
                    timestamp: position.timestamp,
                    position: ecef_to_ground_enu.transform(Coordinate::from_wgs84(&wgs84)),
                })?;
            }
        }
    }

    Ok(())
}

impl Serialize for State {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("State", 5)?;
        state.serialize_field("fa_flight_id", &self.fa_flight_id)?;
        state.serialize_field("timestamp", &self.timestamp)?;
        state.serialize_field("enu_east_km", &self.position.enu_east().get::<kilometer>())?;
        state.serialize_field(
            "enu_north_km",
            &self.position.enu_north().get::<kilometer>(),
        )?;
        state.serialize_field("enu_up_km", &self.position.enu_up().get::<kilometer>())?;
        state.end()
    }
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
