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
use uom::si::angle::radian;
use uom::si::f64::*;
use uom::si::length::kilometer;

system!(struct PlaneFrd using FRD);
system!(struct GroundEnu using ENU);

struct Trial {
    fa_flight_id: String,
    initial_position: Coordinate<GroundEnu>,
    initial_bearing: f64,
    target_position: Coordinate<GroundEnu>,
    target_bearing: f64,
}

fn main() -> Result<()> {
    let airports_response = read_airports_response("airports_response.json")?;
    let max_haversine_distance = Length::new::<kilometer>(75.0);
    let dataset_path = PathBuf::from("/home/ben/downloads/flight_aware_dataset/flight_track");
    let output_path = "trials";

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

    for (date, fa_flight_id) in handles {
        let mut data_path = dataset_path.clone();
        data_path.push(&date);
        data_path.push(&fa_flight_id);
        data_path.set_extension("json");

        let Ok(track_response) = read_track_response(&data_path) else {
            eprintln!("skipping {}: could not read", data_path.display());
            continue;
        };

        println!("hit: {}", data_path.display());

        let positions: Vec<_> = track_response
            .positions
            .into_iter()
            .map(|position| {
                Wgs84::builder()
                    .longitude(position.longitude())
                    .latitude(position.latitude())
                    .expect("latitude is in [-90, 90]")
                    .altitude(position.altitude())
                    .build()
            })
            .filter(|wgs84| wgs84.haversine_distance_on_surface(&airport) <= max_haversine_distance)
            .map(|wgs84| ecef_to_ground_enu.transform(Coordinate::from_wgs84(&wgs84)))
            .collect();

        if positions.len() < 3 {
            eprintln!("too small...");
            continue;
        }

        let initial_position = positions[0];
        let next_position = positions[1];
        let initial_bearing = bearing_between(initial_position, next_position);

        let target_position = positions[positions.len() - 1];
        let prev_position = positions[positions.len() - 2];
        let target_bearing = bearing_between(prev_position, target_position);

        let path = format!("{output_path}/{fa_flight_id}.json");
        let file = File::create(path)?;
        serde_json::to_writer(
            file,
            &Trial {
                fa_flight_id: fa_flight_id.clone(),
                initial_position,
                initial_bearing,
                target_position,
                target_bearing,
            },
        )?;
    }

    Ok(())
}

fn bearing_between(a: Coordinate<GroundEnu>, b: Coordinate<GroundEnu>) -> f64 {
    let dx = b.enu_east() - a.enu_east();
    let dy = b.enu_north() - a.enu_north();

    dy.atan2(dx).get::<radian>()
}

#[test]
fn test_bearing_between() {
    use sguaba::coordinate;
    use std::f64::consts::FRAC_PI_4;
    use uom::ConstZero;

    let one_km = Length::new::<kilometer>(1.);
    let a = coordinate!(e = Length::ZERO, n = Length::ZERO, u = Length::ZERO);
    let b = coordinate!(e = one_km, n = one_km, u = Length::ZERO);

    let bearing = bearing_between(a, b);
    println!("{}", bearing.to_degrees());
    assert!(bearing == FRAC_PI_4);
}

impl Serialize for Trial {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("State", 5)?;
        state.serialize_field("fa_flight_id", &self.fa_flight_id)?;

        state.serialize_field(
            "initial_enu_east_km",
            &self.initial_position.enu_east().get::<kilometer>(),
        )?;
        state.serialize_field(
            "initial_enu_north_km",
            &self.initial_position.enu_north().get::<kilometer>(),
        )?;
        state.serialize_field(
            "initial_enu_up_km",
            &self.initial_position.enu_up().get::<kilometer>(),
        )?;
        state.serialize_field("initial_bearing", &self.initial_bearing)?;

        state.serialize_field(
            "target_enu_east_km",
            &self.target_position.enu_east().get::<kilometer>(),
        )?;
        state.serialize_field(
            "target_enu_north_km",
            &self.target_position.enu_north().get::<kilometer>(),
        )?;
        state.serialize_field(
            "target_enu_up_km",
            &self.target_position.enu_up().get::<kilometer>(),
        )?;
        state.serialize_field("target_bearing", &self.target_bearing)?;

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
