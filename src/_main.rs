use anyhow::Context;
use anyhow::Result;
use atcrs::aero::airport::AirportsResponse;
use serde::Serialize;
use serde::Serializer;
use serde::ser::SerializeStruct;
use sguaba::Coordinate;
use sguaba::math::RigidBodyTransform;
use sguaba::system;
use sguaba::systems::Wgs84;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use uom::si::f64::*;
use uom::si::length::kilometer;

system!(struct PlaneFrd using FRD);
system!(struct GroundEnu using ENU);

fn main() -> Result<()> {
    let airports_response = read_airports_response("airports_response.json")?;
    let track_response = read_track_response("track_response.json")?;
    let max_haversine_distance = Length::new::<kilometer>(30.0);

    let airport = Wgs84::builder()
        .longitude(airports_response.longitude())
        .latitude(airports_response.latitude())
        .expect("latitude is in [-90, 90]")
        .altitude(airports_response.elevation())
        .build();

    let ecef_to_ground_enu =
        unsafe { RigidBodyTransform::<_, GroundEnu>::ecef_to_enu_at(&airport) };

    let mut writer = csv::Writer::from_path("positions.csv")?;
    for position in track_response.positions {
        let wgs84 = Wgs84::builder()
            .longitude(position.longitude())
            .latitude(position.latitude())
            .expect("latitude is in [-90, 90]")
            .altitude(position.altitude())
            .build();

        if wgs84.haversine_distance_on_surface(&airport) <= max_haversine_distance {
            writer.serialize(State {
                position: ecef_to_ground_enu.transform(Coordinate::from_wgs84(&wgs84)),
            })?;
        }
    }

    Ok(())
}

struct State {
    position: Coordinate<GroundEnu>,
}

impl Serialize for State {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("State", 3)?;
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
