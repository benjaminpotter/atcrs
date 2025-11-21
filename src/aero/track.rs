use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;

#[derive(Debug, Serialize, Deserialize)]
pub struct TrackResponse {
    /// Distance (in miles) flown as of the latest position point.
    /// Will include distance from the origin airport to the first position point.
    /// If the flight has been completed, will include the distance from the last
    /// position point to the destination airport.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub actual_distance: Option<i32>,

    /// Array of position data points for the flight
    pub positions: Vec<Position>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Position {
    /// Unique identifier assigned by FlightAware to the flight with this position.
    /// This field is only populated by the /flights/search/positions endpoint.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fa_flight_id: Option<String>,

    /// Aircraft altitude in hundreds of feet
    altitude: i32,

    /// C when the aircraft is climbing, D when descending, and - when the altitude is being maintained.
    pub altitude_change: AltitudeChange,

    /// Most recent groundspeed (knots)
    pub groundspeed: i32,

    /// Aircraft heading in degrees (0-360)
    #[serde(skip_serializing_if = "Option::is_none")]
    heading: Option<i32>,

    /// Most recent latitude position
    latitude: f64,

    /// Most recent longitude position
    longitude: f64,

    /// Time that position was received
    pub timestamp: DateTime<Utc>,

    /// Position update type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub update_type: Option<UpdateType>,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum AltitudeChange {
    /// Climbing
    C,
    /// Descending
    D,
    /// Maintaining
    #[serde(rename = "-")]
    M,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum UpdateType {
    /// Projected
    P,
    /// Oceanic
    O,
    /// Radar
    Z,
    /// ADS-B
    A,
    /// Multilateration
    M,
    /// Datalink
    D,
    /// Surface and near surface (ADS-B and ASDE-X)
    X,
    /// Space-based
    S,
    /// Virtual event
    V,
}

impl Position {
    pub fn altitude(&self) -> Length {
        Length::new::<foot>(self.altitude as f64 * 100.0)
    }

    pub fn heading(&self) -> Option<Angle> {
        self.heading
            .and_then(|heading| Some(Angle::new::<degree>(heading as f64)))
    }

    pub fn latitude(&self) -> Angle {
        Angle::new::<degree>(self.latitude)
    }

    pub fn longitude(&self) -> Angle {
        Angle::new::<degree>(self.longitude)
    }
}
