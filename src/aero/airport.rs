use serde::{Deserialize, Serialize};
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::foot;

#[derive(Debug, Serialize, Deserialize)]
pub struct AirportsResponse {
    pub airport_code: String,
    pub code_icao: Option<String>,
    pub code_iata: Option<String>,
    pub code_lid: Option<String>,
    #[serde(rename = "alternate_ident")]
    #[deprecated(note = "Use code_iata or code_lid instead")]
    pub alternate_ident: Option<String>,
    pub name: String,
    #[serde(rename = "type")]
    pub airport_type: Option<AirportType>,
    elevation: f64,
    pub city: String,
    pub state: String,
    longitude: f64,
    latitude: f64,
    pub timezone: String,
    pub country_code: String,
    pub wiki_url: Option<String>,
    pub airport_flights_url: String,
    pub alternatives: Vec<AirportAlternative>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct AirportAlternative {
    pub airport_code: String,
    pub code_icao: Option<String>,
    pub code_iata: Option<String>,
    pub code_lid: Option<String>,
    #[serde(rename = "alternate_ident")]
    #[deprecated(note = "Use code_iata or code_lid instead")]
    pub alternate_ident: Option<String>,
    pub name: String,
    #[serde(rename = "type")]
    pub airport_type: Option<AirportType>,
    pub elevation: f64,
    pub city: String,
    pub state: String,
    pub longitude: f64,
    pub latitude: f64,
    pub timezone: String,
    pub country_code: String,
    pub wiki_url: Option<String>,
    pub airport_flights_url: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum AirportType {
    Airport,
    Heliport,
    #[serde(rename = "Seaplane Base")]
    SeaplaneBase,
    Ultralight,
    Stolport,
    Gliderport,
    Balloonport,
}

impl AirportsResponse {
    pub fn elevation(&self) -> Length {
        Length::new::<foot>(self.elevation as f64)
    }

    pub fn latitude(&self) -> Angle {
        Angle::new::<degree>(self.latitude)
    }

    pub fn longitude(&self) -> Angle {
        Angle::new::<degree>(self.longitude)
    }
}
