use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct ArrivalsResponse {
    pub links: Option<Links>,
    pub num_pages: i32,
    pub arrivals: Vec<Arrival>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Links {
    pub next: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Arrival {
    pub ident: String,
    pub ident_icao: Option<String>,
    pub ident_iata: Option<String>,
    pub actual_runway_off: Option<String>,
    pub actual_runway_on: Option<String>,
    pub fa_flight_id: String,
    pub operator: Option<String>,
    pub operator_icao: Option<String>,
    pub operator_iata: Option<String>,
    pub flight_number: Option<String>,
    pub registration: Option<String>,
    pub atc_ident: Option<String>,
    pub inbound_fa_flight_id: Option<String>,
    pub codeshares: Vec<String>,
    pub codeshares_iata: Vec<String>,
    pub blocked: bool,
    pub diverted: bool,
    pub cancelled: bool,
    pub position_only: bool,
    pub origin: FlightAirportRef,
    pub destination: FlightAirportRef,
    pub departure_delay: Option<i32>,
    pub arrival_delay: Option<i32>,
    pub filed_ete: Option<i32>,
    pub progress_percent: Option<i32>,
    pub status: String,
    pub aircraft_type: Option<String>,
    pub route_distance: Option<i32>,
    pub filed_airspeed: Option<i32>,
    pub filed_altitude: Option<i32>,
    pub route: Option<String>,
    pub baggage_claim: Option<String>,
    pub seats_cabin_business: Option<i32>,
    pub seats_cabin_coach: Option<i32>,
    pub seats_cabin_first: Option<i32>,
    pub gate_origin: Option<String>,
    pub gate_destination: Option<String>,
    pub terminal_origin: Option<String>,
    pub terminal_destination: Option<String>,
    #[serde(rename = "type")]
    pub flight_type: FlightType,
    pub scheduled_out: Option<DateTime<Utc>>,
    pub estimated_out: Option<DateTime<Utc>>,
    pub actual_out: Option<DateTime<Utc>>,
    pub scheduled_off: Option<DateTime<Utc>>,
    pub estimated_off: Option<DateTime<Utc>>,
    pub actual_off: Option<DateTime<Utc>>,
    pub scheduled_on: Option<DateTime<Utc>>,
    pub estimated_on: Option<DateTime<Utc>>,
    pub actual_on: Option<DateTime<Utc>>,
    pub scheduled_in: Option<DateTime<Utc>>,
    pub estimated_in: Option<DateTime<Utc>>,
    pub actual_in: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct FlightAirportRef {
    pub code: Option<String>,
    pub code_icao: Option<String>,
    pub code_iata: Option<String>,
    pub code_lid: Option<String>,
    pub timezone: Option<String>,
    pub name: Option<String>,
    pub city: Option<String>,
    pub airport_info_url: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum FlightType {
    #[serde(rename = "General_Aviation")]
    GeneralAviation,
    Airline,
}
