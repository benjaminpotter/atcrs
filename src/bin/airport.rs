use anyhow::Result;
use atcrs::aero::airport::*;
use clap::Parser;
use reqwest::Client;
use reqwest::Method;

#[derive(Parser)]
struct Cli {
    api_key: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    let code_icao = "KSEA";
    let url = format!(
        "https://aeroapi.flightaware.com/aeroapi/airports/{}",
        code_icao
    );
    let response: AirportsResponse = Client::new()
        .request(Method::GET, url)
        .header("x-apikey", cli.api_key)
        .send()
        .await?
        .json()
        .await?;

    let json = serde_json::to_string(&response)?;
    println!("{}", json);

    Ok(())
}
