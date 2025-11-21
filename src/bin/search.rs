use anyhow::Result;
use atcrs::aero::search::ArrivalsResponse;
use clap::Parser;
use reqwest::Client;
use reqwest::Method;
use std::fs::File;
use std::io::BufWriter;
use std::io::Write;
use std::path::PathBuf;

#[derive(Parser)]
struct Cli {
    api_key: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    let dataset_path = PathBuf::from("dataset/flight_info");
    std::fs::create_dir_all(&dataset_path)?;

    for (start, end) in vec![
        ("2016-01-11", "2016-01-12"),
        ("2016-01-12", "2016-01-13"),
        ("2016-01-13", "2016-01-14"),
        ("2016-01-14", "2016-01-15"),
    ] {
        let mut output_path = dataset_path.clone();
        output_path.push(&start);
        std::fs::create_dir_all(&output_path)?;

        let code_icao = "KSEA";
        let root_url = "https://aeroapi.flightaware.com/aeroapi";
        let url = format!(
            "{}/history/airports/{}/flights/arrivals",
            root_url, code_icao
        );

        let client = Client::new();

        println!("request: {} to {}", &start, &end);
        let mut response: ArrivalsResponse = client
            .request(Method::GET, &url)
            .header("x-apikey", &cli.api_key)
            .query(&[("start", start), ("end", end)])
            .send()
            .await?
            .json()
            .await?;

        while let Some(links) = response.links {
            for arrival in response.arrivals {
                let mut path = output_path.clone();
                path.push(&arrival.fa_flight_id);
                path.set_extension("json");

                let file = File::create(path)?;
                let mut writer = BufWriter::new(file);

                let serialized = serde_json::to_string(&arrival)?;
                writer.write_all(serialized.as_bytes())?;
            }

            let url = format!("{}{}", root_url, &links.next);
            response = client
                .request(Method::GET, &url)
                .header("x-apikey", &cli.api_key)
                .send()
                .await?
                .json()
                .await?;
        }
    }

    Ok(())
}
