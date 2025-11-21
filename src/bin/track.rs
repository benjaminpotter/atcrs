use anyhow::Result;
use atcrs::aero::track::TrackResponse;
use clap::Parser;
use reqwest::Client;
use reqwest::Method;
use std::fs::File;
use std::io::BufRead;
use std::io::BufReader;
use std::io::BufWriter;
use std::io::Write;
use std::path::PathBuf;

// Return list of fa_flight_id from the dataset
// find dataset/flight_info -type f -name "*.json" | while read -l file; echo (basename (dirname $file)) (basename $file .json); end > fa_flight_ids.txt

#[derive(Parser)]
struct Cli {
    api_key: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    let dataset_path = PathBuf::from("dataset/flight_track");
    std::fs::create_dir_all(&dataset_path)?;

    let api_url = "https://aeroapi.flightaware.com/aeroapi";
    let client = Client::new();

    let reader = BufReader::new(std::io::stdin());
    for (i, result) in reader.lines().enumerate() {
        let line = result?;
        let flight_info: Vec<&str> = line.split_whitespace().collect();
        let date = flight_info[0];
        let fa_flight_id = flight_info[1];
        let endpoint = format!("{}/history/flights/{}/track", &api_url, &fa_flight_id);

        let response: TrackResponse = client
            .request(Method::GET, &endpoint)
            .header("x-apikey", &cli.api_key)
            .send()
            .await?
            .json()
            .await?;

        println!("[{}/?] hit: {}", i, &fa_flight_id);

        let mut path = dataset_path.clone();
        path.push(&date);
        std::fs::create_dir_all(&path)?;

        path.push(&fa_flight_id);
        path.set_extension("json");

        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);

        let serialized = serde_json::to_string(&response)?;
        writer.write_all(serialized.as_bytes())?;
    }

    Ok(())
}
