use anyhow::Result;
use anyhow::anyhow;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde::de;
use serde::de::MapAccess;
use serde::de::Visitor;
use serde::ser::SerializeStruct;
use std::collections::hash_map::Entry;
use std::f64;
use std::f64::consts::PI;
use std::f64::consts::TAU;
use std::fmt;
use std::{collections::HashMap, error::Error, path::Path};

pub mod aero;

#[derive(Clone, Debug, PartialEq, Hash, Eq, Deserialize)]
pub struct GridState {
    x: i64,
    y: i64,
    z: i64,
    b: i64,
}

#[derive(Debug)]
struct PenaltyRecord {
    state: GridState,
    penalty: f64,
}

impl Serialize for PenaltyRecord {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut state = serializer.serialize_struct("PenaltyRecord", 5)?;
        state.serialize_field("x", &self.state.x)?;
        state.serialize_field("y", &self.state.y)?;
        state.serialize_field("z", &self.state.z)?;
        state.serialize_field("b", &self.state.b)?;
        state.serialize_field("penalty", &self.penalty)?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for PenaltyRecord {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Define a visitor struct
        struct PenaltyRecordVisitor;

        impl<'de> Visitor<'de> for PenaltyRecordVisitor {
            type Value = PenaltyRecord;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct PenaltyRecord")
            }

            fn visit_map<V>(self, mut map: V) -> Result<PenaltyRecord, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut x = None;
                let mut y = None;
                let mut z = None;
                let mut b = None;
                let mut penalty = None;

                while let Some(key) = map.next_key::<String>()? {
                    match key.as_str() {
                        "x" => x = Some(map.next_value()?),
                        "y" => y = Some(map.next_value()?),
                        "z" => z = Some(map.next_value()?),
                        "b" => b = Some(map.next_value()?),
                        "penalty" => penalty = Some(map.next_value()?),
                        _ => {
                            return Err(de::Error::unknown_field(
                                &key,
                                &["x", "y", "z", "b", "penalty"],
                            ));
                        }
                    }
                }

                let x = x.ok_or_else(|| de::Error::missing_field("x"))?;
                let y = y.ok_or_else(|| de::Error::missing_field("y"))?;
                let z = z.ok_or_else(|| de::Error::missing_field("z"))?;
                let b = b.ok_or_else(|| de::Error::missing_field("b"))?;
                let penalty = penalty.ok_or_else(|| de::Error::missing_field("penalty"))?;

                Ok(PenaltyRecord {
                    state: GridState { x, y, z, b },
                    penalty,
                })
            }
        }

        deserializer.deserialize_struct(
            "PenaltyRecord",
            &["x", "y", "z", "b", "penalty"],
            PenaltyRecordVisitor,
        )
    }
}

pub struct PenaltyMap {
    resolution: [f64; 4],
    penalties: HashMap<GridState, f64>,
}

impl PenaltyMap {
    pub fn new() -> Self {
        Self {
            resolution: [0.5, 0.5, 0.25, 0.25],
            penalties: HashMap::new(),
        }
    }
    pub fn from_states(states: impl IntoIterator<Item = [f64; 4]>) -> Self {
        let mut map = Self::new();
        let mut max = f64::NEG_INFINITY;
        let mut min = f64::INFINITY;
        for state in states {
            let grid_state = map.align(&state);
            let count = map
                .penalties
                .entry(grid_state)
                .and_modify(|count| *count += 1.0)
                .or_insert(1.0);
            max = count.max(max);
            min = count.min(min);
        }

        let length = max - min;
        for value in map.penalties.values_mut() {
            // Gamma correction
            *value = ((*value - min) / length).powf(0.25) * 0.5;

            // Invert
            *value = 0.5 - *value;
            assert!(*value >= 0. && *value <= 1.);
        }

        map
    }

    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Self> {
        let mut map = Self::new();
        let mut reader = csv::Reader::from_path(path)?;
        for result in reader.deserialize() {
            let record: PenaltyRecord = result?;
            match map.penalties.entry(record.state) {
                Entry::Occupied(_) => return Err(anyhow!("duplicate record")),
                Entry::Vacant(handle) => handle.insert(record.penalty),
            };
        }

        Ok(map)
    }

    pub fn save<P: AsRef<Path>>(self, path: P) -> Result<()> {
        let mut writer = csv::Writer::from_path(path)?;
        for (state, penalty) in self.penalties {
            writer.serialize(PenaltyRecord { state, penalty })?;
        }

        Ok(())
    }

    pub fn penalty(&self, state: &[f64; 4]) -> f64 {
        let grid_state = self.align(state);
        *self.penalties.get(&grid_state).unwrap_or(&1.0)
    }

    fn align(&self, state: &[f64; 4]) -> GridState {
        let bearing = state[3].rem_euclid(TAU);
        GridState {
            x: (state[0] / self.resolution[0]).floor() as i64,
            y: (state[1] / self.resolution[1]).floor() as i64,
            z: (state[2] / self.resolution[2]).floor() as i64,
            b: (bearing / self.resolution[3]).floor() as i64,
        }
    }
}

#[test]
fn angle_wrapping() {
    let tcs = [(-PI, PI), (PI, PI), (TAU, 0.), (-3.0 * PI, PI)];
    for (unwrap, wrap) in tcs {
        let bearing = unwrap.rem_euclid(TAU);
        assert_eq!(bearing, wrap);
    }
}
