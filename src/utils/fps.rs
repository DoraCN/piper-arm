//! Frame-rate counter.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Instant;

/// Thread-safe FPS counter for measuring the rate of incoming CAN messages.
#[derive(Debug, Default)]
pub struct FpsCounter {
    /// Time window over which FPS is computed.
    interval_secs: f64,
    data: Mutex<FpsData>,
}

#[derive(Debug, Default)]
struct FpsData {
    counters: HashMap<String, u64>,
    results: HashMap<String, f64>,
    prev: HashMap<String, u64>,
    last: HashMap<String, Instant>,
}

impl FpsCounter {
    pub fn new() -> Self {
        Self {
            interval_secs: 0.1,
            data: Mutex::new(FpsData::default()),
        }
    }

    pub fn set_cal_fps_time_interval(&mut self, interval_secs: f64) {
        self.interval_secs = interval_secs;
    }

    /// Register a new counter variable.
    pub fn add_variable(&self, name: &str) {
        let mut d = self.data.lock().unwrap();
        d.counters.entry(name.to_string()).or_insert(0);
        d.results.entry(name.to_string()).or_insert(0.0);
        d.prev.entry(name.to_string()).or_insert(0);
        d.last.entry(name.to_string()).or_insert(Instant::now());
    }

    /// Increment the counter for `name`.
    pub fn increment(&self, name: &str) {
        let mut d = self.data.lock().unwrap();
        if let Some(c) = d.counters.get_mut(name) {
            *c += 1;
        }
        d.last.entry(name.to_string()).or_insert(Instant::now());
    }

    /// Return the current FPS estimate for `name`.
    pub fn get_fps(&self, name: &str) -> f64 {
        let d = self.data.lock().unwrap();
        d.results.get(name).copied().unwrap_or(0.0) * (1.0 / self.interval_secs)
    }

    /// Recompute FPS for all variables based on the current counts.
    /// Should be called periodically (every `interval_secs`).
    pub fn tick(&self) {
        let mut d = self.data.lock().unwrap();
        let names: Vec<String> = d.counters.keys().cloned().collect();
        for name in names {
            let count = d.counters[&name];
            let prev = d.prev.get(&name).copied().unwrap_or(0);
            d.results.insert(name.clone(), (count - prev) as f64);
            d.prev.insert(name, count);
        }
    }

    /// Average of the given values, returning 0 if any is 0.
    pub fn cal_average(&self, values: &[f64]) -> f64 {
        if values.is_empty() || values.contains(&0.0) {
            return 0.0;
        }
        let avg = values.iter().sum::<f64>() / values.len() as f64;
        (avg * 1000.0).round() / 1000.0
    }
}
