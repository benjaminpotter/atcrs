use clap::Parser;
use dubins_paths::DubinsPath;
use itertools::Itertools;
use rand::Rng;
use std::{
    f64::consts::{E, PI},
    fs::File,
    io::{BufWriter, Write},
    marker::PhantomData,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

#[derive(serde::Serialize)]
struct BenchSample {
    iters: usize,
    state_count: usize,
    duration_ms: f64,
    iters_per_ms: f64,
}

#[derive(Parser)]
struct Cli {
    trial_path: PathBuf,
    output_path: PathBuf,
}

#[derive(serde::Deserialize)]
struct Trial {
    fa_flight_id: String,
    initial_enu_east_km: f64,
    initial_enu_north_km: f64,
    initial_enu_up_km: f64,
    initial_bearing: f64,
    target_enu_east_km: f64,
    target_enu_north_km: f64,
    target_enu_up_km: f64,
    target_bearing: f64,
}

#[derive(Debug, serde::Serialize)]
struct Results {
    elapsed_30s_ms: u128,
    elapsed_1m_ms: u128,
    elapsed_3m_ms: u128,

    path_length_30s: f64,
    path_length_1m: f64,
    path_length_3m: f64,

    iters_30s: usize,
    iters_1m: usize,
    iters_3m: usize,

    states_30s: usize,
    states_1m: usize,
    states_3m: usize,
}

impl Results {
    fn update(
        mut self,
        elapsed: Duration,
        path_length: f64,
        iters: usize,
        states: usize,
    ) -> Results {
        if elapsed <= Duration::from_secs(30) {
            self.elapsed_30s_ms = elapsed.as_millis();
            self.path_length_30s = path_length;
            self.iters_30s = iters;
            self.states_30s = states;
        } else if elapsed <= Duration::from_secs(1 * 60) {
            self.elapsed_1m_ms = elapsed.as_millis();
            self.path_length_1m = path_length;
            self.iters_1m = iters;
            self.states_1m = states;
        } else if elapsed <= Duration::from_secs(3 * 60) {
            self.elapsed_3m_ms = elapsed.as_millis();
            self.path_length_3m = path_length;
            self.iters_3m = iters;
            self.states_3m = states;
        }

        self
    }
}

impl Default for Results {
    fn default() -> Self {
        Self {
            elapsed_30s_ms: Default::default(),
            elapsed_1m_ms: Default::default(),
            elapsed_3m_ms: Default::default(),
            path_length_30s: f64::INFINITY,
            path_length_1m: f64::INFINITY,
            path_length_3m: f64::INFINITY,
            iters_30s: Default::default(),
            iters_1m: Default::default(),
            iters_3m: Default::default(),
            states_30s: Default::default(),
            states_1m: Default::default(),
            states_3m: Default::default(),
        }
    }
}

fn main() {
    let cli = Cli::parse();
    let reader = File::open(cli.trial_path).unwrap();
    let trial: Trial = serde_json::from_reader(reader).unwrap();

    let sampler = Sampler::new(
        // TODO: Do I need to handle angle wrapping?
        // (State([-75., -75., 0., 0.]), State([75., 75., 7., 2. * PI])),
        (State([-75., -75., 0., -PI]), State([75., 75., 7., PI])),
        0.1,
    );

    let max_turn_rate = 0.025;
    let max_alt_rate = 0.006;
    let xy_velocity = 0.1;
    let airplane = Airplane::new(max_turn_rate, max_alt_rate, xy_velocity);

    let start = State([
        trial.initial_enu_east_km,
        trial.initial_enu_north_km,
        trial.initial_enu_up_km,
        trial.initial_bearing,
    ]);

    let goal = State([
        trial.target_enu_east_km,
        trial.target_enu_north_km,
        trial.target_enu_up_km,
        trial.target_bearing,
    ]);

    let planner = RRTPlanner::new();
    let mut plan = planner.create_plan(start, goal, airplane, rand::rng());
    let mut results = Results::default();

    let poll_after = 1000;
    let timeout = Duration::from_secs(3 * 60);
    let mut elapsed = Duration::ZERO;

    while elapsed < timeout {
        let start = Instant::now();
        for _ in 0..poll_after {
            plan = planner.plan_from(plan, &sampler);
        }
        let duration = start.elapsed();
        elapsed += duration;

        println!("{} of {}", elapsed.as_secs(), timeout.as_secs());
        results = results.update(
            elapsed,
            plan.path_length(),
            plan.iter_count(),
            plan.state_count(),
        );
    }

    // plan.print_to_file("rrt_plan.g");
    // plan.print_solution_to_file("rrt_plan.g");
    let mut writer = csv::Writer::from_path(cli.output_path).unwrap();
    writer.serialize(results).unwrap();
}

trait Statelike {
    fn dim() -> usize;
}

#[derive(Clone, Debug, PartialEq)]
struct State([f64; 4]);

impl State {
    fn dist(&self, state: &State) -> f64 {
        ((state.0[0] - self.0[0]).powf(2.)
            + (state.0[1] - self.0[1]).powf(2.)
            + (state.0[2] - self.0[2]).powf(2.)
            + (state.0[3] - self.0[3]).powf(2.))
        .sqrt()
    }
}

impl Statelike for State {
    fn dim() -> usize {
        4
    }
}

struct Airplane {
    //max_turn_rate: f64,
    max_alt_rate: f64,
    xy_velocity: f64,
    max_curvature: f64,
}

impl Airplane {
    fn new(max_turn_rate: f64, max_alt_rate: f64, xy_velocity: f64) -> Self {
        Self {
            //        max_turn_rate,
            max_alt_rate,
            xy_velocity,
            max_curvature: max_turn_rate / xy_velocity,
        }
    }

    fn shortest_xy_path(&self, from: &State, to: &State) -> Option<DubinsPath> {
        let q0 = [from.0[0], from.0[1], from.0[3]].into();
        let q1 = [to.0[0], to.0[1], to.0[3]].into();
        DubinsPath::shortest_from(q0, q1, self.max_curvature).ok()
    }

    fn xy_velocity(&self) -> f64 {
        self.xy_velocity
    }

    fn max_alt_rate(&self) -> f64 {
        self.max_alt_rate
    }

    fn length_between(&self, from: &State, to: &State) -> Option<(f64, bool)> {
        let shortest_path = self.shortest_xy_path(from, to)?;
        let xy_dist = shortest_path.length();
        let t_min = xy_dist / self.xy_velocity;
        let max_alt_change = self.max_alt_rate * t_min;
        let alt_change = from.0[2] - to.0[2];
        let valid = max_alt_change > alt_change.abs();

        Some((xy_dist.powf(2.) + alt_change.powf(2.), valid))
    }
}

struct Vertex {
    parent: Option<usize>,
    state: State,
    cost_to_come: f64,
}

enum Sample {
    Goal,
    State(State),
}

struct Sampler {
    min: State,
    max: State,
    goal_sampling_chance: f64,
}

impl Sampler {
    fn new(corners: (State, State), goal_sampling_chance: f64) -> Self {
        Self {
            min: State([
                corners.0.0[0].min(corners.1.0[0]),
                corners.0.0[1].min(corners.1.0[1]),
                corners.0.0[2].min(corners.1.0[2]),
                corners.0.0[3].min(corners.1.0[3]),
            ]),
            max: State([
                corners.0.0[0].max(corners.1.0[0]),
                corners.0.0[1].max(corners.1.0[1]),
                corners.0.0[2].max(corners.1.0[2]),
                corners.0.0[3].max(corners.1.0[3]),
            ]),
            goal_sampling_chance,
        }
    }

    fn sample<R: Rng>(&self, rng: &mut R) -> Sample {
        if !rng.random_bool(self.goal_sampling_chance) {
            let state = [
                rng.random_range(self.min.0[0]..self.max.0[0]),
                rng.random_range(self.min.0[1]..self.max.0[1]),
                rng.random_range(self.min.0[2]..self.max.0[2]),
                rng.random_range(self.min.0[3]..self.max.0[3]),
            ];
            return Sample::State(State(state));
        }

        Sample::Goal
    }
}

struct World {
    vertices: Vec<Vertex>,
}

enum Dir {
    From,
    To,
}

impl World {
    pub fn new(start: State) -> Self {
        let vertices = vec![Vertex {
            parent: None,
            state: start,
            cost_to_come: 0.,
        }];

        World { vertices }
    }

    fn root(&self) -> usize {
        0
    }

    fn all(&self) -> Vec<usize> {
        (0..self.vertices.len()).collect()
    }

    fn state(&self, ix: usize) -> Option<&State> {
        self.vertices.get(ix).map(|v| &v.state)
    }

    fn cost_to_come(&self, ix: usize) -> Option<f64> {
        self.vertices.get(ix).map(|v| v.cost_to_come)
    }

    fn cost_to_come_mut(&mut self, ix: usize) -> Option<&mut f64> {
        self.vertices.get_mut(ix).map(|v| &mut v.cost_to_come)
    }

    fn insert(&mut self, state: State) -> usize {
        self.vertices.push(Vertex {
            parent: None,
            state,
            cost_to_come: f64::INFINITY,
        });
        self.vertices.len() - 1
    }

    fn parent(&self, ix: usize) -> Option<Option<usize>> {
        self.vertices.get(ix).map(|v| v.parent)
    }

    fn parent_mut(&mut self, ix: usize) -> Option<&mut Option<usize>> {
        self.vertices.get_mut(ix).map(|v| &mut v.parent)
    }

    pub fn state_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn k_nearest(
        &self,
        airplane: &Airplane,
        state: &State,
        dir: Dir,
        k: usize,
    ) -> Vec<(usize, (f64, bool))> {
        self.vertices
            .iter()
            .enumerate()
            // Filter by euclidean distance for performance.
            .map(|(i, v)| (i, v, v.state.dist(state)))
            .k_smallest_by(2 * k, |(_, _, da), (_, _, db)| {
                da.partial_cmp(&db).expect("distances are well-ordered")
            })
            .map(|(i, v, _)| (i, v))
            .filter_map(|(i, v)| {
                // We can't fly backwards...
                let length_between = match dir {
                    Dir::To => airplane.length_between(&v.state, state),
                    Dir::From => airplane.length_between(state, &v.state),
                };
                Some((i, length_between?))
            })
            .k_smallest_by(k, |(_, (da, _)), (_, (db, _))| {
                da.partial_cmp(&db).expect("distances are well-ordered")
            })
            .collect()
    }
}

struct RRTPlanner {}

impl RRTPlanner {
    pub fn new() -> Self {
        Self {}
    }

    pub fn create_plan<R: Rng>(
        &self,
        start: State,
        goal: State,
        airplane: Airplane,
        rng: R,
    ) -> RRTPlan<R, State> {
        RRTPlan {
            world: World::new(start),
            rng,
            goal,
            goal_ix: None,
            airplane,
            iters: 0,
            k_factor: E * (1. + 1. / State::dim() as f64),
            phan: PhantomData,
        }
    }

    pub fn plan_from<R: Rng>(
        &self,
        mut plan: RRTPlan<R, State>,
        sampler: &Sampler,
    ) -> RRTPlan<R, State> {
        plan.iters += 1;

        let mut is_goal = false;
        let sample = match sampler.sample(plan.rng()) {
            Sample::Goal => {
                is_goal = true;
                plan.goal.clone()
            }
            Sample::State(state) => state,
        };

        let (nearest_ix, _) = plan.world.k_nearest(&plan.airplane, &sample, Dir::To, 1)[0];
        let nearest_state = plan.world.state(nearest_ix).expect("nearest_ix exists");

        // What does it mean to steer?
        // We want to find the closest state to sample that obeys our constraints.
        // Dubins curve will give us the minimum time (t_min) to reach the sample in the XY plane.
        // Then, we compute the maximum altitude change we can perform in t_min.
        // We pick the smallest absolute altitude change between the maximum with our constraints
        // and the change to get to the sample exactly.

        let Some(shortest_path) = plan.airplane.shortest_xy_path(&nearest_state, &sample) else {
            eprintln!("no path exists from nearest to the sample");

            // just try a different sample...
            return plan;
        };
        let xy_dist = shortest_path.length();
        let t_min = xy_dist / plan.airplane.xy_velocity();
        let max_alt_change = plan.airplane.max_alt_rate() * t_min;
        // Clamp the alt_change to prevent us from violating our constraints.
        let alt_change = (nearest_state.0[2] - sample.0[2]).clamp(-max_alt_change, max_alt_change);
        let mut state = sample;
        state.0[2] += alt_change;

        if is_goal && plan.goal_ix.is_some() {
            // Already found the goal.
            return plan;
        }

        // Connect this state to the state which provides the lowest cost-to-come.
        let Some((parent_ix, cost_to_come)) = plan
            .world
            // Move to the state from its parents.
            .k_nearest(&plan.airplane, &state, Dir::To, plan.k_bound())
            .into_iter()
            .filter(|(_, (_, valid))| *valid)
            .map(|(ix, (length_between, _))| {
                (
                    ix,
                    plan.world
                        .cost_to_come(ix)
                        .expect("k_nearest gives valid ids")
                        + length_between,
                )
            })
            .min_by(|(_, a), (_, b)| a.partial_cmp(&b).expect("costs are well-ordered"))
        else {
            // No valid parents exist for this state.
            // Lets try again...
            return plan;
        };

        let ix = if !is_goal {
            // We didn't sample the goal.
            // - Insert
            // - Return ix
            plan.world.insert(state.clone())
        } else if let Some(ix) = plan.goal_ix {
            // We sampled the goal and we have done so before.
            //  - Return goal_ix
            ix
        } else {
            // We sampled the goal and we have not done so before.
            //  - Insert
            //  - Set goal_ix
            //  - Return goal_ix
            println!("found goal");

            let ix = plan.world.insert(state.clone());
            plan.goal_ix = Some(ix);
            ix
        };

        *plan.world.parent_mut(ix).unwrap() = Some(parent_ix);
        *plan.world.cost_to_come_mut(ix).unwrap() = cost_to_come;

        // Locally optimize...
        for (child_ix, (length_between, _)) in plan
            .world
            // Move from the state to its neighbours.
            .k_nearest(&plan.airplane, &state, Dir::From, plan.k_bound())
            .into_iter()
            .filter(|(cix, _)| *cix != parent_ix)
            // Check whether this edge violates our constraints.
            // Could fail to find a path; we assume this means this edge violates the motion constraints.
            .filter(|(_, (_, valid))| *valid)
        {
            let child_cost_to_come = cost_to_come + length_between;
            if plan.world.cost_to_come(child_ix).unwrap() <= child_cost_to_come {
                // Already optimal.
                continue;
            }

            // We found a better path, which does not violate any constraints, and lets use it.
            // TODO: This API is bad because it would let us set the parent to None.
            *plan.world.parent_mut(child_ix).unwrap() = Some(ix);
            *plan.world.cost_to_come_mut(child_ix).unwrap() = child_cost_to_come;
        }

        plan
    }
}

struct RRTPlan<R, S> {
    world: World,
    rng: R,
    goal: State,
    goal_ix: Option<usize>,
    airplane: Airplane,
    iters: usize,
    k_factor: f64,
    phan: PhantomData<S>,
}

impl<R, S> RRTPlan<R, S> {
    fn rng(&mut self) -> &mut R
    where
        R: Rng,
    {
        &mut self.rng
    }

    fn k_bound(&self) -> usize {
        let card = self.world.state_count() as f64;
        let bound = (self.k_factor * card.log2()).ceil() as usize;
        bound.clamp(1, usize::MAX)
    }

    fn state_count(&self) -> usize {
        self.world.state_count()
    }

    fn print_to_file<P: AsRef<Path>>(&self, path: P) {
        println!("state count: {}", self.world.state_count());

        let file = File::create(&path).unwrap();
        let mut writer = BufWriter::new(file);
        // let mut writer = std::io::stdout();

        for ix in self.world.all() {
            let state = self.world.state(ix).unwrap();
            let cost_to_come = self.world.cost_to_come(ix).unwrap();

            let _ = writeln!(
                writer,
                "v {} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2}",
                ix, state.0[0], state.0[1], state.0[2], state.0[3], cost_to_come, 0.,
            );

            if let Some(parent_ix) = self.world.parent(ix).unwrap() {
                let _ = writeln!(writer, "e {} {}", parent_ix, ix);
            }
        }
    }

    fn print_solution_to_file<P: AsRef<Path>>(&self, path: P) {
        let Some(goal_ix) = self.goal_ix else {
            eprintln!("no solution found");
            return;
        };

        let file = File::create(&path).unwrap();
        let mut writer = BufWriter::new(file);
        // let mut writer = std::io::stdout();

        let mut ix = goal_ix;
        loop {
            let state = self.world.state(ix).unwrap();
            let cost_to_come = self.world.cost_to_come(ix).unwrap();

            let _ = writeln!(
                writer,
                "v {} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2}",
                ix, state.0[0], state.0[1], state.0[2], state.0[3], cost_to_come, 0.,
            );

            let Some(parent_ix) = self.world.parent(ix).unwrap() else {
                break;
            };

            let _ = writeln!(writer, "e {} {}", parent_ix, ix);
            ix = parent_ix;
        }
    }

    fn path_length(&self) -> f64 {
        match self.goal_ix {
            Some(ix) => self.world.cost_to_come(ix).unwrap(),
            None => f64::INFINITY,
        }
    }

    fn iter_count(&self) -> usize {
        self.iters
    }
}
