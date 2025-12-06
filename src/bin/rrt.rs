use dubins_paths::DubinsPath;
use itertools::Itertools;
use rand::{Rng, SeedableRng, rngs::SmallRng};
use std::{
    f64::consts::{E, PI},
    fs::File,
    io::{BufWriter, Write},
    marker::PhantomData,
    path::Path,
    time::Instant,
};

#[derive(serde::Serialize)]
struct BenchSample {
    iters: usize,
    state_count: usize,
    duration_ms: f64,
    iters_per_ms: f64,
}

fn main() {
    let max_iters = 10_000;
    let sampler = Sampler::new(
        (State([-75., -75., 0., 0.]), State([75., 75., 7., 2. * PI])),
        0.01,
    );
    let planner = RRTPlanner::new();

    let start = State([-50., 0., 5., 0.]);
    let goal = State([0., 0., 0., 0.]);
    let mut plan = planner.create_plan(start, goal, 0);

    let mut samples = Vec::new();

    let polling_frequency = 100;
    let outer = max_iters / polling_frequency;
    for i in 0..outer {
        let start = Instant::now();
        for _ in 0..polling_frequency {
            plan = planner.plan_from(plan, &sampler);
        }
        let duration = start.elapsed();

        let duration_ms = duration.as_micros() as f64 / 1000.0;
        samples.push(BenchSample {
            iters: (i + 1) * polling_frequency,
            state_count: plan.state_count(),
            duration_ms,
            iters_per_ms: polling_frequency as f64 / duration_ms,
        });
    }

    let mut writer = csv::Writer::from_path("rrt_benchmark.csv").unwrap();
    for sample in samples {
        writer.serialize(sample).unwrap();
    }

    println!("done");

    // plan.print_to_file("rrt_plan.g");
    plan.print_solution_to_file("rrt_plan.g");
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
            let mut state = [
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

    // TODO: THIS IS SUPER SLOW...
    pub fn k_nearest(&self, state: &State, k: usize) -> Vec<usize> {
        // let mut distances: Vec<_> =
        self.vertices
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v.state.dist(state)))
            .k_smallest_by(k, |(_ia, da), (_ib, db)| {
                da.partial_cmp(&db).expect("distances are well-ordered")
            })
            .map(|(i, _dist)| i)
            .collect()

        // distances.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("distances are well-ordered"));
        // distances.truncate(k);
        // distances.into_iter().map(|(i, _dist)| i).collect()
    }
}

struct RRTPlanner {}

impl RRTPlanner {
    pub fn new() -> Self {
        Self {}
    }

    pub fn create_plan<'a>(
        &self,
        start: State,
        goal: State,
        seed: u64,
    ) -> RRTPlan<SmallRng, State> {
        RRTPlan {
            world: World::new(start),
            rng: SmallRng::seed_from_u64(seed),
            goal,
            goal_ix: None,
            k_factor: E * (1. + 1. / State::dim() as f64),
            phan: PhantomData,
        }
    }

    pub fn plan_from(
        &self,
        mut plan: RRTPlan<SmallRng, State>,
        sampler: &Sampler,
    ) -> RRTPlan<SmallRng, State> {
        let mut is_goal = false;
        let sample = match sampler.sample(plan.rng()) {
            Sample::Goal => {
                is_goal = true;
                plan.goal.clone()
            }
            Sample::State(state) => state,
        };

        let nearest_ix = plan.world.k_nearest(&sample, 1)[0];
        let nearest_state = plan.world.state(nearest_ix).expect("nearest_ix exists");

        // What does it mean to steer?
        // We want to find the closest state to sample that obeys our constraints.
        // Dubins curve will give us the minimum time (t_min) to reach the sample in the XY plane.
        // Then, we compute the maximum altitude change we can perform in t_min.
        // We pick the smallest absolute altitude change between the maximum with our constraints
        // and the change to get to the sample exactly.

        let max_turn_rate = 0.025;
        let max_alt_rate = 0.006;
        let xy_velocity = 0.1;
        let q0 = [nearest_state.0[0], nearest_state.0[1], nearest_state.0[3]].into();
        let q1 = [sample.0[0], sample.0[1], sample.0[3]].into();
        let rho = max_turn_rate / xy_velocity;
        let Ok(shortest_path) = DubinsPath::shortest_from(q0, q1, rho) else {
            eprintln!("no path exists from nearest to the sample");

            // just try a different sample...
            return plan;
        };
        let xy_dist = shortest_path.length();
        let t_min = xy_dist / xy_velocity;
        let max_alt_change = max_alt_rate * t_min;
        // Clamp the alt_change to prevent us from violating our constraints.
        let alt_change = (nearest_state.0[2] - sample.0[2]).clamp(-max_alt_change, max_alt_change);
        let mut state = sample;
        state.0[2] += alt_change;

        if is_goal && plan.goal_ix.is_some() {
            // Already found the goal.
            return plan;
        }

        let k_nearest = plan.world.k_nearest(&state, plan.k_bound());

        // Connect this state to the state which provides the lowest cost-to-come.
        let (parent_ix, cost_to_come) = k_nearest
            .iter()
            .map(|ix| {
                (
                    ix,
                    plan.world
                        .cost_to_come(*ix)
                        .expect("k_nearest gives valid ids")
                        + state.dist(plan.world.state(*ix).expect("k_nearest gives valid ids")),
                )
            })
            .min_by(|(_, a), (_, b)| a.partial_cmp(&b).expect("costs are well-ordered"))
            .expect("length of k_nearest > 0");

        let ix = plan.world.insert(state.clone());
        *plan.world.parent_mut(ix).unwrap() = Some(*parent_ix);
        *plan.world.cost_to_come_mut(ix).unwrap() = cost_to_come;

        if is_goal {
            plan.goal_ix = Some(ix);
        }

        // Locally optimize...
        for child_ix in k_nearest.iter().filter(|cix| *cix != parent_ix) {
            let child_state = plan.world.state(*child_ix).unwrap();
            let child_cost_to_come = cost_to_come + state.dist(child_state);
            if plan.world.cost_to_come(*child_ix).unwrap() <= child_cost_to_come {
                // Already optimal.
                continue;
            }

            // Check whether this edge violates our constraints.
            let q0 = [state.0[0], state.0[1], state.0[3]].into();
            let q1 = [child_state.0[0], child_state.0[1], child_state.0[3]].into();
            let Ok(shortest_path) = DubinsPath::shortest_from(q0, q1, rho) else {
                // Failed to find a path; we assume this means this edge violates the motion constraints.
                continue;
            };
            let xy_dist = shortest_path.length();
            let t_min = xy_dist / xy_velocity;
            let max_alt_change = max_alt_rate * t_min;
            let alt_change = state.0[2] - child_state.0[2];
            if max_alt_change > alt_change.abs() {
                // This edge violates the motion constraints.
                continue;
            }

            // We found a better path, which does not violate any constraints, and lets use it.
            // TODO: This API is bad because it would let us set the parent to None.
            *plan.world.parent_mut(*child_ix).unwrap() = Some(ix);
            *plan.world.cost_to_come_mut(*child_ix).unwrap() = child_cost_to_come;
        }

        plan
    }
}

struct RRTPlan<R, S> {
    world: World,
    rng: R,
    goal: State,
    goal_ix: Option<usize>,
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
}
