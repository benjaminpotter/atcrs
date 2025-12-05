use std::{f64::consts::E, marker::PhantomData};

use dubins_paths::DubinsPath;
use rand::{Rng, SeedableRng, rngs::SmallRng};

fn main() {
    let max_iters = 1;
    let sampler = Sampler::new(0.01);
    let planner = RRTPlanner::new();

    let start = State([-50., 0., 0., 0.]);
    let goal = State([0., 0., 0., 0.]);
    let mut plan = planner.create_plan(start, goal, 0);

    for _ in 0..max_iters {
        plan = planner.plan_from(plan, &sampler);
    }
}

trait Statelike {
    fn dim() -> usize;
}

#[derive(Clone, Debug)]
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

struct Vertex<'a> {
    parent: Option<&'a Vertex<'a>>,
    state: State,
    cost_to_come: f64,
    children: Vec<&'a Vertex<'a>>,
}

enum Sample {
    Goal,
    State(State),
}

struct Sampler {
    goal_sampling_chance: f64,
}

impl Sampler {
    fn sample<R: Rng>(&self, rng: &mut R) -> Sample {
        if !rng.random_bool(self.goal_sampling_chance) {
            let mut state = [0.0; 4];
            rng.fill(&mut state);
            return Sample::State(State(state));
        }

        Sample::Goal
    }

    fn new(goal_sampling_chance: f64) -> Self {
        Self {
            goal_sampling_chance,
        }
    }
}

struct World<'a> {
    vertices: Vec<Vertex<'a>>,
}

impl<'a> World<'a> {
    pub fn new(start: State) -> Self {
        let vertices = vec![Vertex {
            parent: None,
            state: start,
            cost_to_come: 0.,
            children: Vec::new(),
        }];

        World { vertices }
    }

    fn state(&self, ix: usize) -> Option<&State> {
        self.vertices.get(ix).map(|v| &v.state)
    }

    pub fn state_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn k_nearest(&'a self, state: &State, k: usize) -> Vec<usize> {
        let mut distances: Vec<_> = self
            .vertices
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v.state.dist(state)))
            .collect();

        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("distances are well-ordered"));
        distances.truncate(k);

        distances.into_iter().map(|(i, _dist)| i).collect()
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
    ) -> RRTPlan<'a, SmallRng, State> {
        RRTPlan {
            world: World::new(start),
            rng: SmallRng::seed_from_u64(seed),
            goal,
            k_factor: E * (1. + 1. / State::dim() as f64),
            phan: PhantomData,
        }
    }

    pub fn plan_from<'a>(
        &self,
        mut plan: RRTPlan<'a, SmallRng, State>,
        sampler: &Sampler,
    ) -> RRTPlan<'a, SmallRng, State> {
        let sample = match sampler.sample(plan.rng()) {
            Sample::Goal => plan.goal.clone(),
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
        let t_min = shortest_path.length() / xy_velocity;
        let max_alt_change = max_alt_rate * t_min;
        let sample_alt_change = nearest_state.0[2] - sample.0[2];
        let alt_change = if max_alt_change.abs() < sample_alt_change.abs() {
            max_alt_change
        } else {
            sample_alt_change
        };
        let mut state = sample;
        state.0[2] = alt_change;

        // No edge check required because we don't have any hard obstacles.

        let k_nearest = plan.world.k_nearest(&nearest_state, plan.k_bound());

        // Connect this state to the state which provides the lowest cost-to-come.
        let parent = k_nearest.iter().map(|ix| {
            (
                ix,
                plan.world.cost_to_come(ix) + state.dist(plan.world.state(ix)),
            )
        }); // TODO: take the min of this list by the cost_to_come, then use that ix as the parent

        // Add this state to the world.

        // Locally optimize...

        plan
    }
}

struct RRTPlan<'a, R, S> {
    world: World<'a>,
    rng: R,
    goal: State,
    k_factor: f64,
    phan: PhantomData<S>,
}

impl<'a, R, S> RRTPlan<'a, R, S> {
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
}
