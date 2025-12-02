use std::{
    collections::{BinaryHeap, HashMap, HashSet},
    ops::Range,
};

use uuid::Uuid;

fn main() {
    let start_state = State {
        x: 10.0,
        y: 10.0,
        z: 5.0,
        b: 0.0,
    };

    let planner = ARAPlanner {
        start_eps: 4.0,
        eps_delta: 0.5,
        suboptimality: 1.0,
        goal_region: StateRegion {
            x: -500.0..500.0,
            y: -500.0..500.0,
            z: -25.0..25.0,
            b: -0.125..0.125,
        },
    };

    let mut plan = planner.create_plan(start_state);
    println!("{:#?}", plan);

    // TODO: Replace with timing logic.
    for _ in 0..10 {
        plan = planner.plan_from(plan.clone());
    }
}

/// # State Discretization
/// - x, y use 125m
/// - z uses 50m
/// - bearing uses 0.05 rad
#[derive(Clone, Debug)]
struct State {
    // TODO: this might be faster if we use a vector
    // TODO: this would also be more extensible as it could hold any number of state components
    // TODO: BEARINGS NEED TO WRAP
    x: f64,
    y: f64,
    z: f64,
    b: f64,
}

#[derive(Clone, Debug)]
struct StateRegion {
    x: Range<f64>,
    y: Range<f64>,
    z: Range<f64>,
    b: Range<f64>,
}

#[derive(Clone, Debug)]
struct Vertex {
    id: Uuid,
    state: State,
    successors: Option<Vec<Uuid>>,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct AlignedState {
    x: i64,
    y: i64,
    z: i64,
    b: i64,
}

#[derive(Clone, Debug)]
struct Motion {
    dxy: f64,
    dz: f64,
    db: f64,
}

#[derive(Clone, Debug)]
struct Airplane {
    motions: [Motion; 9],
}

#[derive(Clone, Debug)]
struct Discretizer {
    dx: f64,
    dy: f64,
    dz: f64,
    db: f64,
}

#[derive(Clone, Debug)]
struct World {
    /// Sparse storage of states in the world.
    vertices: HashMap<Uuid, Vertex>,

    // Let's us search for a vertex using an aligned state.
    /// Mapping from AlignedStates to ids.
    ids: HashMap<AlignedState, Uuid>,

    // Let's us determine which states can be connected
    airplane: Airplane,

    discretizer: Discretizer,
}

#[derive(Clone, Debug)]
struct OpenVertex {
    id: Uuid,
    f: f64,
}

struct ARAPlanner {
    start_eps: f64,
    eps_delta: f64,
    suboptimality: f64,
    goal_region: StateRegion,
}

#[derive(Clone, Debug)]
struct ARAPlan {
    world: World,

    // This will store each state in the open set.
    // Popping will give the min state.
    // It will not guarantee that the popped value is live.
    // Must check with the graph to test for differences.
    open_set: BinaryHeap<OpenVertex>,
    closed_set: HashSet<Uuid>,
    incons_set: HashSet<Uuid>,
    g_vals: HashMap<Uuid, f64>,
    h_vals: HashMap<Uuid, f64>,

    start_id: Uuid,
    goal_id: Option<Uuid>,
    goal_f_val: f64,
    goal_region: StateRegion,
}

impl State {
    pub fn euclidean_dist_to(&self, other: &State) -> f64 {
        ((self.x - other.x).powf(2.)
            + (self.y - other.y).powf(2.)
            + (self.z - other.z).powf(2.)
            + (self.b - other.b).powf(2.))
        .sqrt()
    }
}

impl StateRegion {
    pub fn contains(&self, state: &State) -> bool {
        self.x.contains(&state.x)
            && self.y.contains(&state.y)
            && self.z.contains(&state.z)
            && self.b.contains(&state.b)
    }

    pub fn center(&self) -> State {
        todo!()
    }
}

impl Airplane {
    pub fn new() -> Self {
        let motions = [
            Motion {
                dxy: DXY,
                dz: -DZ,
                db: -DB,
            },
            Motion {
                dxy: DXY,
                dz: -DZ,
                db: 0.,
            },
            Motion {
                dxy: DXY,
                dz: -DZ,
                db: DB,
            },
            Motion {
                dxy: DXY,
                dz: 0.,
                db: -DB,
            },
            Motion {
                dxy: DXY,
                dz: 0.,
                db: 0.,
            },
            Motion {
                dxy: DXY,
                dz: 0.,
                db: DB,
            },
            Motion {
                dxy: DXY,
                dz: DZ,
                db: -DB,
            },
            Motion {
                dxy: DXY,
                dz: DZ,
                db: 0.,
            },
            Motion {
                dxy: DXY,
                dz: DZ,
                db: DB,
            },
        ];

        Self { motions }
    }

    pub fn move_from(&self, state: &State) -> Vec<State> {
        self.motions
            .iter()
            .map(|ctl| State {
                x: state.x + ctl.dxy * state.b.cos(),
                y: state.y + ctl.dxy * state.b.sin(),
                z: state.z + ctl.dz,
                b: state.b + ctl.db,
            })
            .collect()
    }
}

impl Discretizer {
    fn discretize(&self, state: &State) -> AlignedState {
        AlignedState {
            x: (state.x / self.dx).floor() as i64,
            y: (state.y / self.dy).floor() as i64,
            z: (state.z / self.dz).floor() as i64,
            b: (state.b / self.db).floor() as i64,
        }
    }
}

impl World {
    fn new(discretizer: Discretizer) -> Self {
        Self {
            vertices: HashMap::new(),
            ids: HashMap::new(),
            airplane: Airplane::new(),
            discretizer,
        }
    }

    fn vertex(&self, id: &Uuid) -> &Vertex {
        self.vertices
            .get(&id)
            .expect("never look for a vertex that does not exist")
    }

    fn vertex_mut(&mut self, id: &Uuid) -> &mut Vertex {
        self.vertices
            .get_mut(&id)
            .expect("never look for a vertex that does not exist")
    }

    fn insert(&mut self, state: State) -> Uuid {
        let id = Uuid::new_v4();
        let aligned = self.discretizer.discretize(&state);

        if let Some(_) = self.ids.insert(aligned, id) {
            panic!("attempted to overwrite a state that already exists");
        }

        if let Some(_) = self.vertices.insert(
            id,
            Vertex {
                id,
                state,
                successors: None,
            },
        ) {
            panic!("id collision");
        }

        id
    }

    fn id(&self, state: &State) -> Option<&Uuid> {
        self.ids.get(&self.discretizer.discretize(state))
    }

    pub fn state(&self, id: &Uuid) -> &State {
        &self.vertex(id).state
    }

    pub fn successors(&mut self, id: &Uuid) -> Vec<Uuid> {
        let vertex = self.vertex(&id);

        // The vertex we are looking at could have had its successor states already computed.
        // In this case, we likely want to have those connections cached, so that we avoid
        // recomputing the successors using the motion primitives.
        if let Some(successors) = vertex.successors.as_ref() {
            return successors.clone();
        }

        // If the vertex has not had its successor states already computed, then we must compute
        // all of the successors using the motion primitives.
        let successors: Vec<_> = self
            .airplane
            .move_from(&vertex.state)
            .into_iter()
            .map(|state| {
                // It is likely that some of these successors have already been added to the world by other
                // states.
                // In this case, we want to add those ids to our successors list, but we don't touch
                // their successors list as it has either already been filled, or we have not expanded
                // that vertex yet.
                // If we have to update their successor list, then we will do so when this function is
                // called from their context.
                if let Some(id) = self.id(&state) {
                    return id.clone();
                }

                // If this successor is not in our world yet, then we have to create a vertex for
                // it.
                // We return the id of the new vertex so that it gets included in our successor
                // list.
                self.insert(state)
            })
            .collect();

        self.vertex_mut(id).successors = Some(successors.clone());

        successors
    }
}

const V: f64 = 100.;
const DT: f64 = 30.;
const DXY: f64 = V * DT;
const DZ: f64 = 6. * DT;
const DB: f64 = 0.025 * DT;

impl ARAPlanner {
    pub fn create_plan(&self, start: State) -> ARAPlan {
        let mut world = World::new(Discretizer {
            dx: DXY,
            dy: DXY,
            dz: DZ,
            db: DB,
        });

        let start_id = world.insert(start).clone();

        let mut plan = ARAPlan {
            world,
            open_set: BinaryHeap::new(),
            closed_set: HashSet::new(),
            incons_set: HashSet::new(),
            g_vals: HashMap::new(),
            h_vals: HashMap::new(),
            start_id,
            goal_id: None,
            goal_f_val: f64::INFINITY,
            goal_region: self.goal_region.clone(),
        };

        *plan.g_val_mut(&start_id) = 0.;
        plan.open(start_id);

        plan
    }

    pub fn plan_from(&self, mut plan: ARAPlan) -> ARAPlan {
        if plan.suboptimality() > self.suboptimality {
            // TODO: eps -= eps_delta;
            // TODO: move states from incons to open
            // TODO: update priorities of open using f values
            // TODO: reset the closed set

            plan.improve_path();
        }

        plan
    }
}

impl ARAPlan {
    fn g_val(&self, id: &Uuid) -> &f64 {
        self.g_vals.get(id).unwrap_or(&f64::INFINITY)
    }

    fn g_val_mut(&mut self, id: &Uuid) -> &mut f64 {
        self.g_vals.entry(*id).or_insert(f64::INFINITY)
    }

    fn open(&mut self, id: Uuid) {
        self.open_set.push(OpenVertex { id, f: 0. })
    }

    fn close(&mut self, id: Uuid) {
        self.closed_set.insert(id);
    }

    fn is_closed(&self, id: &Uuid) -> bool {
        self.closed_set.contains(id)
    }

    fn mark_incons(&self, _id: Uuid) {
        todo!()
    }

    fn suboptimality(&self) -> f64 {
        // TODO: actually implement this
        10.0
    }

    fn clean_open_set(&mut self) {
        while let Some(v) = self.open_set.peek() {
            let g_val = *self.g_val(&v.id);
            // TODO: This should be an f val
            // Need to write the heuristic stuff first.
            if v.f != g_val {
                self.open_set.pop();
            } else {
                break;
            }
        }
    }

    fn peek(&self) -> Uuid {
        self.open_set
            .peek()
            .expect("the open set is never empty")
            .id
    }

    fn pop(&mut self) -> Uuid {
        let id = self.open_set.pop().expect("the open set is never empty").id;
        self.close(id);
        id
    }

    fn successors(&mut self, id: Uuid) -> Vec<Uuid> {
        self.world.successors(&id)
    }

    fn cost(&self, start: &Uuid, end: &Uuid) -> f64 {
        // This function can only be used to compare adjacent vertices
        // TODO: maybe we should have an assertion here
        self.world
            .state(start)
            .euclidean_dist_to(&self.world.state(end))
    }

    fn improve_path(&mut self) {
        loop {
            self.clean_open_set();
            let peeked = self.peek();
            // TODO: this should be f_val
            if self.goal_f_val <= *self.g_val(&peeked) {
                break;
            }

            let id = self.pop();

            println!("{id}");

            for succ in self.successors(id) {
                println!("succ: {succ}");

                let cost = self.g_val(&id) + self.cost(&id, &succ);

                // Save as the goal if inside the goal region
                if self.goal_id.is_none() {
                    if self.goal_region.contains(&self.world.state(&succ)) {
                        self.goal_id = Some(succ);
                        self.goal_f_val = cost;
                    }
                }

                // if succ is better through s then
                if cost < *self.g_val(&succ) {
                    // update its g value
                    *self.g_val_mut(&succ) = cost;

                    // if its not in closed then insert it into open
                    if !self.is_closed(&succ) {
                        self.open(succ)
                    }
                    // otherwise insert it into incons
                    else {
                        self.mark_incons(succ);
                    }
                }
            }
        }
    }
}

impl PartialEq for OpenVertex {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.id == other.id
    }
}

impl Eq for OpenVertex {}

impl Ord for OpenVertex {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f
            .partial_cmp(&other.f)
            .expect("we should ensure that floats are always ordered")
            .then_with(|| self.id.cmp(&other.id))
    }
}

impl PartialOrd for OpenVertex {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(&other))
    }
}
