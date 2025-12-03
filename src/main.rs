use dubins_paths::DubinsPath;
use radians::Wrap64;
use std::f64::consts::PI;
use std::io::Write;
use std::{
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
    fs::File,
    io::BufWriter,
    ops::Range,
};
use uuid::Uuid;

fn main() {
    let start_state = State::new(-1.0, 0.0, 1.0, 0.0);
    let mut world = World::new(Discretizer {
        dx: XY,
        dy: XY,
        dz: Z,
        db: B,
    });

    // let start_id = world.insert(start_state);
    // let successors = world.successors(&start_id);
    //
    // println!("{:#?}", world.state(&start_id));
    // for succ in successors {
    //     println!("{:#?}", world.state(&succ));
    // }

    let planner = ARAPlanner {
        start_eps: 4.0,
        eps_delta: 0.5,
        suboptimality: 1.0,
        goal_region: StateRegion {
            x: -0.5..0.5,
            y: -0.5..0.5,
            z: -0.25..0.25,
            b: -0.125..0.125,
        },
    };

    let mut plan = planner.create_plan(start_state);
    println!("{:#?}", plan);

    // TODO: Replace with timing logic.
    for _ in 0..1 {
        plan = planner.plan_from(plan.clone());
    }
}

#[derive(Clone, Debug, PartialEq)]
struct State {
    // TODO: this might be faster if we use a vector
    // TODO: this would also be more extensible as it could hold any number of state components
    x: f64,
    y: f64,
    z: f64,
    b: Wrap64,
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
    state: AlignedState,
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
    eps: f64,

    start_id: Uuid,
    goal_id: Option<Uuid>,
    goal_f_val: f64,
    goal_region: StateRegion,
}

impl State {
    pub fn new(x: f64, y: f64, z: f64, b: f64) -> Self {
        State {
            x,
            y,
            z,
            b: Wrap64::wrap(b),
        }
    }

    pub fn euclidean_dist_to(&self, other: &State) -> f64 {
        ((self.x - other.x).powf(2.)
            + (self.y - other.y).powf(2.)
            + (self.z - other.z).powf(2.)
            + (self.b - other.b).val().powf(2.))
        .sqrt()
    }

    pub fn bearing(&self) -> f64 {
        self.b.val()
    }
}

impl StateRegion {
    pub fn contains(&self, state: &State) -> bool {
        self.x.contains(&state.x)
            && self.y.contains(&state.y)
            && self.z.contains(&state.z)
            && self.b.contains(&state.bearing())
    }

    fn center_between(range: &Range<f64>) -> f64 {
        range.start + (range.end - range.start) / 2.
    }

    pub fn center(&self) -> State {
        State::new(
            Self::center_between(&self.x),
            Self::center_between(&self.y),
            Self::center_between(&self.z),
            Self::center_between(&self.b),
        )
    }
}

impl Airplane {
    pub fn new() -> Self {
        let motions = [
            Motion {
                dxy: XY,
                dz: -Z,
                db: -B,
            },
            Motion {
                dxy: XY,
                dz: -Z,
                db: 0.,
            },
            Motion {
                dxy: XY,
                dz: -Z,
                db: B,
            },
            Motion {
                dxy: XY,
                dz: 0.,
                db: -B,
            },
            Motion {
                dxy: XY,
                dz: 0.,
                db: 0.,
            },
            Motion {
                dxy: XY,
                dz: 0.,
                db: B,
            },
            Motion {
                dxy: XY,
                dz: Z,
                db: -B,
            },
            Motion {
                dxy: XY,
                dz: Z,
                db: 0.,
            },
            Motion {
                dxy: XY,
                dz: Z,
                db: B,
            },
        ];

        Self { motions }
    }

    pub fn move_from(&self, state: &State) -> Vec<State> {
        self.motions
            .iter()
            .map(|ctl| {
                State::new(
                    state.x + ctl.dxy * state.b.cos(),
                    state.y + ctl.dxy * state.b.sin(),
                    state.z + ctl.dz,
                    state.bearing() + ctl.db,
                )
            })
            .collect()
    }
}

impl Discretizer {
    fn discrete(&self, state: &State) -> AlignedState {
        AlignedState {
            x: (state.x / self.dx).floor() as i64,
            y: (state.y / self.dy).floor() as i64,
            z: (state.z / self.dz).floor() as i64,
            b: (state.bearing() / self.db).floor() as i64,
        }
    }

    fn continuous(&self, state: &AlignedState) -> State {
        // NOTE: I think this is fine because we are guaranteed that aligned states are in
        // bounds
        let b = state.b as f64 * self.db;
        // FIXME: This assertion fails, i think its related to floating point precision with the
        // many discrete-continuous seesawing.
        // I got a b value of like -3.15000001.
        // The floor probably means that we are rounding out of the -PI lower bound.
        // I dont think this is a HUGE issue since we would just wrap the value when the state is
        // constructed anyways.
        // assert!(b >= -PI && b <= PI);

        State::new(
            state.x as f64 * self.dx,
            state.y as f64 * self.dy,
            state.z as f64 * self.dz,
            b,
        )
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
        let aligned = self.discretizer.discrete(&state);

        if let Some(_) = self.ids.insert(aligned.clone(), id) {
            panic!("attempted to overwrite a state that already exists");
        }

        if let Some(_) = self.vertices.insert(
            id,
            Vertex {
                id,
                state: aligned,
                successors: None,
            },
        ) {
            panic!("id collision");
        }

        id
    }

    fn id(&self, state: &State) -> Option<&Uuid> {
        self.ids.get(&self.discretizer.discrete(state))
    }

    pub fn state(&self, id: &Uuid) -> State {
        self.discretizer.continuous(&self.vertex(id).state)
    }

    pub fn state_count(&self) -> usize {
        assert_eq!(self.ids.len(), self.vertices.len());
        self.ids.len()
    }

    pub fn walk_from(&self, id: &Uuid) -> Vec<Uuid> {
        self.vertex(id).successors.clone().unwrap_or_default()
    }

    pub fn successors(&mut self, id: &Uuid) -> Vec<Uuid> {
        let vertex = self.vertex(id);

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
            .move_from(&self.discretizer.continuous(&vertex.state))
            .into_iter()
            // If the discretization is more coarse than the motion primitives, we can end up with
            // non-unique successor states.
            // FIXME: Why doesn't the self.id check handle this case?
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

        assert_eq!(successors.len(), 9);

        successors
    }
}

const V: f64 = 0.1;
const D_T: f64 = 30.;
const D_Z: f64 = 0.006;
const D_B: f64 = 0.025;

const XY: f64 = V * D_T;
const Z: f64 = D_Z * D_T;
const B: f64 = D_B * D_T;

impl ARAPlanner {
    pub fn create_plan(&self, start: State) -> ARAPlan {
        let mut world = World::new(Discretizer {
            dx: 0.125,
            dy: 0.125,
            dz: 0.05,
            db: 0.05,
        });

        let start_id = world.insert(start).clone();

        let mut plan = ARAPlan {
            world,
            open_set: BinaryHeap::new(),
            closed_set: HashSet::new(),
            incons_set: HashSet::new(),
            g_vals: HashMap::new(),
            h_vals: HashMap::new(),
            eps: self.start_eps,
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

    fn h_val(&self, id: &Uuid) -> f64 {
        let state = self.world.state(id);
        let q0 = [state.x, state.y, state.bearing()].into();
        // TODO: might not need to recompute
        // want to cache this somewhere
        let goal = self.goal_region.center();
        let q1 = [goal.x, goal.y, goal.bearing()].into();
        // TODO: This should maybe be computed by the airplane?
        let rho = D_B / V;
        let shortest_path = DubinsPath::shortest_from(q0, q1, rho).expect("should get a path");
        let dist_xy = shortest_path.length();
        let mut t_min = dist_xy / V;
        let t_z = (state.z - goal.z).abs() / D_Z;
        while t_z < t_min {
            t_min += 2. * PI / D_B;
        }

        ((V * t_min).powf(2.) + (state.z - goal.z).powf(2.)).sqrt()
    }

    fn f_val(&self, id: &Uuid) -> f64 {
        self.g_val(id) + self.eps * self.h_val(id)
    }

    fn open(&mut self, id: Uuid) {
        self.open_set.push(OpenVertex {
            id,
            f: self.f_val(&id),
        })
    }

    fn close(&mut self, id: Uuid) {
        self.closed_set.insert(id);
    }

    fn is_closed(&self, id: &Uuid) -> bool {
        self.closed_set.contains(id)
    }

    fn mark_incons(&mut self, id: Uuid) {
        self.incons_set.insert(id);
    }

    fn suboptimality(&self) -> f64 {
        // TODO: actually implement this
        10.0
    }

    fn clean_open_set(&mut self) {
        while let Some(v) = self.open_set.peek() {
            if v.f != self.f_val(&v.id) {
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
        // TODO: cache the heuristic here
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
        let iters = 1000;
        for _ in 0..iters {
            self.clean_open_set();
            let peeked = self.peek();
            if self.goal_f_val <= self.f_val(&peeked) {
                break;
            }

            let id = self.pop();

            for succ in self.successors(id) {
                let cost = self.g_val(&id) + self.cost(&id, &succ);

                // Save as the goal if inside the goal region
                if self.goal_id.is_none() {
                    if self.goal_region.contains(&self.world.state(&succ)) {
                        println!("found goal region");
                        self.goal_f_val = self.f_val(&succ);
                        self.goal_id = Some(succ);
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
                    // TODO: this should be f_val
                    // otherwise insert it into incons
                    else {
                        self.mark_incons(succ);
                    }
                }
            }
        }

        println!("state count: {}", self.world.state_count());

        let path = format!("path_{iters}.g");
        let file = File::create(&path).unwrap();
        let mut writer = BufWriter::new(file);

        let mut visited = HashSet::new();
        let mut queue: VecDeque<Uuid> = VecDeque::new();

        queue.push_back(self.start_id);

        while let Some(id) = queue.pop_front() {
            visited.insert(id);
            let state = self.world.state(&id);
            let _ = writeln!(
                writer,
                "v {} {} {} {} {}",
                id,
                state.x,
                state.y,
                state.z,
                state.bearing()
            );

            let successors = self.world.walk_from(&id);
            assert!(successors.len() == 9 || successors.len() == 0);

            if let Some(best) = successors.into_iter().min_by(|id1, id2| {
                self.f_val(&id1)
                    .partial_cmp(&self.f_val(&id2))
                    .expect("not nan")
            }) {
                let _ = writeln!(writer, "e {} {}", id, best);
                if !visited.contains(&best) {
                    queue.push_back(best);
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

#[test]
fn distinct_successors() {
    // No successor corresponds to the same aligned state or id.
    let start_state = State::new(10.0, 10.0, 5.0, 0.0);
    let mut world = World::new(Discretizer {
        dx: 0.125,
        dy: 0.125,
        dz: 0.05,
        db: 0.05,
    });

    let start_id = world.insert(start_state);

    let mut ids = HashSet::new();
    let mut states = HashSet::new();

    ids.insert(start_id);
    states.insert(world.vertex(&start_id).state.clone());

    for succ in world.successors(&start_id) {
        let state = &world.vertex(&succ).state;

        println!("{succ}: {state:#?}");
        assert!(!ids.contains(&succ));
        assert!(!states.contains(state));

        ids.insert(succ);
        states.insert(state.clone());
    }
}

#[test]
fn region_center() {
    assert_eq!(
        StateRegion {
            x: -0.5..0.5,
            y: -0.5..0.5,
            z: -0.25..0.25,
            b: -0.125..0.125,
        }
        .center(),
        State::new(0.0, 0.0, 0.0, 0.0)
    );
}
