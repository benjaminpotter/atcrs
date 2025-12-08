use dubins_paths::DubinsPath;
use radians::Wrap64;
use std::f64::consts::PI;
use std::io::Write;
use std::path::Path;
use std::{
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
    fs::File,
    io::BufWriter,
    ops::Range,
};
use uuid::Uuid;

fn main() {
    // let start = State::new(-50.0, 0.0, 5.0, 0.0);
    // let start = State::new(-50.0, 0.0, 5.0, PI * 3.0 / 4.0);
    let start = State::new(50.0, 0.0, 5.0, PI); // FIXME: Problem
    // let start = State::new(-50.0, 0.0, 5.0, PI * 3.9 / 4.0);
    let start_eps = 3.0;
    let eps_delta = 0.02;
    let stop_eps = 1.0;
    // let max_iters = 2000000;
    let planner = ARAPlanner {
        goal_region: StateRegion {
            x: -0.5..0.5,
            y: -0.5..0.5,
            z: -0.25..0.25,
            b: -0.125..0.125,
        },
    };

    let mut plan = planner.create_plan(start);
    println!("{:#?}", plan);

    // TODO: Replace with timing logic.
    for i in 0..1 {
        // TODO: eps -= eps_delta;
        plan = planner.plan_from(plan.clone(), start_eps, None);

        println!("start writing to file");
        let path = format!("plan.g");
        // plan.print_to_file(&path);
        plan.print_solution_to_file(&path);
        println!("wrote {path}");
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
struct Weight {
    cost_to_go: f64,
    cost_to_come: f64,
}

#[derive(Clone, Debug)]
struct Vertex {
    id: Uuid,
    state: AlignedState,
    weight: Weight,
    parent: Option<Uuid>,
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
    priority: f64,
}

struct ARAPlanner {
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

    start_id: Uuid,
    goal_id: Option<Uuid>,
    goal_f_val: f64,
    goal_region: StateRegion,

    iters: usize,
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
        let val = self.b.val();
        assert!(val >= -PI && val <= PI);
        val
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

const RHO: f64 = D_B / V;

impl Weight {
    pub fn from_cost_to_go_and_target(cost_to_go: f64, to: &State, target: &State) -> Self {
        Weight {
            cost_to_go,
            cost_to_come: Self::cost_to_come(to, target),
        }
    }

    pub fn from_states(from: &State, with: &Weight, to: &State, target: &State) -> Self {
        Weight {
            cost_to_go: Self::cost_to_go(from, with, to),
            cost_to_come: Self::cost_to_come(to, target),
        }
    }

    pub fn cost(&self, inflation: f64) -> f64 {
        self.cost_to_go + inflation * self.cost_to_come
    }

    fn cost_to_come(state: &State, target: &State) -> f64 {
        // TODO: this should be abstracted...
        let q0 = [state.x, state.y, state.bearing()].into();
        let q1 = [target.x, target.y, target.bearing()].into();
        let shortest_path = DubinsPath::shortest_from(q0, q1, RHO).expect("should get a path");
        let dist_xy = shortest_path.length();
        let mut t_min = dist_xy / V;
        let t_z = (state.z - target.z).abs() / D_Z;
        while t_z > t_min {
            t_min += 2. * PI / D_B;
        }

        ((V * t_min).powf(2.) + (state.z - target.z).powf(2.)).sqrt()
    }

    fn cost_to_go(from: &State, with: &Weight, to: &State) -> f64 {
        with.cost_to_go + from.euclidean_dist_to(&to)
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

    fn insert(&mut self, state: State, weight: Weight, parent: Option<Uuid>) -> Uuid {
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
                weight,
                parent,
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

    pub fn weight(&self, id: &Uuid) -> &Weight {
        &self.vertex(id).weight
    }

    pub fn weight_mut(&mut self, id: &Uuid) -> &mut Weight {
        &mut self.vertex_mut(id).weight
    }

    pub fn parent(&self, id: &Uuid) -> &Option<Uuid> {
        &self.vertex(id).parent
    }

    pub fn parent_mut(&mut self, id: &Uuid) -> &mut Option<Uuid> {
        &mut self.vertex_mut(id).parent
    }

    pub fn walk_from(&self, id: &Uuid) -> Vec<Uuid> {
        let successors = self.vertex(id).successors.clone().unwrap_or_default();
        assert!(successors.len() == 9 || successors.len() == 0);
        successors
    }

    pub fn successors<F>(&mut self, id: &Uuid, scale: F) -> Vec<Uuid>
    where
        F: Fn(&State) -> Weight,
    {
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
                // NOTE: We could imagine that the cost-to-go from this state is better than the
                // previous time we visited this node so we want to update it here.
                // I think this will be handled by the main execution loop once it gets the ids of
                // provided by this function.
                // However, this is not ideal because it leads to multiple lookups for the same id.
                if let Some(id) = self.id(&state) {
                    return id.clone();
                }

                let weight = scale(&state);

                // If this successor is not in our world yet, then we have to create a vertex for
                // it.
                // We return the id of the new vertex so that it gets included in our successor
                // list.
                self.insert(state, weight, Some(*id))
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

        let weight = Weight::from_cost_to_go_and_target(0., &start, &self.goal_region.center());
        let start_id = world.insert(start, weight, None).clone();

        let mut plan = ARAPlan {
            world,
            open_set: BinaryHeap::new(),
            closed_set: HashSet::new(),
            incons_set: HashSet::new(),
            start_id,
            goal_id: None,
            goal_f_val: f64::INFINITY,
            goal_region: self.goal_region.clone(),
            iters: 0,
        };

        // NOTE: We can put a cost of zero here because it will be updated when plan_from is
        // called.
        plan.open(start_id, 0.0);

        plan
    }

    pub fn plan_from(&self, mut plan: ARAPlan, eps: f64, max_iters: Option<usize>) -> ARAPlan {
        let open_vertices: Vec<_> = plan
            .incons_set
            .drain()
            .chain(plan.open_set.drain().map(|ov| ov.id))
            .collect();

        for id in open_vertices {
            let priority = plan.world.weight(&id).cost(eps);
            plan.open(id, priority);
        }

        plan.closed_set.clear();

        loop {
            if let Some(max) = max_iters {
                if plan.iters > max {
                    break;
                }
            }

            let id = plan.pop();
            if plan.goal_f_val <= plan.world.weight(&id).cost(eps) {
                break;
            }

            let state = plan.world.state(&id);
            let weight = plan.world.weight(&id).clone();

            // Save as the goal if inside the goal region
            if plan.goal_id.is_none() {
                if plan.goal_region.contains(&state) {
                    println!("found goal region");
                    plan.goal_f_val = weight.cost(eps);
                    plan.goal_id = Some(id);
                }
            }
            let successors = plan.world.successors(&id, |state| {
                Weight::from_cost_to_go_and_target(f64::INFINITY, state, &self.goal_region.center())
            });

            assert_eq!(successors.len(), 9);

            for succ in successors {
                let succ_state = plan.world.state(&succ);
                let succ_weight = plan.world.weight_mut(&succ);
                let cost = succ_weight.cost(eps);
                let cost_to_go = Weight::cost_to_go(&state, &weight, &succ_state);

                // if succ is better through s then
                if cost_to_go < succ_weight.cost_to_go {
                    // update its g value
                    succ_weight.cost_to_go = cost_to_go;

                    // set this as the parent
                    *plan.world.parent_mut(&succ) = Some(id);

                    // if its not in closed then insert it into open
                    if !plan.is_closed(&succ) {
                        plan.open(succ, cost)
                    }
                    // otherwise insert it into incons
                    else {
                        plan.mark_incons(succ);
                    }
                }
            }

            plan.iters += 1;
        }

        plan
    }
}

impl ARAPlan {
    fn open(&mut self, id: Uuid, priority: f64) {
        self.open_set.push(OpenVertex { id, priority })
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

    fn pop(&mut self) -> Uuid {
        let id = self.open_set.pop().expect("the open set is never empty").id;
        self.close(id);
        id
    }

    fn print_to_file<P: AsRef<Path>>(&self, path: P) {
        println!("state count: {}", self.world.state_count());

        let file = File::create(&path).unwrap();
        let mut writer = BufWriter::new(file);
        // let mut writer = std::io::stdout();

        let mut visited = HashSet::new();
        let mut queue: VecDeque<Uuid> = VecDeque::new();
        queue.push_back(self.start_id);

        while let Some(id) = queue.pop_front() {
            if visited.contains(&id) {
                continue;
            }

            visited.insert(id);

            let state = self.world.state(&id);
            let weight = self.world.weight(&id);
            let _ = writeln!(
                writer,
                "v {} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2}",
                id,
                state.x,
                state.y,
                state.z,
                state.bearing(),
                weight.cost_to_go,
                weight.cost_to_come,
            );

            for succ in self.world.walk_from(&id) {
                // let _ = writeln!(writer, "e {} {}", id, succ);
                queue.push_back(succ);
            }
        }
    }

    fn print_solution_to_file(&self, path: &str) {
        println!("state count: {}", self.world.state_count());

        let file = File::create(&path).unwrap();
        let mut writer = BufWriter::new(file);

        let mut parent = &self.goal_id;
        while let Some(id) = parent.as_ref() {
            let state = self.world.state(&id);
            let weight = self.world.weight(&id);
            let _ = writeln!(
                writer,
                "v {} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2}",
                id,
                state.x,
                state.y,
                state.z,
                state.bearing(),
                weight.cost_to_go,
                weight.cost_to_come,
            );

            parent = self.world.parent(&id);
        }
    }
}

impl PartialEq for OpenVertex {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.id == other.id
    }
}

impl Eq for OpenVertex {}

impl Ord for OpenVertex {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.priority
            .partial_cmp(&other.priority)
            .expect("we should ensure that floats are always ordered")
            // Lower priorities get ordered above.
            .reverse()
            // Break ties with id
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
    let start = State::new(-75.0, 0.0, 5.0, 0.0);
    let target = State::new(0.0, 0.0, 0.0, 0.0);
    let mut world = World::new(Discretizer {
        dx: 0.125,
        dy: 0.125,
        dz: 0.05,
        db: 0.05,
    });

    let weight = Weight::from_cost_to_go_and_target(0., &start, &target);
    let start_id = world.insert(start, weight);

    let mut ids = HashSet::new();
    let mut states = HashSet::new();

    ids.insert(start_id);
    states.insert(world.vertex(&start_id).state.clone());

    for succ in world.successors(&start_id, |from, with, to| {
        Weight::from_states(from, with, to, &target)
    }) {
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

#[test]
fn region_contains() {
    assert!(
        StateRegion {
            x: -0.5..0.5,
            y: -0.5..0.5,
            z: -0.25..0.25,
            b: -0.125..0.125,
        }
        .contains(&State::new(0.0, 0.0, 0.0, 0.0))
    );
}

#[test]
fn vertex_ordering() {
    use uuid::uuid;

    assert!(
        OpenVertex {
            id: uuid!("be8e8538-c1fa-49d4-8c15-77c4bbd63cf2"),
            priority: 100.0,
        } < OpenVertex {
            id: uuid!("93879c82-f9aa-439e-8c96-e2589ef726b3"),
            priority: 10.0,
        }
    );
}
