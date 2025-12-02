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
        bearing: 0.0,
    };

    let planner = ARAPlanner {
        start_eps: 4.0,
        eps_delta: 0.5,
        suboptimality: 1.0,
        goal_region: StateRegion {
            x: -500.0..500.0,
            y: -500.0..500.0,
            z: -25.0..25.0,
            bearing: -0.125..0.125,
        },
    };

    let mut plan = planner.plan(start_state);

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
    x: f64,
    y: f64,
    z: f64,
    bearing: f64,
}

#[derive(Clone, Debug)]
struct StateRegion {
    x: Range<f64>,
    y: Range<f64>,
    z: Range<f64>,
    bearing: Range<f64>,
}

#[derive(Clone, Debug)]
struct Vertex {
    id: Uuid,
    state: State,
    g: f64,
    h: f64,
    successors: Vec<Uuid>,
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
    graph: HashMap<Uuid, Vertex>,

    // This will store each state in the open set.
    // Popping will give the min state.
    // It will not guarantee that the popped value is live.
    // Must check with the graph to test for differences.
    open_set: BinaryHeap<OpenVertex>,
    closed_set: HashSet<Uuid>,
    incons_set: HashSet<Uuid>,

    eps: f64,

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
            + (self.bearing - other.bearing).powf(2.))
        .sqrt()
    }
}

impl StateRegion {
    pub fn contains(&self, state: &State) -> bool {
        self.x.contains(&state.x)
            && self.y.contains(&state.y)
            && self.z.contains(&state.z)
            && self.bearing.contains(&state.bearing)
    }

    pub fn center(&self) -> State {
        todo!()
    }
}

impl Vertex {
    pub fn f_val(&self) -> f64 {
        self.g + self.h
    }
}

impl ARAPlanner {
    pub fn plan(&self, start: State) -> ARAPlan {
        let mut plan = ARAPlan::new(start, self.goal_region.clone(), self.start_eps);
        plan.improve_path();
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
    fn new(start: State, goal_region: StateRegion, eps: f64) -> Self {
        let mut graph = HashMap::new();
        let id = Uuid::new_v4();
        graph.insert(
            id,
            Vertex {
                id,
                state: start,
                g: 0.,
                h: 0., // TODO: to the center of the goal region
                successors: Vec::new(),
            },
        );

        let mut open_set = BinaryHeap::new();
        open_set.push(OpenVertex {
            id,
            f: 0., // TODO: update this with the actual cost
        });

        Self {
            graph,
            open_set,
            closed_set: HashSet::new(),
            incons_set: HashSet::new(),
            eps,
            start_id: id,
            goal_id: None,
            goal_f_val: f64::INFINITY,
            goal_region,
        }
    }

    fn open(&mut self, id: Uuid) {
        todo!()
    }

    fn close(&mut self, id: Uuid) {
        self.closed_set.insert(id);
    }

    fn is_closed(&self, id: Uuid) -> bool {
        todo!()
    }

    fn mark_incons(&self, id: Uuid) {
        todo!()
    }

    fn suboptimality(&self) -> f64 {
        todo!()
    }

    fn clean_open_set(&mut self) {
        while let Some(v) = self.open_set.peek() {
            if v.f != self.vertex(&v.id).f_val() {
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
        // Nine cases of successor
        // u_z in { -dz, 0, dz }
        // u_b in { -db, 0, db }

        const V: f64 = 100.;
        const DT: f64 = 30.;
        const DXY: f64 = V * DT;
        const DZ: f64 = 6. * DT;
        const DB: f64 = 0.025 * DT;
        static CONTROLS: [(f64, f64); 9] = [
            (-DZ, -DB),
            (-DZ, 0.),
            (-DZ, DB),
            (0., -DB),
            (0., 0.),
            (0., DB),
            (DZ, -DB),
            (DZ, 0.),
            (DZ, DB),
        ];

        if self.vertex(&id).successors.len() == 0 {
            // FIXME: this is going to create new vertices even when those vertices already exist
            let state = self.vertex(&id).state.clone();
            let succ_ids: Vec<_> = CONTROLS
                .iter()
                .map(|(dz, db)| {
                    // generate successors
                    State {
                        x: state.x + DXY * state.bearing.cos(),
                        y: state.y + DXY * state.bearing.sin(),
                        z: state.z + dz,
                        bearing: state.bearing + db,
                    }
                })
                .map(|state| Vertex {
                    id: Uuid::new_v4(),
                    state,
                    g: 0.,
                    h: 0.,
                    successors: Vec::new(),
                })
                .map(|vertex| {
                    let id = vertex.id;
                    self.graph.insert(id, vertex);
                    id
                })
                .collect();

            self.vertex_mut(&id).successors = succ_ids;
        }

        self.vertex(&id).successors.clone()
    }

    fn vertex(&self, id: &Uuid) -> &Vertex {
        self.graph
            .get(&id)
            .expect("never look for a vertex that does not exist")
    }

    fn vertex_mut(&mut self, id: &Uuid) -> &mut Vertex {
        self.graph
            .get_mut(&id)
            .expect("never look for a vertex that does not exist")
    }

    fn cost(&self, start: &Uuid, end: &Uuid) -> f64 {
        // This function can only be used to compare adjacent vertices
        assert!(self.vertex(&start).successors.contains(&end));

        self.vertex(&start)
            .state
            .euclidean_dist_to(&self.vertex(&end).state)
    }

    fn improve_path(&mut self) {
        loop {
            self.clean_open_set();
            let peeked = self.peek();
            if self.goal_f_val <= self.vertex(&peeked).f_val() {
                break;
            }

            let id = self.pop();

            for succ in self.successors(id) {
                let cost = self.vertex(&id).g + self.cost(&id, &succ);

                // Save as the goal if inside the goal region
                if self.goal_id.is_none() {
                    if self.goal_region.contains(&self.vertex(&succ).state) {
                        self.goal_id = Some(succ);
                        self.goal_f_val = cost;
                    }
                }

                // if succ is better through s then
                if cost < self.vertex(&succ).g {
                    // update its g value
                    self.vertex_mut(&succ).g = cost;

                    // if its not in closed then insert it into open
                    if !self.is_closed(succ) {
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
