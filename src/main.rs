use std::fmt;
use std::io;
use std::ops::{Add, Mul, Sub};

// Const
const MAX_THRUST: i32 = 100;
const MIN_ANGLE: i32 = 90;
const BOOST_ANGLE: i32 = 15;
const BOOST_DISTANCE: i32 = 3500;
const BRAKE_DISTANCE: i32 = 1500;
const BRAKE_ANGLE: i32 = 30;
const DRIFT_FACTOR: i32 = 3;
const COLLISION_DISTANCE: i64 = 800;
const SHIELD_MIN: i64 = 400;
const SHIELD_CD: i32 = 3;
const CHECKPOINT_RADIUS: i32 = 600;
const CORNER_CUT_TURNS: f64 = 2.0;
const BLOCKER_TURNS: i32 = 3;
const WAIT_DISTANCE: i64 = 2000;

// Types
#[allow(dead_code)]
#[derive(Debug)]
struct Game {
    laps: i32,
    checkpoints: Vec<Point>,
    boost_cp: usize,
}

#[derive(Debug, Clone, Copy)]
struct Pod {
    pos: Point,
    vel: Point,
    angle: i32,
    next_cp: usize,
}

#[derive(Debug)]
struct Turn {
    my_pods: [Pod; 2],
    opp_pods: [Pod; 2],
}

impl Turn {
    fn all(&self) -> [Pod; 4] {
        [
            self.my_pods[0],
            self.my_pods[1],
            self.opp_pods[0],
            self.opp_pods[1],
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Point {
    x: i32,
    y: i32,
}

impl Add for Point {
    type Output = Point;
    fn add(self, other: Point) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Point {
    type Output = Point;
    fn sub(self, other: Point) -> Point {
        Point {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<i32> for Point {
    type Output = Point;
    fn mul(self, k: i32) -> Point {
        Point {
            x: self.x * k,
            y: self.y * k,
        }
    }
}

impl Point {
    fn norm2(self) -> i64 {
        let (x, y) = (self.x as i64, self.y as i64);
        x * x + y * y
    }
    fn len(self) -> f64 {
        (self.norm2() as f64).sqrt()
    }
    fn dot(self, other: Point) -> i64 {
        self.x as i64 * other.x as i64 + self.y as i64 * other.y as i64
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Thrust {
    Power(i32),
    Boost,
    Shield,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Action {
    target: Point,
    thrust: Thrust,
}

#[derive(Debug, Default)]
struct State {
    turn: i32,
    boost_used: bool,
    shield_cd: [i32; 2],
    last_cp: [Option<usize>; 4],
    passed_cp: [i32; 4],
    runner: usize,
}

// Outputs
impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self.thrust {
            Thrust::Power(p) => write!(f, "{} {} {}", self.target.x, self.target.y, p),
            Thrust::Boost => write!(f, "{} {} BOOST", self.target.x, self.target.y),
            Thrust::Shield => write!(f, "{} {} SHIELD", self.target.x, self.target.y),
        }
    }
}

// Inputs
fn read_ints() -> Option<Vec<i32>> {
    let mut line = String::new();
    let bytes = io::stdin().read_line(&mut line).ok()?;
    if bytes == 0 {
        return None;
    }
    Some(
        line.split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect(),
    )
}

fn read_game() -> Option<Game> {
    let laps = read_ints()?[0];
    let count = read_ints()?[0];
    let mut checkpoints = Vec::new();
    for _ in 0..count {
        let v = read_ints()?;
        checkpoints.push(Point { x: v[0], y: v[1] });
    }
    let boost_cp = best_boost_cp(&checkpoints);
    Some(Game {
        laps,
        checkpoints,
        boost_cp,
    })
}

fn read_pod() -> Option<Pod> {
    let v = read_ints()?;
    Some(Pod {
        pos: Point { x: v[0], y: v[1] },
        vel: Point { x: v[2], y: v[3] },
        angle: v[4],
        next_cp: v[5] as usize,
    })
}

fn read_turn() -> Option<Turn> {
    Some(Turn {
        my_pods: [read_pod()?, read_pod()?],
        opp_pods: [read_pod()?, read_pod()?],
    })
}

// Utils
fn throttle(angle: i32) -> i32 {
    if angle >= MIN_ANGLE {
        return 0;
    }
    // change speed depending on angle
    let factor = (angle as f64).to_radians().cos();
    (MAX_THRUST as f64 * factor).round() as i32
}

fn brake(power: i32, angle: i32, distance: i32) -> i32 {
    if distance < BRAKE_DISTANCE && angle > BRAKE_ANGLE {
        power * distance / BRAKE_DISTANCE
    } else {
        power
    }
}

fn angle_to(pod: &Pod, target: Point, first_turn: bool) -> i32 {
    if first_turn {
        return 0;
    }
    let d = target - pod.pos;
    let wanted = (d.y as f64).atan2(d.x as f64).to_degrees();
    let diff = (wanted - pod.angle as f64 + 540.0).rem_euclid(360.0) - 180.0;
    diff.round() as i32
}

fn race(game: &Game, pod: &Pod, state: &State) -> Action {
    let cp = game.checkpoints[pod.next_cp];
    let cps = &game.checkpoints;
    let distance = (cp - pod.pos).len() as i32;
    let next = cps[(pod.next_cp + 1) % cps.len()];
    let gap = (distance - CHECKPOINT_RADIUS) as f64;
    let aim = if gap < pod.vel.len() * CORNER_CUT_TURNS {
        next
    } else {
        cp
    };
    let target = aim - pod.vel * DRIFT_FACTOR;

    let angle = angle_to(pod, target, state.turn == 1).abs();
    let thrust = if !state.boost_used
        && angle < BOOST_ANGLE
        && distance > BOOST_DISTANCE
        && pod.next_cp == game.boost_cp
    {
        Thrust::Boost
    } else {
        Thrust::Power(brake(throttle(angle), angle, distance))
    };
    Action { target, thrust }
}

fn update_progress(turn: &Turn, state: &mut State) {
    for (i, pod) in turn.all().iter().enumerate() {
        if let Some(prev) = state.last_cp[i] {
            if prev != pod.next_cp {
                state.passed_cp[i] += 1;
            }
        }
        state.last_cp[i] = Some(pod.next_cp);
    }
}

fn opp_leader(game: &Game, turn: &Turn, state: &State) -> Pod {
    let key = |i: usize| {
        let pod = turn.opp_pods[i];
        let left = (game.checkpoints[pod.next_cp] - pod.pos).norm2();
        (state.passed_cp[2 + i], -left)
    };
    if key(0) >= key(1) {
        turn.opp_pods[0]
    } else {
        turn.opp_pods[1]
    }
}

fn block(game: &Game, pod: &Pod, pod_target: &Pod, state: &State) -> Action {
    let cps = &game.checkpoints;
    let first_turn = state.turn == 1;
    let target_cp = cps[pod_target.next_cp];

    let ahead = (target_cp - pod.pos).norm2() < (target_cp - pod_target.pos).norm2();
    if ahead {
        let target = pod_target.pos + pod_target.vel * BLOCKER_TURNS;
        let angle = angle_to(pod, target, first_turn).abs();
        return Action {
            target,
            thrust: Thrust::Power(throttle(angle)),
        };
    }

    let wait = cps[(pod_target.next_cp + 1) % cps.len()];
    if (wait - pod.pos).norm2() < WAIT_DISTANCE * WAIT_DISTANCE {
        return Action {
            target: pod_target.pos,
            thrust: Thrust::Power(0),
        };
    }

    let target = wait - pod.vel * DRIFT_FACTOR;
    let angle = angle_to(pod, target, first_turn).abs();
    Action {
        target,
        thrust: Thrust::Power(throttle(angle)),
    }
}

fn will_hit(me: &Pod, other: &Pod) -> bool {
    let dist = (me.pos + me.vel) - (other.pos + other.vel);
    let collision = me.vel - other.vel;
    dist.norm2() < COLLISION_DISTANCE * COLLISION_DISTANCE
        && collision.norm2() > SHIELD_MIN * SHIELD_MIN
}

fn shield_or_speed(
    me: &Pod,
    turn: &Turn,
    cooldown: &mut i32,
    thrust: Thrust,
    facing: bool,
) -> Thrust {
    if *cooldown > 0 {
        *cooldown -= 1;
        return Thrust::Power(0);
    }
    let hit = |other: &Pod| will_hit(me, other) && (!facing || me.vel.dot(other.vel) < 0);
    if turn.opp_pods.iter().any(hit) {
        *cooldown = SHIELD_CD;
        return Thrust::Shield;
    }
    thrust
}

fn choose_runner(state: &State) -> usize {
    let other = 1 - state.runner;
    if state.passed_cp[other] > state.passed_cp[state.runner] {
        other
    } else {
        state.runner
    }
}

fn best_boost_cp(cps: &[Point]) -> usize {
    let n = cps.len();
    let path = |i: usize| (cps[(i + 1) % n] - cps[i]).norm2();
    let start = (0..n).max_by_key(|&i| path(i)).unwrap_or(0);
    (start + 1) % n
}

// Actions
fn decide(game: &Game, turn: &Turn, state: &mut State) -> [Action; 2] {
    state.turn += 1;
    update_progress(turn, state);
    state.runner = choose_runner(state);
    let pod_runner = state.runner;
    let pod_blocker = 1 - pod_runner;

    let leader = opp_leader(game, turn, state);
    let runner = race(game, &turn.my_pods[pod_runner], state);
    let blocker = block(game, &turn.my_pods[pod_blocker], &leader, state);

    let mut actions = if pod_runner == 0 {
        [runner, blocker]
    } else {
        [blocker, runner]
    };

    for i in 0..2 {
        actions[i].thrust = shield_or_speed(
            &turn.my_pods[i],
            turn,
            &mut state.shield_cd[i],
            actions[i].thrust,
            i == pod_runner,
        );
        if actions[i].thrust == Thrust::Boost {
            state.boost_used = true;
        }
    }
    actions
}

// Game Loop
fn main() {
    let game = match read_game() {
        Some(g) => g,
        None => return,
    };
    eprintln!("{:?}", game);
    let mut state = State::default();
    while let Some(turn) = read_turn() {
        for action in decide(&game, &turn, &mut state) {
            println!("{}", action);
        }
    }
}
