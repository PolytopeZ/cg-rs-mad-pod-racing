use std::fmt;
use std::fmt::write;
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
const COLLISION_DISTANCE: i32 = 800;
const SHIELD_MIN: i32 = 400;
const SHIELD_CD: i32 = 3;

// Types
#[allow(dead_code)]
#[derive(Debug)]
struct Game {
    laps: i32,
    checkpoints: Vec<Point>,
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
    Some(Game { laps, checkpoints })
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
    let distance = (cp - pod.pos).len() as i32;
    let target = cp - pod.vel * DRIFT_FACTOR;

    let angle = angle_to(pod, target, state.turn == 1).abs();
    let thrust = if !state.boost_used && angle < BOOST_ANGLE && distance > BOOST_DISTANCE {
        Thrust::Boost
    } else {
        Thrust::Power(brake(throttle(angle), angle, distance))
    };
    Action { target, thrust }
}

// Actions
fn decide(game: &Game, turn: &Turn, state: &mut State) -> [Action; 2] {
    state.turn += 1;
    let actions = [
        race(game, &turn.my_pods[0], state),
        race(game, &turn.my_pods[1], state),
    ];
    if actions.iter().any(|a| a.thrust == Thrust::Boost) {
        state.boost_used = true;
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
