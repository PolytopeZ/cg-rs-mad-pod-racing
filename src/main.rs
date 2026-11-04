use std::fmt;
use std::io;
use std::ops::{Mul, Sub};

// Const
const MAX_THRUST: i32 = 100;
const MIN_ANGLE: i32 = 90;
const BOOST_ANGLE: i32 = 15;
const BOOST_DISTANCE: i32 = 3500;
const BRAKE_DISTANCE: i32 = 1500;
const BRAKE_ANGLE: i32 = 30;
const DRIFT_FACTOR: i32 = 3;

// Types
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Point {
    x: i32,
    y: i32,
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

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
struct Turn {
    pod: Point,
    checkpoint: Point,
    checkpoint_distance: i32,
    checkpoint_angle: i32,
    opponent: Point,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Thrust {
    Power(i32),
    Boost,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Action {
    target: Point,
    thrust: Thrust,
}

#[derive(Debug, Default)]
struct State {
    boost_used: bool,
    last_pod: Option<Point>,
}

// Outputs
impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self.thrust {
            Thrust::Power(p) => write!(f, "{} {} {}", self.target.x, self.target.y, p),
            Thrust::Boost => write!(f, "{} {} BOOST", self.target.x, self.target.y),
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

fn read_turn() -> Option<Turn> {
    let a = read_ints()?; // x y cp_x cp_y cp_dist cp_angle
    let b = read_ints()?; // opp_x opp_y
    Some(Turn {
        pod: Point { x: a[0], y: a[1] },
        checkpoint: Point { x: a[2], y: a[3] },
        checkpoint_distance: a[4],
        checkpoint_angle: a[5],
        opponent: Point { x: b[0], y: b[1] },
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

fn update_speed(turn: &Turn, state: &mut State) -> Point {
    let speed = match state.last_pod {
        Some(last) => turn.pod - last,
        None => Point::default(),
    };
    state.last_pod = Some(turn.pod);
    speed
}

// Actions
fn decide(turn: &Turn, state: &mut State) -> Action {
    let angle = turn.checkpoint_angle.abs();

    let speed = update_speed(turn, state);

    let thrust =
        if !state.boost_used && angle < BOOST_ANGLE && turn.checkpoint_distance > BOOST_DISTANCE {
            state.boost_used = true;
            Thrust::Boost
        } else {
            Thrust::Power(brake(throttle(angle), angle, turn.checkpoint_distance))
        };

    Action {
        target: turn.checkpoint - speed * DRIFT_FACTOR,
        thrust,
    }
}

// Game Loop
fn main() {
    let mut state = State::default();
    while let Some(turn) = read_turn() {
        let action = decide(&turn, &mut state);
        eprintln!("{:?} -> {}", turn, action);
        println!("{}", action);
    }
}
