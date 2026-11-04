use std::fmt;
use std::io;

// Const
const MAX_THRUST: i32 = 100;
const MIN_ANGLE: i32 = 90;
const BOOST_ANGLE: i32 = 15;
const BOOST_DISTANCE: i32 = 3500;

// Types
#[derive(Debug, Clone, Copy, PartialEq)]
struct Point {
    x: i32,
    y: i32,
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

// Actions
fn decide(turn: &Turn, state: &mut State) -> Action {
    let angle = turn.checkpoint_angle.abs();

    let thrust = if angle > MIN_ANGLE {
        Thrust::Power(0)
    } else if !state.boost_used && angle < BOOST_ANGLE && turn.checkpoint_distance > BOOST_DISTANCE
    {
        state.boost_used = true;
        Thrust::Boost
    } else {
        Thrust::Power(MAX_THRUST)
    };

    Action {
        target: turn.checkpoint,
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
