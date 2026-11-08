use std::fmt;
use std::io;
use std::ops::{Add, Mul, Sub};

// Const
const MAX_THRUST: i32 = 200;
const MIN_ANGLE: i32 = 90;
const BOOST_ANGLE: i32 = 15;
const BOOST_DISTANCE: i32 = 3500;
const BRAKE_TURNS: f64 = 3.0;
const BRAKE_ANGLE: i32 = 30;
const DRIFT_FACTOR: i32 = 3;
const COLLISION_DISTANCE: i64 = 800;
const SHIELD_MIN: f64 = 0.7;
const SHIELD_CD: i32 = 3;
const CHECKPOINT_RADIUS: i32 = 600;
const CORNER_CUT_TURNS: f64 = 2.0;
const BLOCKER_TURNS: i32 = 3;
const WAIT_DISTANCE: i64 = 2000;
const FRICTION: f64 = 0.85;
const MAX_SPEED: f64 = MAX_THRUST as f64 * FRICTION / (1.0 - FRICTION);
const GLIDE_TURNS: f64 = 1.0 / (1.0 - FRICTION);
const COLLIDE_DISTANCE: i64 = 1000;

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
    simulated: Option<[Pod; 2]>,
}

// Sim
mod sim {
    use super::*;
    const MAX_ROTATION: f64 = 18.0;
    const BOOST_POWER: f64 = 650.0;

    pub fn angle_diff(pod: &Pod, target: Point) -> f64 {
        let dist = target - pod.pos;
        let target = (dist.y as f64).atan2(dist.x as f64).to_degrees();
        (target - pod.angle as f64 + 540.0).rem_euclid(360.0) - 180.0
    }

    fn power(thrust: Thrust) -> f64 {
        match thrust {
            Thrust::Power(p) => p as f64,
            Thrust::Boost => BOOST_POWER,
            Thrust::Shield => 0.0,
        }
    }

    pub fn step(game: &Game, pod: &Pod, action: &Action, first_turn: bool) -> Pod {
        // rotate
        let diff = if action.target == pod.pos {
            0.0
        } else {
            angle_diff(pod, action.target)
        };
        let turn = if first_turn {
            diff
        } else {
            diff.clamp(-MAX_ROTATION, MAX_ROTATION)
        };
        let angle = (pod.angle as f64 + turn).rem_euclid(360.0);

        // speed
        let (sin, cos) = angle.to_radians().sin_cos();
        let pow = power(action.thrust);
        let vx = pod.vel.x as f64 + cos * pow;
        let vy = pod.vel.y as f64 + sin * pow;

        // move
        let x = pod.pos.x as f64 + vx;
        let y = pod.pos.y as f64 + vy;

        // friction and clamp
        let pos = Point {
            x: x.round() as i32,
            y: y.round() as i32,
        };
        let vel = Point {
            x: (vx * FRICTION).trunc() as i32,
            y: (vy * FRICTION).trunc() as i32,
        };

        // cp
        let cps = &game.checkpoints;
        let radius = CHECKPOINT_RADIUS as i64;
        let next_cp = if (cps[pod.next_cp] - pos).norm2() < radius * radius {
            (pod.next_cp + 1) % cps.len()
        } else {
            pod.next_cp
        };

        Pod {
            pos,
            vel,
            angle: angle.round() as i32 % 360,
            next_cp,
        }
    }
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

fn brake(power: i32, angle: i32, turns_left: f64) -> i32 {
    if turns_left < BRAKE_TURNS && angle > BRAKE_ANGLE {
        (power as f64 * turns_left / BRAKE_TURNS).round() as i32
    } else {
        power
    }
}

fn angle_to(pod: &Pod, target: Point, first_turn: bool) -> i32 {
    if first_turn {
        return 0;
    }
    sim::angle_diff(pod, target).round() as i32
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
    let target = aim - drift(pod.vel, aim - pod.pos) * DRIFT_FACTOR;

    let turns_left = distance as f64 / pod.vel.len().max(1.0);
    let angle = angle_to(pod, target, state.turn == 1).abs();
    let thrust = if !state.boost_used
        && angle < BOOST_ANGLE
        && distance > BOOST_DISTANCE
        && pod.next_cp == game.boost_cp
    {
        Thrust::Boost
    } else {
        Thrust::Power(brake(throttle(angle), angle, turns_left))
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
    if (wait - stop_at_point(pod)).norm2() < WAIT_DISTANCE * WAIT_DISTANCE {
        return Action {
            target: pod_target.pos,
            thrust: Thrust::Power(0),
        };
    }

    let target = wait - drift(pod.vel, wait - pod.pos) * DRIFT_FACTOR;
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
        && collision.norm2() as f64 > (SHIELD_MIN * MAX_SPEED).powi(2)
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

fn drift(vel: Point, direction: Point) -> Point {
    let dist = direction.norm2();
    if dist == 0 {
        return vel; // already on it
    }
    let k = vel.dot(direction) as f64 / dist as f64;
    Point {
        x: vel.x - (direction.x as f64 * k).round() as i32,
        y: vel.y - (direction.y as f64 * k).round() as i32,
    }
}

fn stop_at_point(pod: &Pod) -> Point {
    Point {
        x: pod.pos.x + (pod.vel.x as f64 * GLIDE_TURNS).round() as i32,
        y: pod.pos.y + (pod.vel.y as f64 * GLIDE_TURNS).round() as i32,
    }
}

fn check_simulation(turn: &Turn, state: &State) {
    let simulated = match state.simulated {
        Some(p) => p,
        None => return,
    };

    let all = turn.all();
    for i in 0..2 {
        let (s, a) = (simulated[i], turn.my_pods[i]);
        let dpos = a.pos - s.pos;
        let dvel = a.vel - s.vel;
        let dangle = (a.angle - s.angle + 540).rem_euclid(360) - 180;
        let cp_ok = a.next_cp == s.next_cp;
        let contact = all
            .iter()
            .enumerate()
            .any(|(j, o)| j != i && (o.pos - a.pos).norm2() < COLLIDE_DISTANCE * COLLIDE_DISTANCE);
        eprintln!(
            "pod {} err pos ({},{}) vel ({},{}) angle {} cp_ok {}{}",
            i,
            dpos.x,
            dpos.y,
            dvel.x,
            dvel.y,
            dangle,
            cp_ok,
            if contact { " (contact?)" } else { "" }
        );
    }
}

// Actions
fn decide(game: &Game, turn: &Turn, state: &mut State) -> [Action; 2] {
    state.turn += 1;
    check_simulation(turn, state);
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

    let first_turn = state.turn == 1;
    state.simulated = Some([
        sim::step(game, &turn.my_pods[0], &actions[0], first_turn),
        sim::step(game, &turn.my_pods[1], &actions[1], first_turn),
    ]);
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
