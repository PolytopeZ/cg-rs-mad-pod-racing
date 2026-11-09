use std::fmt;
use std::io;
use std::iter::Successors;
use std::ops::{Add, Mul, Sub};
use std::time::Instant;

use crate::sim::angle_diff;

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
const AIM_DISTANCE: f64 = 10000.0;
const SIM_DEPTH: usize = 10;
const CP_PASS_REWARD: f64 = 100000.0;
const SEARCH_ANGLES: [f64; 5] = [-18.0, -9.0, 0.0, 9.0, 18.0];
const SEARCH_THRUSTS: [i32; 5] = [0, 50, 100, 150, 200];
const PLAN_TURNS: usize = 6;
const MAX_TURN: f64 = 18.0;
const SEARCH_MS: u128 = 40;
const TURN_COST: f64 = MAX_THRUST as f64 / (1.0 - FRICTION);

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
    rng: Rng,
    plans: [Plan; 2],
}

#[derive(Debug, Clone, Copy)]
struct Rng(u64);

impl Default for Rng {
    fn default() -> Rng {
        Rng(0x9_24C8_73FA_7E41)
    }
}

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        let unit = (self.next() >> 11) as f64 / (1u64 << 53) as f64;
        lo + (hi - lo) * unit
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct Move {
    turn: f64,
    thrust: i32,
}

type Plan = [Move; PLAN_TURNS];

// Sim
mod sim {
    use super::*;
    const MAX_ROTATION: f64 = 18.0;
    const BOOST_POWER: f64 = 650.0;
    const POD_RADIUS: f64 = 400.0;
    const MIN_IMPULSE: f64 = 120.0;
    const SHIELD_MASS: f64 = 10.0;
    const MAX_EVENTS: usize = 20;

    enum Event {
        Pods(usize, usize),
        Checkpoint(usize),
    }

    #[derive(Debug, Clone, Copy)]
    struct Body {
        pos: V,
        vel: V,
        angle: f64,
        mass: f64,
        next_cp: usize,
    }

    #[derive(Debug, Clone, Copy, Default)]
    pub struct V {
        pub x: f64,
        pub y: f64,
    }

    impl V {
        fn dot(self, o: V) -> f64 {
            self.x * o.x + self.y * o.y
        }
    }

    impl Add for V {
        type Output = V;
        fn add(self, o: V) -> V {
            V {
                x: self.x + o.x,
                y: self.y + o.y,
            }
        }
    }

    impl Sub for V {
        type Output = V;
        fn sub(self, o: V) -> V {
            V {
                x: self.x - o.x,
                y: self.y - o.y,
            }
        }
    }

    impl Mul<f64> for V {
        type Output = V;
        fn mul(self, k: f64) -> V {
            V {
                x: self.x * k,
                y: self.y * k,
            }
        }
    }

    impl From<Point> for V {
        fn from(p: Point) -> V {
            V {
                x: p.x as f64,
                y: p.y as f64,
            }
        }
    }

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

    fn steer(pod: &Pod, action: &Action, first_turn: bool) -> Body {
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
        let (sin, cos) = angle.to_radians().sin_cos();
        let p = power(action.thrust);
        Body {
            pos: V::from(pod.pos),
            vel: V::from(pod.vel)
                + V {
                    x: cos * p,
                    y: sin * p,
                },
            angle,
            mass: if action.thrust == Thrust::Shield {
                SHIELD_MASS
            } else {
                1.0
            },
            next_cp: pod.next_cp,
        }
    }

    fn end(b: &Body) -> Pod {
        Pod {
            pos: Point {
                x: b.pos.x.round() as i32,
                y: b.pos.y.round() as i32,
            },
            vel: Point {
                x: (b.vel.x * FRICTION).trunc() as i32,
                y: (b.vel.y * FRICTION).trunc() as i32,
            },
            angle: b.angle.round() as i32 % 360,
            next_cp: b.next_cp,
        }
    }

    fn hit_time(pa: V, va: V, pb: V, vb: V, radius: f64, max_t: f64) -> Option<f64> {
        let p = pa - pb;
        let v = va - vb;
        let b = p.dot(v);
        if b >= 0.0 {
            return None;
        }
        let c = p.dot(p) - radius * radius;
        if c <= 0.0 {
            return Some(0.0);
        }
        let a = v.dot(v);
        let disc = b * b - a * c;
        if disc < 0.0 {
            return None;
        }
        let t = (-b - disc.sqrt()) / a;
        if t <= max_t { Some(t) } else { None }
    }

    fn bounce(a: &mut Body, b: &mut Body) {
        let n = a.pos - b.pos;
        let dv = a.vel - b.vel;
        let m = (a.mass + b.mass) / (a.mass * b.mass);
        let mut f = n * (n.dot(dv) / (n.dot(n) * m));
        a.vel = a.vel - f * (1.0 / a.mass);
        b.vel = b.vel + f * (1.0 / b.mass);
        let impulse = f.dot(f).sqrt();
        if impulse > 0.0 && impulse < MIN_IMPULSE {
            f = f * (MIN_IMPULSE / impulse);
        }
        a.vel = a.vel - f * (1.0 / a.mass);
        b.vel = b.vel + f * (1.0 / b.mass);
    }

    fn play(game: &Game, bodies: &mut [Body]) {
        let cps = &game.checkpoints;
        let radius = CHECKPOINT_RADIUS as f64;
        let mut t = 0.0;
        for _ in 0..MAX_EVENTS {
            let left = 1.0 - t;
            let mut first: Option<(f64, Event)> = None;
            for i in 0..bodies.len() {
                let bi = bodies[i];
                let cp = V::from(cps[bi.next_cp]);
                if let Some(dt) = hit_time(bi.pos, bi.vel, cp, V::default(), radius, left) {
                    if first.as_ref().map_or(true, |(best, _)| dt < *best) {
                        first = Some((dt, Event::Checkpoint(i)));
                    }
                }
                for j in i + 1..bodies.len() {
                    let bj = bodies[j];
                    if let Some(dt) =
                        hit_time(bi.pos, bi.vel, bj.pos, bj.vel, 2.0 * POD_RADIUS, left)
                    {
                        if first.as_ref().map_or(true, |(best, _)| dt < *best) {
                            first = Some((dt, Event::Pods(i, j)));
                        }
                    }
                }
            }
            let dt = first.as_ref().map_or(left, |(dt, _)| *dt);
            for b in bodies.iter_mut() {
                b.pos = b.pos + b.vel * dt;
            }
            t += dt;
            match first {
                None => return,
                Some((_, Event::Checkpoint(i))) => {
                    bodies[i].next_cp = (bodies[i].next_cp + 1) % cps.len();
                }
                Some((_, Event::Pods(i, j))) => {
                    let (low, high) = bodies.split_at_mut(j);
                    bounce(&mut low[i], &mut high[0]);
                }
            }
        }

        let left = 1.0 - t;
        for b in bodies.iter_mut() {
            b.pos = b.pos + b.vel * left;
        }
    }

    pub fn step(game: &Game, pod: &Pod, action: &Action, first_turn: bool) -> Pod {
        let mut bodies = [steer(pod, action, first_turn)];
        play(game, &mut bodies);
        end(&bodies[0])
    }

    pub fn step_all(
        game: &Game,
        pods: &[Pod; 4],
        actions: &[Action; 4],
        first_turn: bool,
    ) -> [Pod; 4] {
        let mut bodies = [0, 1, 2, 3].map(|i| steer(&pods[i], &actions[i], first_turn));
        play(game, &mut bodies);
        bodies.map(|b| end(&b))
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

fn race(game: &Game, pod: &Pod, first_turn: bool, boost_left: bool) -> Action {
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
    let angle = angle_to(pod, target, first_turn).abs();
    let thrust = if boost_left
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

fn point_toward(pod: &Pod, angle: f64) -> Point {
    let (sin, cos) = angle.to_radians().sin_cos();
    Point {
        x: pod.pos.x + (cos * AIM_DISTANCE).round() as i32,
        y: pod.pos.y + (sin * AIM_DISTANCE).round() as i32,
    }
}

fn eval(game: &Game, pod: &Pod, action: Action, first_turn: bool) -> f64 {
    let mut p = sim::step(game, pod, &action, first_turn);
    let mut passed = (p.next_cp != pod.next_cp) as i32;
    for _ in 0..SIM_DEPTH {
        let previous = p.next_cp;
        let next_move = race(game, &p, false, false);
        p = sim::step(game, &p, &next_move, false);
        if p.next_cp != previous {
            passed += 1;
        }
    }
    let left = (game.checkpoints[p.next_cp] - p.pos).len();
    passed as f64 * CP_PASS_REWARD - left
}

fn search(game: &Game, pod: &Pod, state: &State) -> Action {
    let first_turn = state.turn == 1;
    let heuristic = race(game, pod, first_turn, !state.boost_used);

    let base = if first_turn {
        let d = game.checkpoints[pod.next_cp] - pod.pos;
        (d.y as f64).atan2(d.x as f64).to_degrees()
    } else {
        pod.angle as f64
    };
    let thrusts: Vec<Thrust> = if heuristic.thrust == Thrust::Boost {
        vec![Thrust::Boost]
    } else {
        SEARCH_THRUSTS.iter().map(|&t| Thrust::Power(t)).collect()
    };

    let mut candidates: Vec<Action> = SEARCH_ANGLES
        .iter()
        .flat_map(|&offset| {
            thrusts.iter().map(move |&thrust| Action {
                target: point_toward(pod, base + offset),
                thrust,
            })
        })
        .collect();
    candidates.push(heuristic);

    let best = candidates
        .into_iter()
        .map(|a| (a, eval(game, pod, a, first_turn)))
        .max_by(|x, y| x.1.total_cmp(&y.1));

    // debug
    if let Some((a, score)) = best {
        let h = eval(game, pod, heuristic, first_turn);
        eprintln!(
            "search: best {} ({:.0}) // heuristic {} ({:.0})",
            a, score, heuristic, h
        );
    }

    best.map(|(a, _)| a).unwrap_or(heuristic)
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

fn random_move(rng: &mut Rng) -> Move {
    let turn = match rng.next() % 4 {
        0 => -MAX_TURN,
        1 => MAX_TURN,
        _ => rng.range(-MAX_TURN, MAX_TURN),
    };
    let thrust = match rng.next() % 4 {
        0 => 0,
        1 => MAX_THRUST,
        _ => rng.range(0.0, MAX_THRUST as f64 + 1.0) as i32,
    };
    Move { turn, thrust }
}

fn random_plan(rng: &mut Rng) -> Plan {
    let mut plan = [Move::default(); PLAN_TURNS];
    for mv in plan.iter_mut() {
        *mv = random_move(rng);
    }
    plan
}

fn to_action(game: &Game, pod: &Pod, mv: Move, first_turn: bool) -> Action {
    let base = if first_turn {
        let d = game.checkpoints[pod.next_cp] - pod.pos;
        (d.y as f64).atan2(d.x as f64).to_degrees()
    } else {
        pod.angle as f64
    };
    Action {
        target: point_toward(pod, base + mv.turn),
        thrust: Thrust::Power(mv.thrust),
    }
}

fn score_plan(game: &Game, pod: &Pod, plan: &Plan, first_turn: bool) -> f64 {
    let mut p = *pod;
    let mut score = 0.0;
    for k in 0..SIM_DEPTH {
        let first = first_turn && k == 0;
        let action = if k < PLAN_TURNS {
            to_action(game, &p, plan[k], first)
        } else {
            race(game, &p, false, false)
        };
        let before = p.next_cp;
        p = sim::step(game, &p, &action, first);
        if p.next_cp != before {
            score += CP_PASS_REWARD - k as f64 * TURN_COST;
        }
    }
    score - (game.checkpoints[p.next_cp] - p.pos).len()
}

fn heuristic_plan(game: &Game, pod: &Pod, first_turn: bool) -> Plan {
    let mut plan = [Move::default(); PLAN_TURNS];
    let mut p = *pod;
    for k in 0..PLAN_TURNS {
        let first = first_turn && k == 0;
        let action = race(game, &p, first, false);
        let turn = if first {
            0.0
        } else {
            sim::angle_diff(&p, action.target).clamp(-MAX_TURN, MAX_TURN)
        };
        let thrust = match action.thrust {
            Thrust::Power(t) => t,
            _ => MAX_THRUST,
        };
        plan[k] = Move { turn, thrust };
        p = sim::step(game, &p, &to_action(game, &p, plan[k], first), first);
    }
    plan
}

fn shifted(plan: &Plan, rng: &mut Rng) -> Plan {
    let mut next = *plan;
    next.rotate_left(1);
    next[PLAN_TURNS - 1] = random_move(rng);
    next
}

fn random_search(game: &Game, pod: &Pod, state: &mut State, i: usize) -> Action {
    let start = Instant::now();
    let first_turn = state.turn == 1;
    let heuristic = race(game, pod, first_turn, !state.boost_used);
    let kept = shifted(&state.plans[i], &mut state.rng);
    if heuristic.thrust == Thrust::Boost {
        state.plans[i] = kept;
        return heuristic;
    }

    let mut best = heuristic_plan(game, pod, first_turn);
    let mut best_score = score_plan(game, pod, &best, first_turn);
    let heuristic_score = best_score;
    let mut plan = kept;
    let mut tries = 1;
    while start.elapsed().as_millis() < SEARCH_MS {
        let score = score_plan(game, pod, &plan, first_turn);
        if score > best_score {
            best = plan;
            best_score = score;
        }
        tries += 1;
        plan = random_plan(&mut state.rng);
    }

    eprintln!(
        "random search: {} plans, best {:.0} vs heuristic {:.0}",
        tries, best_score, heuristic_score
    );
    state.plans[i] = best;
    to_action(game, pod, best[0], first_turn)
}

// Actions
fn decide(game: &Game, turn: &Turn, state: &mut State) -> [Action; 2] {
    let start_time = Instant::now();
    state.turn += 1;
    check_simulation(turn, state);
    update_progress(turn, state);
    state.runner = choose_runner(state);
    let pod_runner = state.runner;
    let pod_blocker = 1 - pod_runner;

    let leader = opp_leader(game, turn, state);
    let runner = random_search(game, &turn.my_pods[pod_runner], state, pod_runner);
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
    let guess = |pod: &Pod| race(game, pod, first_turn, false);
    let all_actions = [
        actions[0],
        actions[1],
        guess(&turn.opp_pods[0]),
        guess(&turn.opp_pods[1]),
    ];
    let all = sim::step_all(game, &turn.all(), &all_actions, first_turn);
    state.simulated = Some([all[0], all[1]]);
    // debug
    eprintln!("turn took {}ms", start_time.elapsed().as_millis());
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
