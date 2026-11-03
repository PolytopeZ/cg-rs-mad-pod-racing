use std::io;

macro_rules! parse_input {
    ($x:expr, $t:ident) => {
        $x.trim().parse::<$t>().unwrap()
    };
}

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
fn main() {
    // game loop
    loop {
        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let inputs = input_line.split(" ").collect::<Vec<_>>();
        let x = parse_input!(inputs[0], i32);
        let y = parse_input!(inputs[1], i32);
        let next_checkpoint_x = parse_input!(inputs[2], i32); // x position of the next check point
        let next_checkpoint_y = parse_input!(inputs[3], i32); // y position of the next check point
        let next_checkpoint_dist = parse_input!(inputs[4], i32); // distance to the next checkpoint
        let next_checkpoint_angle = parse_input!(inputs[5], i32); // angle between your pod orientation and the direction of the next checkpoint
        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let inputs = input_line.split(" ").collect::<Vec<_>>();
        let opponent_x = parse_input!(inputs[0], i32);
        let opponent_y = parse_input!(inputs[1], i32);

        // Write an action using println!("message...");
        // To debug: eprintln!("Debug message...");

        // You have to output the target position
        // followed by the power (0 <= thrust <= 100) or "BOOST"
        // i.e.: "x y thrust"
        println!("{} {} 80", next_checkpoint_x, next_checkpoint_y);
    }
}
