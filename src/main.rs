use std::{
    error::Error,
    time::{Duration, SystemTime},
};

use prost::Message;
use zenoh::{config::Config, pubsub::Publisher};

use interfaces::{
    geometry_msgs::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
    },
    nav_msgs::Odometry,
    std_msgs::{Header, Time},
};
use webots::{Accelerometer, Compass, Gps, Gyro, InertialUnit, Robot, Sensor};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    const INFINITY: f64 = 1.0 / 0.0;
    const MAX_SPEED: f64 = 30.0;
    const TIME_STEP: i32 = 64;

    // Initiate logging
    env_logger::init();

    Robot::init();

    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();

    let robot_name = Robot::name();

    println!("Declaring Subscriber on '{}'...", &robot_name);

    let subscriber = session.declare_subscriber(robot_name).await.unwrap();
    let odom_publisher = session
        .declare_publisher(robot_name.to_string() + "/odom")
        .await
        .unwrap();

    let camera_rgb = Robot::camera("camera rgb");
    camera_rgb.enable(TIME_STEP);

    let camera_depth = Robot::range_finder("camera depth");
    camera_depth.enable(TIME_STEP);

    let left_motor = Robot::motor("left_motor");
    let right_motor = Robot::motor("right_motor");
    left_motor.set_position(INFINITY);
    right_motor.set_position(INFINITY);

    left_motor.set_velocity(0.1 * MAX_SPEED);
    right_motor.set_velocity(0.1 * MAX_SPEED);

    let (accel, gyro, _compass, imu, gps) = odom_start(TIME_STEP);

    loop {
        if Robot::step(TIME_STEP) == -1 {
            break;
        }
        let now = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;

        odom_pub(&odom_publisher, &imu, &accel, &gyro, &gps, now).await?;
        if let Ok(msg) = subscriber.recv_async().await {
            match msg.key_expr() {
                _ => println!("{}", msg.key_expr()),
            }
        }
        // initialize motor speeds at 50% of MAX_SPEED.
        let left_speed = 0.5 * MAX_SPEED;
        let right_speed = 0.5 * MAX_SPEED;

        // write actuators inputs
        left_motor.set_velocity(left_speed);
        right_motor.set_velocity(right_speed);
    }

    Ok(())
}

fn odom_start(sampling_period: i32) -> (Accelerometer, Gyro, Compass, InertialUnit, Gps) {
    // Motion and Location
    let accelerometer = Robot::accelerometer("imu accelerometer");
    accelerometer.enable(sampling_period);
    let gyro = Robot::gyro("imu gyro");
    gyro.enable(sampling_period);
    let compass = Robot::compass("imu compass");
    compass.enable(sampling_period);
    let imu = Robot::inertial_unit("imu inertial_unit");
    imu.enable(sampling_period);
    let gps = Robot::gps("gps");
    gps.enable(sampling_period);
    (accelerometer, gyro, compass, imu, gps)
}

async fn odom_pub<'a>(
    publisher: &Publisher<'a>,
    imu: &InertialUnit,
    acc: &Accelerometer,
    gyro: &Gyro,
    gps: &Gps,
    now: Duration,
) -> Result<(), Box<dyn Error>> {
    let p = gps.location();
    let q = imu.quaternion()?;
    let la = acc.values()?;
    let av = gyro.values()?;

    let odom = Odometry {
        header: Some(Header {
            stamp: Some(Time {
                sec: now.as_secs(),
                nanosec: now.subsec_nanos(),
            }),
            frame_id: "robot".to_string(),
        }),
        child_frame_id: "odom".to_string(),
        pose: Some(PoseWithCovariance {
            pose: Some(Pose {
                position: Some(Point {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                }),
                orientation: Some(Quaternion {
                    x: q[0],
                    y: q[1],
                    z: q[2],
                    w: q[3],
                }),
            }),
            covariance: vec![],
        }),
        twist: Some(TwistWithCovariance {
            twist: Some(Twist {
                linear: Some(Vector3 {
                    x: la[0],
                    y: la[1],
                    z: la[2],
                }),
                angular: Some(Vector3 {
                    x: av[0],
                    y: av[1],
                    z: av[2],
                }),
            }),
            covariance: vec![],
        }),
    };
    println!("{:?}", odom);

    publisher.put(odom.encode_to_vec()).await.unwrap();
    Ok(())
}
