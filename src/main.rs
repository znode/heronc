use std::{
    error::Error,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};

use prost::Message;
use zenoh::{config::Config, pubsub::Publisher, sample::Sample};

use interfaces::{
    geometry_msgs::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
    },
    nav_msgs::Odometry,
    sensor_msgs::{Image, PointCloud2, PointField, point_field},
    std_msgs::{Header, Time},
};
use webots::{
    Accelerometer, Camera, Compass, Gps, Gyro, InertialUnit, Lidar, Motor, Robot, Sensor,
    WbLidarPoint,
};

const INFINITY: f64 = 1.0 / 0.0;
const MAX_SPEED: f64 = 30.0;
const TIME_STEP: i32 = 10;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
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

    let camera_publisher = session
        .declare_publisher(robot_name.to_string() + "/image")
        .await
        .unwrap();

    let lidar_publisher = session
        .declare_publisher(robot_name.to_string() + "/lidar")
        .await
        .unwrap();

    let left_motor = Robot::motor("left_motor");
    let right_motor = Robot::motor("right_motor");
    left_motor.set_position(INFINITY);
    right_motor.set_position(INFINITY);

    left_motor.set_velocity(0.1 * MAX_SPEED);
    right_motor.set_velocity(0.1 * MAX_SPEED);
    let left_motor = Arc::new(Mutex::new(left_motor));
    let right_motor = Arc::new(Mutex::new(right_motor));

    let (accel, gyro, _compass, imu, gps) = odom_start(20);
    let camera = camera_start(40);
    let lidar = lidar_start(100);

    loop {
        if Robot::step(TIME_STEP) == -1 {
            break;
        }
        let now = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;

        tokio::select! {
            msg = subscriber.recv_async() => {
                if let Ok(msg) = msg {
                    handle_msg(msg, (left_motor.clone(), right_motor.clone()))
                }
            }

            _ = odom_pub(&odom_publisher, &imu, &accel, &gyro, &gps, now) => {}
            _ = camera_pub(&camera_publisher, &camera, now) => {}
            _ = lidar_pub(&lidar_publisher, &lidar, now) => {}
        };
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

    publisher.put(odom.encode_to_vec()).await.unwrap();
    Ok(())
}

pub fn camera_start(sampling_period: i32) -> Camera {
    let camera_rgb = Robot::camera("camera rgb");
    camera_rgb.enable(sampling_period);

    camera_rgb
}

async fn camera_pub<'a>(
    publisher: &Publisher<'a>,
    camera: &Camera,
    now: Duration,
) -> Result<(), Box<dyn Error>> {
    let image = Image {
        header: Some(Header {
            stamp: Some(Time {
                sec: now.as_secs(),
                nanosec: now.subsec_nanos(),
            }),
            frame_id: "camera".to_string(),
        }),
        height: camera.height() as u32,
        width: camera.width() as u32,
        encoding: "RGB888".to_string(),
        is_bigendian: false,
        step: camera.width() as u32,
        data: camera.image()?.to_vec(),
    };
    publisher.put(image.encode_to_vec()).await.unwrap();

    Ok(())
}

fn lidar_start(sampling_period: i32) -> Lidar {
    let lidar = Robot::lidar("lidar");
    lidar.enable(sampling_period);
    lidar.enable_point_cloud();
    println!(
        "Lidar PointCloud enabled {}",
        lidar.is_point_cloud_enabled()
    );

    lidar
}

async fn lidar_pub<'a>(
    publisher: &Publisher<'a>,
    lidar: &Lidar,
    now: Duration,
) -> Result<(), Box<dyn Error>> {
    if !lidar.is_point_cloud_enabled() {
        return Err("Lidar PointCloud is not enabled".into());
    }
    let point_step = std::mem::size_of::<WbLidarPoint>() as u32;

    let pc = PointCloud2 {
        header: Some(Header {
            stamp: Some(Time {
                sec: now.as_secs(),
                nanosec: now.subsec_nanos(),
            }),
            frame_id: "lidar".to_string(),
        }),
        height: 1,
        width: lidar.number_of_points() as u32,
        fields: vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: point_field::DataType::Float32 as i32,
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: point_field::DataType::Float32 as i32,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: point_field::DataType::Float32 as i32,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step,
        row_step: point_step * lidar.number_of_points() as u32,
        data: unsafe { std::mem::transmute(lidar.point_cloud().to_vec()) },
        is_dense: false,
    };

    publisher.put(pc.encode_to_vec()).await.unwrap();

    Ok(())
}

fn handle_msg(msg: Sample, motor: (Arc<Mutex<Motor>>, Arc<Mutex<Motor>>)) {
    let (left, right) = motor;
    match msg.key_expr() {
        _ => println!("{}", msg.key_expr()),
    }

    // initialize motor speeds at 50% of MAX_SPEED.
    let left_speed = 0.5 * MAX_SPEED;
    let right_speed = 0.5 * MAX_SPEED;

    // write actuators inputs
    if let Ok(left) = left.try_lock() {
        left.set_velocity(left_speed);
    }
    if let Ok(right) = right.try_lock() {
        right.set_velocity(right_speed);
    }
}
