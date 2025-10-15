use std::{error::Error, time::Duration};

use chrono::{DateTime, Local};
use log::{debug, error, info, warn};
use prost::Message;
use tokio::task::JoinSet;
use zenoh::{Session, config::Config, pubsub::Publisher};

use interfaces::{
    geometry_msgs::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3, Wrench,
    },
    nav_msgs::Odometry,
    sensor_msgs::{Image, PointCloud2, PointField, point_field},
    std_msgs::{Header, Time},
};
use webots::{
    Accelerometer, Camera, Compass, Gps, Gyro, InertialUnit, Lidar, Robot, Sensor, WbLidarPoint,
};

const INFINITY: f64 = 1.0 / 0.0;

async fn handle_message(session: Session, topic: String) {
    let subscriber = session.declare_subscriber(topic.as_str()).await.unwrap();

    let left = Robot::motor("left_motor");
    let right = Robot::motor("right_motor");

    while let Ok(msg) = subscriber.recv_async().await {
        match msg.key_expr().as_str().split('/').last() {
            Some("cmd_vel") => {
                let twist = Twist::decode(&*msg.payload().to_bytes()).unwrap();
                if let Some(speed) = twist.linear {
                    debug!("Speed {:?}", speed);
                    // write actuators inputs
                    left.set_velocity(speed.x.clamp(-left.max_velocity(), left.max_velocity()));
                    right.set_velocity(speed.y.clamp(-right.max_velocity(), right.max_velocity()));
                }
            }
            Some("cmd_force") => {
                let wrench = Wrench::decode(&*msg.payload().to_bytes()).unwrap();
                if let Some(force) = wrench.force {
                    debug!("force {force:?}");
                    // write actuators inputs
                    left.set_force(force.x.clamp(-left.max_force(), left.max_force()));
                    right.set_force(force.y.clamp(-right.max_force(), right.max_force()));
                }
            }
            _ => warn!("Unknown topic: {}", msg.key_expr()),
        }
    }
}

async fn enable_odom(session: Session, period: i32, topic: String) {
    let publisher = session.declare_publisher(topic.as_str()).await.unwrap();
    let (accel, gyro, _compass, imu, gps) = odom_start(period);

    loop {
        let now = chrono::Local::now();
        if let Err(err) = odom_pub(&publisher, &imu, &accel, &gyro, &gps, now).await {
            error!("Error publishing odom data: {}", err);
        }

        tokio::time::sleep(Duration::from_millis(period as u64)).await;
    }
}

async fn enable_camera(session: Session, period: i32, topic: String) {
    let publisher = session.declare_publisher(topic.as_str()).await.unwrap();
    let camera = camera_start(period);

    loop {
        let now = chrono::Local::now();
        if let Err(err) = camera_pub(&publisher, &camera, now).await {
            error!("Error publishing camera data: {}", err);
        }

        tokio::time::sleep(Duration::from_millis(period as u64)).await;
    }
}

async fn enable_lidar(session: Session, period: i32, topic: String) {
    let publisher = session.declare_publisher(topic.as_str()).await.unwrap();
    let lidar = lidar_start(period);

    loop {
        let now = chrono::Local::now();
        if let Err(err) = lidar_pub(&publisher, &lidar, now).await {
            error!("Error publishing lidar data: {}", err);
        }

        tokio::time::sleep(Duration::from_millis(period as u64)).await;
    }
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
    _acc: &Accelerometer,
    gyro: &Gyro,
    gps: &Gps,
    now: DateTime<Local>,
) -> Result<(), Box<dyn Error>> {
    let p = gps.location();
    let q = imu.quaternion()?;
    let lv = gps.speeds();
    // let la = acc.values()?;
    let av = gyro.values()?;

    let odom = Odometry {
        header: Some(Header {
            stamp: Some(Time {
                sec: now.timestamp() as u64,
                nanosec: now.timestamp_subsec_nanos(),
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
                    x: lv[0],
                    y: lv[1],
                    z: lv[2],
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

    debug!("{odom:?}");
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
    now: DateTime<Local>,
) -> Result<(), Box<dyn Error>> {
    let image = Image {
        header: Some(Header {
            stamp: Some(Time {
                sec: now.timestamp() as u64,
                nanosec: now.timestamp_subsec_nanos(),
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
    info!(
        "Lidar PointCloud enabled {}",
        lidar.is_point_cloud_enabled()
    );

    lidar
}

async fn lidar_pub<'a>(
    publisher: &Publisher<'a>,
    lidar: &Lidar,
    now: DateTime<Local>,
) -> Result<(), Box<dyn Error>> {
    if !lidar.is_point_cloud_enabled() {
        return Err("Lidar PointCloud is not enabled".into());
    }
    let point_step = std::mem::size_of::<WbLidarPoint>() as u32;

    let pc = PointCloud2 {
        header: Some(Header {
            stamp: Some(Time {
                sec: now.timestamp() as u64,
                nanosec: now.timestamp_subsec_nanos(),
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
        data: unsafe {
            //FIXME: Use a more efficient and safety conversion method
            std::mem::transmute::<Vec<webots::WbLidarPoint>, Vec<u8>>(lidar.point_cloud().to_vec())
        },
        is_dense: false,
    };

    publisher.put(pc.encode_to_vec()).await.unwrap();

    Ok(())
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    // Initiate logging
    env_logger::init();

    Robot::init();
    let mut engine = JoinSet::<()>::new();

    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();

    let robot_name = Robot::name();

    info!("Declaring Subscriber on '{}'...", &robot_name);

    engine.spawn(handle_message(
        session.clone(),
        robot_name.to_string() + "/cmd_vel",
    ));
    engine.spawn(handle_message(
        session.clone(),
        robot_name.to_string() + "/cmd_force",
    ));

    engine.spawn(enable_odom(
        session.clone(),
        20,
        robot_name.to_string() + "/odom",
    ));

    engine.spawn(enable_camera(
        session.clone(),
        40,
        robot_name.to_string() + "/image",
    ));

    engine.spawn(enable_lidar(
        session.clone(),
        100,
        robot_name.to_string() + "/pointcloud",
    ));

    engine.spawn(async move {
        let time_step = Robot::basic_time_step();
        info!("Robot basic time step: {}", time_step);
        loop {
            if Robot::step(time_step as i32) == -1 {
                break;
            }
        }
        error!("Robot step failed");
    });

    let left_motor = Robot::motor("left_motor");
    let right_motor = Robot::motor("right_motor");
    left_motor.set_position(INFINITY);
    right_motor.set_position(INFINITY);
    left_motor.set_velocity(0.0);
    right_motor.set_velocity(0.0);

    while let Some(res) = engine.join_next_with_id().await {
        match res {
            Ok((id, _)) => {
                info!("Task {id} completed");
            }
            Err(err) => error!("Error joining task: {}", err),
        }
    }
}
