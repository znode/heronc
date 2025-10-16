use chrono::{DateTime, Local};
use interfaces::{
    geometry_msgs::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3, Wrench,
    },
    nav_msgs::Odometry,
    sensor_msgs::{Image, PointCloud2, PointField, point_field},
    std_msgs::{Header, Time},
};
use log::{debug, error, info, trace, warn};
use prost::Message;
use std::error::Error;
use tokio::task::JoinSet;
use webots::{Accelerometer, Camera, Gps, Gyro, InertialUnit, Lidar, Robot, Sensor, WbLidarPoint};
use zenoh::{Session, config::Config, pubsub::Publisher};

async fn handle_message(session: Session, topic: String) {
    let subscriber = session.declare_subscriber(topic.as_str()).await.unwrap();

    let left = Robot::motor("left_motor");
    let right = Robot::motor("right_motor");

    while let Ok(msg) = subscriber.recv_async().await {
        match msg.key_expr().as_str().split('/').next_back() {
            Some("cmd_vel") => {
                if let Ok(twist) = Twist::decode(&*msg.payload().to_bytes())
                    && let Some(speed) = twist.linear
                {
                    debug!("speed {:?}", speed);
                    left.set_position(f64::INFINITY);
                    right.set_position(f64::INFINITY);
                    left.set_velocity(speed.x.clamp(-left.max_velocity(), left.max_velocity()));
                    right.set_velocity(speed.y.clamp(-right.max_velocity(), right.max_velocity()));
                }
            }
            Some("cmd_force") => {
                if let Ok(wrench) = Wrench::decode(&*msg.payload().to_bytes())
                    && let Some(force) = wrench.force
                {
                    debug!("force {force:?}");
                    left.set_force(force.x.clamp(-left.max_force(), left.max_force()));
                    right.set_force(force.y.clamp(-right.max_force(), right.max_force()));
                }
            }
            Some("cmd_pos") => {
                if let Ok(twist) = Twist::decode(&*msg.payload().to_bytes())
                    && let Some(pos) = twist.linear
                {
                    debug!("position {pos:?}");
                    left.set_force(pos.x.clamp(-left.max_position(), left.max_position()));
                    right.set_force(pos.y.clamp(-right.max_position(), right.max_position()));
                }
            }
            _ => warn!("Unknown topic: {}", msg.key_expr()),
        }
    }
}

async fn handle_state(session: Session, robot_name: &str, time_step: i32) {
    let podom = session
        .declare_publisher(robot_name.to_string() + "/odom")
        .await
        .unwrap();
    let pcam = session
        .declare_publisher(robot_name.to_string() + "/image")
        .await
        .unwrap();
    let plidar = session
        .declare_publisher(robot_name.to_string() + "/pointcloud")
        .await
        .unwrap();
    let odom = enable_odom(20);
    let camera = enable_camera(40);
    let lidar = enable_lidar(100);

    Robot::step(time_step);

    while Robot::step_begin(time_step) != -1 {
        let now = chrono::Local::now();
        if let Err(err) = odom_pub(&podom, &odom, now).await {
            error!("Error publishing odom data: {}", err);
        }
        if let Err(err) = camera_pub(&pcam, &camera, now).await {
            error!("Error publishing camera data: {}", err);
        }
        if let Err(err) = lidar_pub(&plidar, &lidar, now).await {
            error!("Error publishing lidar data: {}", err);
        }

        Robot::step_end();
        trace!("Robot step......")
    }
    error!("Robot step failed");
}

fn enable_odom(period: i32) -> (Accelerometer, Gyro, Gps, InertialUnit) {
    let imu = Robot::inertial_unit("imu inertial_unit");
    let accelerometer = Robot::accelerometer("imu accelerometer");
    let gyro = Robot::gyro("imu gyro");
    let gps = Robot::gps("gps");
    imu.enable(period);
    accelerometer.enable(period);
    gyro.enable(period);
    gps.enable(period);

    (accelerometer, gyro, gps, imu)
}

fn enable_camera(period: i32) -> Camera {
    let camera = Robot::camera("camera rgb");
    camera.enable(period);
    camera
}

fn enable_lidar(period: i32) -> Lidar {
    let lidar = Robot::lidar("lidar");
    lidar.enable(period);
    lidar.enable_point_cloud();
    info!(
        "Lidar PointCloud enabled {}",
        lidar.is_point_cloud_enabled()
    );
    lidar
}

async fn odom_pub<'a>(
    publisher: &Publisher<'a>,
    odom: &(Accelerometer, Gyro, Gps, InertialUnit),
    now: DateTime<Local>,
) -> Result<(), Box<dyn Error>> {
    let (_acc, gyro, gps, imu) = odom;
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
    if let Err(e) = publisher.put(odom.encode_to_vec()).await {
        error!("Failed to publish odometry: {}", e);
    }
    Ok(())
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
    if let Err(e) = publisher.put(image.encode_to_vec()).await {
        error!("Failed to publish camera image: {}", e);
    }

    Ok(())
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

    if let Err(e) = publisher.put(pc.encode_to_vec()).await {
        error!("Failed to publish point cloud: {}", e);
    }

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

    let time_step = Robot::basic_time_step() as i32;
    info!("Robot basic time step: {}", time_step);
    engine.spawn(handle_message(
        session.clone(),
        robot_name.to_string() + "/cmd_vel",
    ));
    engine.spawn(handle_message(
        session.clone(),
        robot_name.to_string() + "/cmd_force",
    ));

    engine.spawn(handle_state(session.clone(), robot_name, time_step));

    while let Some(res) = engine.join_next_with_id().await {
        match res {
            Ok((id, _)) => {
                info!("Task {} completed", id);
            }
            Err(err) => {
                error!("Task failed: {}", err);
            }
        }
    }
}
