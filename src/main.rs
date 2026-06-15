use anyhow::anyhow;
use chrono::{DateTime, Local};
use clap::Parser;
use hiroz::{
    Builder, Result, context::ZContextBuilder, msg::NativeCdrSerdes, node::ZNode, pubsub::ZPub,
};
use hiroz_msgs::{
    builtin_interfaces::Time,
    geometry_msgs::{
        Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3, Wrench,
    },
    nav_msgs::Odometry,
    sensor_msgs::{Image, PointCloud2, PointField},
    std_msgs::Header,
};
use tokio::task::JoinSet;
use tracing::{debug, error, info, trace};
use tracing_subscriber::prelude::*;
use webots::{Accelerometer, Camera, Gps, Gyro, InertialUnit, Lidar, Robot, Sensor, WbLidarPoint};

async fn handle_state(node: ZNode, robot_name: &str, time_step: i32) {
    let podom = node
        .create_pub((robot_name.to_string() + "/odom").as_str())
        .build()
        .expect("Failed to create odom publisher");
    let pcam = node
        .create_pub((robot_name.to_string() + "/image").as_str())
        .build()
        .expect("Failed to create camera publisher");
    let plidar = node
        .create_pub((robot_name.to_string() + "/pointcloud").as_str())
        .build()
        .expect("Failed to create lidar publisher");
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

async fn odom_pub(
    publisher: &ZPub<Odometry, NativeCdrSerdes<Odometry>>,
    odom: &(Accelerometer, Gyro, Gps, InertialUnit),
    now: DateTime<Local>,
) -> Result<()> {
    let (_acc, gyro, gps, imu) = odom;
    let p = gps.location();
    let q = imu.quaternion()?;
    let lv = gps.speeds();
    // let la = acc.values()?;
    let av = gyro.values()?;

    let odom = Odometry {
        header: Header {
            stamp: Time {
                sec: now.timestamp() as i32,
                nanosec: now.timestamp_subsec_nanos(),
            },
            frame_id: "robot".to_string(),
        },
        child_frame_id: "odom".to_string(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                },
                orientation: Quaternion {
                    x: q[0],
                    y: q[1],
                    z: q[2],
                    w: q[3],
                },
            },
            covariance: [0.0; 36],
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: lv[0],
                    y: lv[1],
                    z: lv[2],
                },
                angular: Vector3 {
                    x: av[0],
                    y: av[1],
                    z: av[2],
                },
            },
            covariance: [0.0; 36],
        },
    };

    debug!("{odom:?}");
    if let Err(e) = publisher.async_publish(&odom).await {
        error!("Failed to publish odometry: {}", e);
    }
    Ok(())
}

async fn camera_pub(
    publisher: &ZPub<Image, NativeCdrSerdes<Image>>,
    camera: &Camera,
    now: DateTime<Local>,
) -> Result<()> {
    let image = Image {
        header: Header {
            stamp: Time {
                sec: now.timestamp() as i32,
                nanosec: now.timestamp_subsec_nanos(),
            },
            frame_id: "camera".to_string(),
        },
        height: camera.height() as u32,
        width: camera.width() as u32,
        encoding: "RGB888".to_string(),
        is_bigendian: 0,
        step: camera.width() as u32,
        data: camera.image()?.into(),
    };
    if let Err(e) = publisher.async_publish(&image).await {
        error!("Failed to publish camera image: {}", e);
    }

    Ok(())
}

async fn lidar_pub(
    publisher: &ZPub<PointCloud2, NativeCdrSerdes<PointCloud2>>,
    lidar: &Lidar,
    now: DateTime<Local>,
) -> Result<()> {
    if !lidar.is_point_cloud_enabled() {
        return Err("Lidar PointCloud is not enabled".into());
    }
    let point_step = std::mem::size_of::<WbLidarPoint>() as u32;

    let pc = PointCloud2 {
        header: Header {
            stamp: Time {
                sec: now.timestamp() as i32,
                nanosec: now.timestamp_subsec_nanos(),
            },
            frame_id: "lidar".to_string(),
        },
        height: 1,
        width: lidar.number_of_points() as u32,
        fields: vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: PointField::FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: PointField::FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: PointField::FLOAT32,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step,
        row_step: point_step * lidar.number_of_points() as u32,
        data: {
            let points = lidar.point_cloud();
            let len = points.len() * point_step as usize;
            let mut data = Vec::<u8>::with_capacity(len);
            for point in points {
                data.extend_from_slice(&point.x.to_le_bytes());
                data.extend_from_slice(&point.y.to_le_bytes());
                data.extend_from_slice(&point.z.to_le_bytes());
            }
            data
        }
        .into(),
        is_dense: false,
    };

    if let Err(e) = publisher.async_publish(&pc).await {
        error!("Failed to publish point cloud: {}", e);
    }

    Ok(())
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short = 'm', long, default_value = "peer")]
    mode: String,

    #[arg(short = 'e', long)]
    endpoint: Option<String>,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<()> {
    let args = Args::parse();
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(tracing_subscriber::EnvFilter::from_default_env())
        .init();

    let domain = std::env::var("ROS_DOMAIN_ID").unwrap_or("0".to_string());
    info!("domain id: {}", domain);

    let ctx = if let Some(e) = &args.endpoint {
        ZContextBuilder::default()
            .with_mode(&args.mode)
            .with_shm_enabled()
            .map_err(|e| anyhow!(e))?
            .with_domain_id(domain.parse::<usize>().unwrap_or(0))
            .with_connect_endpoints([e])
            .build()?
    } else {
        ZContextBuilder::default()
            .with_mode(&args.mode)
            .with_shm_enabled()?
            .with_domain_id(domain.parse::<usize>().unwrap_or(0))
            .build()?
    };
    let node = ctx.create_node("HeronC").build()?;

    Robot::init();
    let mut engine = JoinSet::<()>::new();

    let robot_name = Robot::name();

    info!("Declaring Subscriber on '{}'...", &robot_name);

    let time_step = Robot::basic_time_step() as i32;
    info!("Robot basic time step: {}", time_step);

    let twist = node
        .create_sub::<Twist>((robot_name.to_string() + "/cmd_vel").as_str())
        .build()?;
    let left = Robot::motor("left_motor");
    let right = Robot::motor("right_motor");
    engine.spawn(async move {
        while let Ok(msg) = twist.async_recv().await {
            let speed = msg.linear;
            debug!("speed {:?}", speed);
            left.set_force(speed.x.clamp(-left.max_force(), left.max_force()));
            right.set_force(speed.y.clamp(-right.max_force(), right.max_force()));
        }
    });
    let wrench = node
        .create_sub::<Wrench>((robot_name.to_string() + "/cmd_force").as_str())
        .build()?;
    let left = Robot::motor("left_motor");
    let right = Robot::motor("right_motor");
    engine.spawn(async move {
        while let Ok(msg) = wrench.async_recv().await {
            let force = msg.force;
            debug!("force {force:?}");
            left.set_force(force.x.clamp(-left.max_force(), left.max_force()));
            right.set_force(force.y.clamp(-right.max_force(), right.max_force()));
        }
    });

    let twist = node
        .create_sub::<Twist>((robot_name.to_string() + "/cmd_pos").as_str())
        .build()?;
    let left = Robot::motor("left_motor");
    let right = Robot::motor("right_motor");
    engine.spawn(async move {
        while let Ok(msg) = twist.async_recv().await {
            let pos = msg.linear;
            debug!("pos {pos:?}");
            left.set_force(pos.x.clamp(-left.max_position(), left.max_position()));
            right.set_force(pos.y.clamp(-right.max_position(), right.max_position()));
        }
    });

    engine.spawn(handle_state(node, robot_name, time_step));

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

    Ok(())
}
