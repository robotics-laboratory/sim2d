#include "simulator2d/simulator_node.h"
#include "simulator2d/status_code.h"

#include "geom/msg.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace simulator2d {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initializeParameters();
    initializeTopicHandlers();
    initializeEngine();

    timer_ = create_wall_timer(
        std::chrono::duration<double>(params_.update_period),
        std::bind(&SimulatorNode::makeSimulationTick, this));
}

void SimulatorNode::initializeParameters() {
    params_ = {
        .update_period = declare_parameter("update_period", 0.01),

        .noise_generator = {
            .lidar = {
                .enable   = declare_parameter("sensor_noise.lidar.enable", false),
                .mean     = declare_parameter("sensor_noise.lidar.mean", 0.0),
                .variance = declare_parameter("sensor_noise.lidar.variance", 0.0),
            },
            .gyro = {
                .enable   = declare_parameter("sensor_noise.gyro.enable", false),
                .mean     = declare_parameter("sensor_noise.gyro.mean", 0.0),
                .variance = declare_parameter("sensor_noise.gyro.variance", 0.0),
            },
            .accel = {
                .enable   = declare_parameter("sensor_noise.accel.enable", false),
                .mean     = declare_parameter("sensor_noise.accel.mean", 0.0),
                .variance = declare_parameter("sensor_noise.accel.variance", 0.0),
            },
        },

        .init_state = {
            .x   = declare_parameter("initial_x", 0.0),
            .y   = declare_parameter("initial_y", 0.0),
            .yaw = declare_parameter("initial_yaw", 0.0),
        },
    };
}

void SimulatorNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    // Подписчик на Control убран - это сделает адаптер (truck_msgs::Control) и вызовет setControl()

    signals_.time = Node::create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).reliability(qos));

    signals_.localization = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/simulator/localization", rclcpp::QoS(1).reliability(qos));

    signals_.hardware_odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/hardware/wheel/odometry", rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(1).reliability(qos));

    signals_.scan = Node::create_publisher<sensor_msgs::msg::LaserScan>(
        "/lidar/scan", rclcpp::QoS(1).reliability(qos));

    signals_.imu = Node::create_publisher<sensor_msgs::msg::Imu>(
        "/camera/imu", rclcpp::QoS(1).reliability(qos));

    // telemetry и state убраны 
}

void SimulatorNode::initializeCache(const std::unique_ptr<model::Model>& model) {
    cache_.lidar_config.angle_min =
        static_cast<float>(model->lidar().angle_min.radians());
    cache_.lidar_config.angle_max =
        static_cast<float>(model->lidar().angle_max.radians());
    cache_.lidar_config.angle_increment =
        static_cast<float>(model->lidar().angle_increment.radians());
    cache_.lidar_config.range_min = model->lidar().range_min;
    cache_.lidar_config.range_max = model->lidar().range_max;
}

void SimulatorNode::initializeEngine() {
    auto model = std::make_unique<model::Model>(
        model::load(get_logger(), declare_parameter("model_config", "")));
    initializeCache(model);

    auto noise_generator = std::make_unique<NoiseGenerator>(params_.noise_generator);

    const geom::Pose init_pose = {
        geom::Vec2(params_.init_state.x, params_.init_state.y),
        geom::AngleVec2(geom::Angle(params_.init_state.yaw))};

    engine_ = std::make_unique<SimulatorEngine>(
        std::move(model),
        std::move(noise_generator),
        declare_parameter("integration_step", 0.001),
        declare_parameter("calculations_precision", 1e-8));

    engine_->resetBase(init_pose);
    engine_->resetMap(declare_parameter("map_config", ""));

    // Публикуем нулевое состояние сразу после инициализации - zero state of simulation
    publishSimulationState();
}

// Точка входа управления для адаптера

void SimulatorNode::setControl(double velocity, double curvature) {
    engine_->setBaseControl(velocity, curvature);
}

void SimulatorNode::setControl(double velocity, double curvature, double acceleration) {
    engine_->setBaseControl(velocity, acceleration, curvature);
}

std::optional<tf2::Transform> SimulatorNode::getLatestTransform(
    const std::string& source, const std::string& target) {
    try {
        const auto tf_msg =
            tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        return tf;
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

// Публикация

void SimulatorNode::publishTime(const TruckState& truck_state) {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = truck_state.time();
    signals_.time->publish(clock_msg);
}

void SimulatorNode::publishSimulatorLocalizationMessage(const TruckState& truck_state) {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "world";
    msg.child_frame_id = "base";
    msg.header.stamp = truck_state.time();

    const auto pose = truck_state.odomBasePose();
    msg.pose.pose = geom::msg::toPose(pose);

    const auto linear_velocity = truck_state.odomBaseLinearVelocity();
    msg.twist.twist.linear.x = linear_velocity.x;
    msg.twist.twist.linear.y = linear_velocity.y;
    msg.twist.twist.angular.z = truck_state.baseAngularVelocity();

    signals_.localization->publish(msg);
}

void SimulatorNode::publishHardwareOdometryMessage(const TruckState& truck_state) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "base";
    odom_msg.header.stamp = truck_state.time();
    odom_msg.twist.twist.linear.x = truck_state.baseTwist().velocity;
    odom_msg.twist.covariance[0] = 0.0001;
    signals_.hardware_odometry->publish(odom_msg);
}

void SimulatorNode::publishTransformMessage(const TruckState& truck_state) {
    if (!transforms_.ekf_base.has_value()) {
        return;
    }

    const tf2::Transform& tf_ekf_base = *transforms_.ekf_base;
    const auto pose = truck_state.odomBasePose();

    tf2::Transform tf_world_base;
    tf2::fromMsg(geom::msg::toPose(pose), tf_world_base);

    const tf2::Transform tf_world_ekf = tf_world_base * tf_ekf_base.inverse();

    geometry_msgs::msg::TransformStamped tf_msg_world_ekf;
    tf_msg_world_ekf.header.frame_id = "world";
    tf_msg_world_ekf.child_frame_id = "odom_ekf";
    tf_msg_world_ekf.header.stamp = truck_state.time();
    tf2::toMsg(tf_world_ekf, tf_msg_world_ekf.transform);

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(tf_msg_world_ekf);
    signals_.tf_publisher->publish(tf_msg);
}

void SimulatorNode::publishLaserScanMessage(const TruckState& truck_state) {
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.frame_id = "lidar_link";
    scan_msg.header.stamp = truck_state.time();
    scan_msg.angle_min = cache_.lidar_config.angle_min;
    scan_msg.angle_max = cache_.lidar_config.angle_max;
    scan_msg.angle_increment = cache_.lidar_config.angle_increment;
    scan_msg.range_min = cache_.lidar_config.range_min;
    scan_msg.range_max = cache_.lidar_config.range_max;
    scan_msg.scan_time = params_.update_period;
    scan_msg.ranges = truck_state.lidarRanges();
    signals_.scan->publish(scan_msg);
}

void SimulatorNode::publishImuMessage(const TruckState& truck_state) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "camera_imu_optical_frame";
    imu_msg.header.stamp = truck_state.time();
    imu_msg.orientation_covariance[0] = -1;

    const auto angular_velocity = truck_state.gyroAngularVelocity();
    imu_msg.angular_velocity.x = angular_velocity.x;
    imu_msg.angular_velocity.y = angular_velocity.y;
    imu_msg.angular_velocity.z = angular_velocity.z;

    const auto acceleration = truck_state.accelLinearAcceleration();
    imu_msg.linear_acceleration.x = acceleration.x;
    imu_msg.linear_acceleration.y = acceleration.y;
    imu_msg.linear_acceleration.z = acceleration.z;

    signals_.imu->publish(imu_msg);
}

void SimulatorNode::publishSimulationState() {
    const auto truck_state = engine_->getTruckState();

    publishTime(truck_state);
    publishSimulatorLocalizationMessage(truck_state);
    publishHardwareOdometryMessage(truck_state);
    publishTransformMessage(truck_state);
    publishLaserScanMessage(truck_state);
    publishImuMessage(truck_state);

    // публикация truck-специфичных топиков
    onSimulationTick(truck_state);
}

void SimulatorNode::makeSimulationTick() {
    const auto tf_ekf_base = getLatestTransform("base", "odom_ekf");
    if (tf_ekf_base.has_value()) {
        transforms_.ekf_base = tf_ekf_base;
    }

    engine_->advance(params_.update_period);
    publishSimulationState();
}

}  // namespace simulator2d

/*
Убрано
все #include "truck_msgs/..."  
slots_.control и handleControl - подписчик на Control уходит в адаптер
signals_.telemetry и signals_.state - паблишеры truck_msgs убраны
publishTelemetryMessage и publishSimulationStateMessage убраны
truck:: префиксы в namespace и в geom::msg::toPose
*/