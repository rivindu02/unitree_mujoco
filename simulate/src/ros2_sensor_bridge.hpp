#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <mujoco/mujoco.h>
#include <vector>
#include <cmath>
#include <algorithm>

struct Point3D { float x, y, z; };

class Ros2SensorBridge : public rclcpp::Node {
public:
    Ros2SensorBridge() : Node("sim_sensor_bridge"), ray_index_(0), is_scanning_(false) {
        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        int channels = 16;
        int rays_per_channel = num_rays_ / channels;
        for (int i = 0; i < channels; ++i) {
            float va = -30.0f * (M_PI / 180.0f) + i * (45.0f * (M_PI / 180.0f) / (channels - 1));
            for (int j = 0; j < rays_per_channel; ++j) {
                float ha = j * (2.0f * M_PI / rays_per_channel);
                float x = std::cos(va) * std::cos(ha);
                float y = std::cos(va) * std::sin(ha);
                float z = std::sin(va);
                ray_dirs_.push_back({x, y, z});
            }
        }

        RCLCPP_INFO(this->get_logger(), "Ros2SensorBridge initialized: publishing /lidar_points, /imu/data, /clock");
    }

    void process_step(mjModel* mj_model, mjData* mj_data, int step_counter) {
        if (step_counter % 10 == 0) {
            publish_clock(mj_data);
            publish_imu(mj_data);
        }
        if (step_counter % 100 == 0 && !is_scanning_) {
            start_lazy_flash(mj_model, mj_data);
        }
        if (is_scanning_) {
            process_lazy_flash_step(mj_model, mj_data);
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    int num_rays_ = 4000;
    int rays_per_step_ = 400;
    std::vector<Point3D> ray_dirs_;

    bool is_scanning_;
    int ray_index_;
    std::vector<Point3D> points_buffer_;
    double frozen_pos_[3];
    double frozen_mat_[9];
    builtin_interfaces::msg::Time frozen_time_;
    int lidar_site_id_ = -1;

    builtin_interfaces::msg::Time get_sim_time(mjData* mj_data) {
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(mj_data->time);
        t.nanosec = static_cast<uint32_t>((mj_data->time - t.sec) * 1e9);
        return t;
    }

    void publish_clock(mjData* mj_data) {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = get_sim_time(mj_data);
        clock_pub_->publish(msg);
    }

    void publish_imu(mjData* mj_data) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = get_sim_time(mj_data);
        msg.header.frame_id = "imu_link";
        msg.orientation.w = mj_data->qpos[3];
        msg.orientation.x = mj_data->qpos[4];
        msg.orientation.y = mj_data->qpos[5];
        msg.orientation.z = mj_data->qpos[6];
        imu_pub_->publish(msg);
    }

    void start_lazy_flash(mjModel* mj_model, mjData* mj_data) {
        if (lidar_site_id_ < 0) {
            lidar_site_id_ = mj_name2id(mj_model, mjOBJ_SITE, "lidar_site");
            if (lidar_site_id_ < 0) {
                RCLCPP_WARN(this->get_logger(), "lidar_site not found in model - LiDAR disabled");
                return;
            }
        }

        for(int i=0; i<3; ++i) frozen_pos_[i] = mj_data->site_xpos[3*lidar_site_id_ + i];
        for(int i=0; i<9; ++i) frozen_mat_[i] = mj_data->site_xmat[9*lidar_site_id_ + i];

        frozen_time_ = get_sim_time(mj_data);
        points_buffer_.clear();
        points_buffer_.reserve(num_rays_);
        ray_index_ = 0;
        is_scanning_ = true;
    }

    void process_lazy_flash_step(mjModel* mj_model, mjData* mj_data) {
        int end_index = std::min(ray_index_ + rays_per_step_, num_rays_);
        int geom_id[1] = {-1};

        for (int i = ray_index_; i < end_index; ++i) {
            const auto& local_dir = ray_dirs_[i];
            double world_dir[3];
            world_dir[0] = frozen_mat_[0]*local_dir.x + frozen_mat_[1]*local_dir.y + frozen_mat_[2]*local_dir.z;
            world_dir[1] = frozen_mat_[3]*local_dir.x + frozen_mat_[4]*local_dir.y + frozen_mat_[5]*local_dir.z;
            world_dir[2] = frozen_mat_[6]*local_dir.x + frozen_mat_[7]*local_dir.y + frozen_mat_[8]*local_dir.z;

            mjtNum dist = mj_ray(mj_model, mj_data, frozen_pos_, world_dir, nullptr, 1, -1, geom_id);

            if (dist > 0.15 && dist < 15.0) {
                points_buffer_.push_back({
                    static_cast<float>(local_dir.x * dist),
                    static_cast<float>(local_dir.y * dist),
                    static_cast<float>(local_dir.z * dist)
                });
            }
        }

        ray_index_ = end_index;

        if (ray_index_ >= num_rays_) {
            publish_pointcloud();
            is_scanning_ = false;
        }
    }

    void publish_pointcloud() {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = frozen_time_;
        msg.header.frame_id = "lidar_frame";

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points_buffer_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for (const auto& p : points_buffer_) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            ++iter_x; ++iter_y; ++iter_z;
        }
        pcl_pub_->publish(msg);
    }
};
