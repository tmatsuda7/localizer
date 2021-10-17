#ifndef LOCALIZER_H
#define LOCALIZER_H

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<sensor_msgs/LaserScan.h>
#include<color_detector_msgs/TargetPosition.h>
#include<localizer/STD_Status.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/OccupancyGrid.h>
#include<random>
#include<tf/transform_broadcaster.h>
#include<math.h>

class Localizer
{
    public:
        Localizer();
        void process();
    private:
        class Particle
        {
            public:
                Particle(Localizer* localizer);
                geometry_msgs::PoseStamped p_pose;
                double w;
                void set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma);
                void p_move(double dtrans, double drot1, double drot2);
            private:
                Localizer* mcl;
        };
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void positioning_callback(const color_detector_msgs::TargetPosition::ConstPtr &msg);
        void positioning_callback2(const color_detector_msgs::TargetPosition::ConstPtr &msg);
        void position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void position_callback2(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void particle_callback(const geometry_msgs::PoseArray::ConstPtr &msg);
        void particle_callback2(const geometry_msgs::PoseArray::ConstPtr &msg);
        void std_callback(const localizer::STD_Status::ConstPtr &msg);
        void std_callback2(const localizer::STD_Status::ConstPtr &msg);

        Particle make_particle();
        double gaussian(double mu, double sigma);
        double gaussian(double mu, double sigma, double x);
        void create_p_pose_array_from_p_array(std::vector<Particle> &p_array);
        double substract_yawA_from_yawB(double yawA, double yawB);
        double adjust_yaw(double yaw);
        double create_yaw_from_msg(geometry_msgs::Quaternion q);
        void motion_update();
        int xy_to_map_index(double x, double y);
        double dist_from_p_to_wall(double x_start, double y_start, double yaw, double laser_range);
        double calc_w(geometry_msgs::PoseStamped &pose);
        double calc_w_relative_positioning(geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &reference_pose, bool show_result);
        double calc_w_relative_positioning2(geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &reference_pose, bool show_result);
        void normalize_w();
        void adaptive_resampling();
        void observation_update();
        void estimate_pose();
        void expansion_reset();
        void calc_standard_deviations(geometry_msgs::PoseArray &pose_array, double *std);
        double AngleTrim(double input);
        double AngleTrim360(double input);

        int hz;
        int particle_number;
        double init_x;
        double init_y;
        double init_yaw;
        double init_x_sigma;
        double init_y_sigma;
        double init_yaw_sigma;
        double move_noise_ratio;
        double laser_noise_ratio;
        double search_range;
        int laser_step;
        double alpha_slow_th;
        double alpha_fast_th;
        double reset_x_sigma;
        double reset_y_sigma;
        double reset_yaw_sigma;
        double expansion_x_speed;
        double expansion_y_speed;
        double expansion_yaw_speed;
        double estimated_pose_w_th;
        double reset_limit;
        bool positioning_ok;
        bool positioning_ok2;
        bool position_ok;
        bool position_ok2;
        bool std_ok;
        bool std_ok2;
        double standard_deviation[3];
        double standard_deviation_tmp[3];
        bool observation_flag;
        bool observation_switch;

        double alpha = 0;
        double alpha_slow = alpha;
        double alpha_fast = alpha;
        bool map_get_ok = false;
        bool odometry_get_ok = false;
        int reset_count = 0;
        bool parent = true;
        std::string reference_name = "roomba";
        std::string reference_name2 = "roomba";


        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Subscriber laser_sub;
        ros::Subscriber relative_positioning_sub;
        ros::Subscriber relative_positioning_sub2;
        ros::Subscriber position_sub;
        ros::Subscriber position_sub2;
        ros::Subscriber std_sub;
        ros::Subscriber std_sub2;
        ros::Subscriber odometry_sub;
        ros::Publisher mcl_pose_pub;
        ros::Publisher p_pose_array_pub;
        ros::Publisher std_pub;
        ros::Time start_time;

        nav_msgs::OccupancyGrid map;
        sensor_msgs::LaserScan laser;
        color_detector_msgs::TargetPosition positioning;
        color_detector_msgs::TargetPosition positioning2;
        nav_msgs::Odometry current_odometry;
        nav_msgs::Odometry previous_odometry;
        geometry_msgs::PoseStamped mcl_pose;
        geometry_msgs::PoseStamped reference_mcl_pose;
        geometry_msgs::PoseStamped reference_mcl_pose2;
        geometry_msgs::PoseArray p_pose_array;
        geometry_msgs::PoseArray p_pose_array_tmp;
        std::vector<Particle> p_array;
        localizer::STD_Status std_status;
        localizer::STD_Status reference_std_status;
        localizer::STD_Status reference_std_status2;

};

#endif

