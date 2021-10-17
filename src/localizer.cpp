#include<localizer/localizer.h>
std::random_device seed;
std::mt19937 engine(seed());
std::default_random_engine engine2(seed());

Localizer::Localizer():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("particle_number", particle_number);
    private_nh.getParam("init_x", init_x);
    private_nh.getParam("init_y", init_y);
    private_nh.getParam("init_yaw", init_yaw);
    private_nh.getParam("init_x_sigma", init_x_sigma);
    private_nh.getParam("init_y_sigma", init_y_sigma);
    private_nh.getParam("init_yaw_sigma", init_yaw_sigma);
    private_nh.getParam("move_noise_ratio", move_noise_ratio);
    private_nh.getParam("search_range", search_range);
    private_nh.getParam("laser_noise_ratio", laser_noise_ratio);
    private_nh.getParam("laser_step", laser_step);
    private_nh.getParam("alpha_slow_th", alpha_slow_th);
    private_nh.getParam("alpha_fast_th", alpha_fast_th);
    private_nh.getParam("reset_x_sigma", reset_x_sigma);
    private_nh.getParam("reset_y_sigma", reset_y_sigma);
    private_nh.getParam("reset_yaw_sigma", reset_yaw_sigma);
    private_nh.getParam("expansion_x_speed", expansion_x_speed);
    private_nh.getParam("expansion_y_speed", expansion_y_speed);
    private_nh.getParam("expansion_yaw_speed", expansion_yaw_speed);
    private_nh.getParam("estimated_pose_w_th", estimated_pose_w_th);
    private_nh.getParam("reset_limit", reset_limit);
    private_nh.getParam("reference_name", reference_name);
    private_nh.getParam("reference_name2", reference_name2);
    private_nh.getParam("parent", parent);
    private_nh.getParam("observation_flag", observation_flag);
    observation_switch = true;
    
    positioning_ok = false;
    positioning_ok2 = false;
    position_ok = false;
    position_ok2 = false;
    std_ok = false;
    std_ok2 = false;

    map_sub = nh.subscribe("/map", 1, &Localizer::map_callback, this);
    laser_sub = nh.subscribe("/scan", 1, &Localizer::laser_callback, this);
    odometry_sub = nh.subscribe("/roomba/odometry", 1, &Localizer::odometry_callback, this);
    relative_positioning_sub = nh.subscribe("/target/position", 1, &Localizer::positioning_callback, this);
    relative_positioning_sub2 = nh.subscribe(reference_name2 +"/target/position", 1, &Localizer::positioning_callback2, this);
    position_sub = nh.subscribe(reference_name +"/mcl_pose", 1, &Localizer::position_callback, this);
    position_sub2 = nh.subscribe(reference_name2 +"/mcl_pose", 1, &Localizer::position_callback2, this);
    std_sub = nh.subscribe(reference_name +"/std_status", 1, &Localizer::std_callback, this);
    std_sub2 = nh.subscribe(reference_name2 +"/std_status", 1, &Localizer::std_callback2, this);

    mcl_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mcl_pose", 100);
    p_pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/p_pose_array", 100);
    std_pub = nh.advertise<localizer::STD_Status>("/std_status", 100);

    mcl_pose.pose.position.x = 0.0;
    mcl_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), mcl_pose.pose.orientation);
    mcl_pose.header.frame_id = "map";
    p_pose_array.header.frame_id = "map";
    p_array.reserve(particle_number);
    p_pose_array.poses.reserve(particle_number);
    p_pose_array_tmp.header.frame_id = "map";
    p_pose_array_tmp.poses.reserve(particle_number);
    
    ROS_INFO_STREAM("Init");
}

void Localizer::positioning_callback(const color_detector_msgs::TargetPosition::ConstPtr &msg)
{
    positioning = *msg;
    positioning_ok = true;
    
    if(map_get_ok && observation_switch && !parent){
        observation_update();
    }

}

void Localizer::positioning_callback2(const color_detector_msgs::TargetPosition::ConstPtr &msg)
{
    positioning2 = *msg;
    positioning_ok2 = true;
    
    if(map_get_ok && observation_switch && !parent){
        observation_update();
    }

}

void Localizer::position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    reference_mcl_pose = *msg;
    position_ok = true;

}

void Localizer::position_callback2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    reference_mcl_pose2 = *msg;
    position_ok2 = true;

}

void Localizer::std_callback(const localizer::STD_Status::ConstPtr &msg)
{
    reference_std_status = *msg;
    std_ok = true;

}

void Localizer::std_callback2(const localizer::STD_Status::ConstPtr &msg)
{
    reference_std_status2 = *msg;
    std_ok2 = true;

}



void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser = *msg;
    if(map_get_ok && observation_switch && parent){
        observation_update();
    }

}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    map_get_ok = true;
    for(int i=0; i<particle_number; ++i){
        Particle p = make_particle();
        p_array.push_back(p);
    }
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(map_get_ok){
        previous_odometry = current_odometry;
        current_odometry = *msg;
        if(!odometry_get_ok){previous_odometry = current_odometry;}
        motion_update();
        odometry_get_ok = true;
    }
}

Localizer::Particle Localizer::make_particle()
{
    Particle p(this);
    return p;
}

double Localizer::gaussian(double mu, double sigma)
{
    std::normal_distribution<> dist(mu, sigma);
    return dist(engine);
}

double Localizer::gaussian(double mu, double sigma, double x)
{
    double ans = exp(- std::pow((mu - x), 2) / std::pow(sigma, 2) / 2.0) / sqrt(2.0 * M_PI * std::pow(sigma, 2));
    return ans;
}

void Localizer::create_p_pose_array_from_p_array(std::vector<Particle> &p_array)
{
    p_pose_array.poses.clear();
    p_pose_array.header.frame_id = "map";
    for(auto& p:p_array){
        p_pose_array.poses.push_back(p.p_pose.pose);
    }
}

void Localizer::motion_update()
{
    double dx = current_odometry.pose.pose.position.x - previous_odometry.pose.pose.position.x;
    double dy = current_odometry.pose.pose.position.y - previous_odometry.pose.pose.position.y;
    double current_yaw = create_yaw_from_msg(current_odometry.pose.pose.orientation);
    double previous_yaw = create_yaw_from_msg(previous_odometry.pose.pose.orientation);
    double dyaw = substract_yawA_from_yawB(previous_yaw, current_yaw);
    double dtrans = sqrt(dx*dx + dy*dy);
    double drot1 = adjust_yaw(atan2(dy, dx) - previous_yaw);
    double drot2 = adjust_yaw(dyaw - drot1);
    static double previous_drot1;
    static double previous_drot2;
    
    
    if(abs(adjust_yaw(atan2(dy, dx) - previous_yaw)*180.0/M_PI) > 10.0){
       drot1 = previous_drot1;
       drot2 = previous_drot2;
    }else{
       previous_drot1 = drot1;
       previous_drot2 = drot2;    
    }

    for(auto& p:p_array){
        p.p_move(dtrans, drot1, drot2);
    }
}

int Localizer::xy_to_map_index(double x, double y)
{
    int index_x = int((x - map.info.origin.position.x) / map.info.resolution);
    int index_y = int((y - map.info.origin.position.y) / map.info.resolution);
    return index_x + index_y * map.info.width;
}


double Localizer::dist_from_p_to_wall(double x_start, double y_start, double yaw, double laser_range)
{
    double search_step = map.info.resolution;
    double search_limit = std::min(laser_range * (1.0 + laser_noise_ratio * 5), search_range);
    for(double dist_from_start = search_step; dist_from_start <= search_limit; dist_from_start += search_step){
        double x_now = x_start + dist_from_start * cos(yaw);
        double y_now = y_start + dist_from_start * sin(yaw);
        int map_index = xy_to_map_index(x_now, y_now);
        if(map.data[map_index] == 100){
            return dist_from_start;
        }
        if(map.data[map_index] == -1){
            return search_range * 2;
        }
    }
    return search_limit;
}

void Localizer::normalize_w()
{
    alpha = 0;
    for(auto& p:p_array){
        alpha += p.w;
    }
    for(auto& p:p_array){
        p.w /= alpha;
    }
}

void Localizer::adaptive_resampling()
{
    std::uniform_real_distribution<> random(0.0, 1.0);

    double r = random(engine2);
    int index = 0;
    double reset_ratio = 1 - (alpha_fast / alpha_slow);
    std::vector<Particle> p_array_after_resampling;
    p_array_after_resampling.reserve(p_array.size());
    for(int i=0, size=p_array.size(); i<size; ++i){
        r += 1.0 / particle_number;
        while(r > p_array[index].w){
            r -= p_array[index].w;
            index = (index + 1) % particle_number;
        }
        if(random(engine2) > reset_ratio){
            Particle p = p_array[index];
            p.w = 1.0 / particle_number;
            p_array_after_resampling.push_back(p);
        }
        else{
            Particle p = p_array[index];
            p.w = 1.0 / particle_number;

            double x = mcl_pose.pose.position.x;
            double y = mcl_pose.pose.position.y;
            double yaw = create_yaw_from_msg(mcl_pose.pose.orientation);

            p.set_p(x, y, yaw, reset_x_sigma, reset_y_sigma, reset_yaw_sigma);

            p_array_after_resampling.push_back(p);

        }
    }
    p_array = p_array_after_resampling;
}

void Localizer::expansion_reset()
{
    for(auto& p:p_array){
        double x = p.p_pose.pose.position.x;
        double y = p.p_pose.pose.position.y;
        double yaw = create_yaw_from_msg(p.p_pose.pose.orientation);

        p.set_p(x, y, yaw, expansion_x_speed, expansion_y_speed, expansion_yaw_speed);
    }
}

void Localizer::observation_update()
{
    double weight = 1.0;
    double estimated_pose_w = 1.0;
    std_status.self_observation[0] = -1;
    std_status.self_observation[1] = -1;
    std_status.other_observation[0] = -1;
    std_status.other_observation[1] = -1;

    for(auto& p:p_array){
           if(parent)
              weight = calc_w(p.p_pose);

           if(!parent && positioning_ok && position_ok && std_ok && odometry_get_ok){
              if(sqrt(pow(std_status.standard_deviation[0], 2.0) + pow(std_status.standard_deviation[1], 2.0)) > sqrt(pow(reference_std_status.standard_deviation[0], 2.0) + pow(reference_std_status.standard_deviation[1], 2.0))){
                  weight = calc_w_relative_positioning(p.p_pose, reference_mcl_pose, false);
                  if(reference_name == "/roomba0"){
                      std_status.self_observation[0] = 1;
                      std_status.self_observation[1] = 1;
                  }
                  if(reference_name == "/roomba1"){
                      std_status.self_observation[0] = 1;
                      std_status.self_observation[1] = 1;
                  }
                  if(reference_name == "/roomba2"){
                      std_status.self_observation[0] = 1;
                      std_status.self_observation[1] = 1;
                  }
                  if(reference_name == "/roomba3"){
                      std_status.self_observation[0] = 1;
                      std_status.self_observation[1] = 1;
                  }
                  if(reference_name == "/roomba4"){
                      std_status.self_observation[0] = 1;
                      std_status.self_observation[1] = 1;
                  }
                  if(reference_name == "/roomba5"){
                      std_status.self_observation[0] = 1;
                      std_status.self_observation[1] = 1;
                  }              
              }
           }
           if(!parent && positioning_ok2 && position_ok2 && std_ok2 && odometry_get_ok){
              if(sqrt(pow(std_status.standard_deviation[0], 2.0) + pow(std_status.standard_deviation[1], 2.0)) > sqrt(pow(reference_std_status2.standard_deviation[0], 2.0) + pow(reference_std_status2.standard_deviation[1], 2.0))){
                  weight = calc_w_relative_positioning2(p.p_pose, reference_mcl_pose2, false);
                  if(reference_name2 == "/roomba2"){
                      std_status.other_observation[0] = 2;
                      std_status.other_observation[1] = 2;
                  }
                  if(reference_name2 == "/roomba3"){
                      std_status.other_observation[0] = 2;
                      std_status.other_observation[1] = 2;
                  }
                  if(reference_name2 == "/roomba4"){
                      std_status.other_observation[0] = 2;
                      std_status.other_observation[1] = 2;
                  }
                  if(reference_name2 == "/roomba5"){
                      std_status.other_observation[0] = 2;
                      std_status.other_observation[1] = 2;
                  }
                  if(reference_name2 == "/roomba6"){
                      std_status.other_observation[0] = 2;
                      std_status.other_observation[1] = 2;
                  }
                  if(reference_name2 == "/roomba7"){
                      std_status.other_observation[0] = 2;
                      std_status.other_observation[1] = 2;
                  }
              }
           }
        p.w = weight;
    }
    estimate_pose();
    
    if(parent)
       estimated_pose_w = calc_w(mcl_pose) / (laser.ranges.size() / laser_step);
       if(!parent && positioning_ok && position_ok && std_ok && odometry_get_ok){
          if(sqrt(pow(std_status.standard_deviation[0], 2.0) + pow(std_status.standard_deviation[1], 2.0)) > sqrt(pow(reference_std_status.standard_deviation[0], 2.0) + pow(reference_std_status.standard_deviation[1], 2.0))){
             estimated_pose_w = calc_w_relative_positioning(mcl_pose, reference_mcl_pose, true);
          }
          positioning_ok = false;
          position_ok = false;
          std_ok = false;
       }
       if(!parent && positioning_ok2 && position_ok2 && std_ok2 && odometry_get_ok){
          if(sqrt(pow(std_status.standard_deviation[0], 2.0) + pow(std_status.standard_deviation[1], 2.0)) > sqrt(pow(reference_std_status2.standard_deviation[0], 2.0) + pow(reference_std_status2.standard_deviation[1], 2.0))){
             estimated_pose_w = calc_w_relative_positioning2(mcl_pose, reference_mcl_pose2, true);
          }
          positioning_ok2 = false;
          position_ok2 = false;
          std_ok2 = false;
       }
    if(alpha_slow == 0){
        alpha_slow = alpha;
    }
    else{
        alpha_slow += alpha_slow_th * (alpha - alpha_slow);
    }
    if(alpha_fast == 0){
        alpha_fast = alpha;
    }
    else{
        alpha_fast += alpha_fast_th * (alpha - alpha_fast);
    }

    if(estimated_pose_w > estimated_pose_w_th || reset_count > reset_limit){
        reset_count = 0;
        adaptive_resampling();
    }
    else{
        reset_count += 1;
        expansion_reset();
    }

}

void Localizer::estimate_pose()
{
    normalize_w();
    double x = 0;
    double y = 0;
    double yaw = 0;
    double w_max = 0;
    for(auto& p:p_array){
        x += p.p_pose.pose.position.x * p.w;
        y += p.p_pose.pose.position.y * p.w;
        if(p.w > w_max){
            w_max = p.w;
            yaw = create_yaw_from_msg(p.p_pose.pose.orientation);
        }
    }

    mcl_pose.pose.position.x = x;
    mcl_pose.pose.position.y = y;
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), mcl_pose.pose.orientation);
}
void Localizer::process()
{
    //TF Broadcasterの実体化
    tf::TransformBroadcaster odom_state_broadcaster;

    ros::Rate rate(10);
    p_pose_array_pub.publish(p_pose_array);
    mcl_pose_pub.publish(mcl_pose);
    bool init_flag = false;
    while(ros::ok()){
        if(!init_flag && (ros::Time::now()).toSec() != 0 && odometry_get_ok){
            init_flag = true;
            start_time = ros::Time::now();
        }

        if(map_get_ok && odometry_get_ok){
            try{
                //map座標で見たbase_linkの位置の取得
                double map_to_base_x = mcl_pose.pose.position.x;
                double map_to_base_y = mcl_pose.pose.position.y;
                double map_to_base_yaw = create_yaw_from_msg(mcl_pose.pose.orientation);
                //odom座標で見たbase_linkの位置の取得
                double odom_to_base_x = current_odometry.pose.pose.position.x;
                double odom_to_base_y = current_odometry.pose.pose.position.y;
                double odom_to_base_yaw = create_yaw_from_msg(current_odometry.pose.pose.orientation);
                //map座標で見たodom座標の位置の取得
                double map_to_odom_yaw = substract_yawA_from_yawB(odom_to_base_yaw, map_to_base_yaw);
                double map_to_odom_x = map_to_base_x - odom_to_base_x * cos(map_to_odom_yaw) + odom_to_base_y * sin(map_to_odom_yaw);
                double map_to_odom_y = map_to_base_y - odom_to_base_x * sin(map_to_odom_yaw) - odom_to_base_y * cos(map_to_odom_yaw);
                geometry_msgs::Quaternion map_to_odom_quat;
                quaternionTFToMsg(tf::createQuaternionFromYaw(map_to_odom_yaw), map_to_odom_quat);

                //odom座標系の元となるodomの位置姿勢情報格納用変数の作成
                geometry_msgs::TransformStamped odom_state;
                //現在時刻の格納
                odom_state.header.stamp = ros::Time::now();
                //座標系mapとodomの指定
                odom_state.header.frame_id = "map";
                odom_state.child_frame_id = "odom";
                //map座標系からみたodom座標系の原点位置と方向の格納
                odom_state.transform.translation.x = map_to_odom_x;
                odom_state.transform.translation.y = map_to_odom_y;
                odom_state.transform.rotation = map_to_odom_quat;
            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
            }
            create_p_pose_array_from_p_array(p_array);
            p_pose_array_pub.publish(p_pose_array);
            estimate_pose();
            mcl_pose_pub.publish(mcl_pose);
            calc_standard_deviations(p_pose_array, standard_deviation);

            std_status.header.stamp = ros::Time::now();
            std_status.standard_deviation[0] = standard_deviation[0];
            std_status.standard_deviation[1] = standard_deviation[1];
            std_status.standard_deviation[2] = standard_deviation[2];
            std_status.position[0] = mcl_pose.pose.position.x;
            std_status.position[1] = mcl_pose.pose.position.y;
            std_status.position[2] = create_yaw_from_msg(mcl_pose.pose.orientation)*180.0/M_PI;
            std_pub.publish(std_status);
            std_status.self_observation[0] = -1;
            std_status.self_observation[1] = -1;
            std_status.other_observation[0] = -1;
            std_status.other_observation[1] = -1;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
Localizer::Particle::Particle(Localizer* localizer)
{
    mcl = localizer;
    p_pose.header.frame_id = "map";
    set_p(mcl->init_x, mcl->init_y, mcl->init_yaw, mcl->init_x_sigma, mcl->init_y_sigma, mcl->init_yaw_sigma);
    w = 1.0 / mcl->particle_number;
}

void Localizer::Particle::set_p(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
    p_pose.pose.position.x = mcl->gaussian(x, x_sigma);
    p_pose.pose.position.y = mcl->gaussian(y, y_sigma);
    yaw = mcl->adjust_yaw(mcl->gaussian(yaw, yaw_sigma));
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw),p_pose.pose.orientation);
}

double Localizer::create_yaw_from_msg(geometry_msgs::Quaternion q)
{
    double roll, pitch, yaw;
    tf::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
}

double Localizer::substract_yawA_from_yawB(double yawA, double yawB)
{
    double dyaw = yawB - yawA;
    dyaw = adjust_yaw(dyaw);

    return dyaw;
}

void Localizer::Particle::p_move(double dtrans, double drot1, double drot2)
{
    dtrans += mcl->gaussian(0.0, dtrans * mcl->move_noise_ratio);
    drot1 += mcl->gaussian(0.0, drot1 * mcl->move_noise_ratio);
    drot2 += mcl->gaussian(0.0, drot2 * mcl->move_noise_ratio);

    double yaw = mcl->create_yaw_from_msg(p_pose.pose.orientation);
    p_pose.pose.position.x += dtrans * cos(mcl->adjust_yaw(yaw + drot1));
    p_pose.pose.position.y += dtrans * sin(mcl->adjust_yaw(yaw + drot1));
    quaternionTFToMsg(tf::createQuaternionFromYaw(mcl->adjust_yaw(yaw + drot1 + drot2)), p_pose.pose.orientation);

}

double Localizer::calc_w(geometry_msgs::PoseStamped &pose)
{
    double weight = 1.0;
    double angle_increment =laser.angle_increment;
    double angle_min = laser.angle_min;
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = create_yaw_from_msg(pose.pose.orientation);
    if(parent){
        for(int i=0, size=laser.ranges.size(); i<size; i+=laser_step){
            if(laser.ranges[i] > 0.3 && laser.ranges[i] < 10.0){
                double angle = i * angle_increment + angle_min;
                double dist_to_wall = dist_from_p_to_wall(x, y, yaw + angle, laser.ranges[i]);
                if(abs(laser.ranges[i]-dist_to_wall) < 3*laser.ranges[i] * laser_noise_ratio){
                    weight += exp(pow(3.0,2.0)/2-pow(abs(laser.ranges[i]-dist_to_wall),2.0)/(2*pow(laser.ranges[i] * laser_noise_ratio,2.0)));
                }
            }
        }
    }
    return weight;
}

//calculating weight by positioning measurements from itself to the other robot
double Localizer::calc_w_relative_positioning(geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &reference_pose, bool show_result)
{

    double weight, x, y, yaw;
    double reference_x, reference_y, reference_yaw;
    double range_measurement, estimated_range, range_standard_deviation;
    double angle_measurement, estimated_angle, angle_standard_deviation;
    double angle_deviation;
    
    weight = 1.0;
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    yaw = create_yaw_from_msg(pose.pose.orientation)*180.0/M_PI;
    
    reference_x = reference_pose.pose.position.x;
    reference_y = reference_pose.pose.position.y;
    reference_yaw = create_yaw_from_msg(reference_pose.pose.orientation)*180.0/M_PI;
    
    range_measurement = sqrt(pow(positioning.x,2.0) + pow(positioning.z,2.0)) + 0.15;
    estimated_range = sqrt(pow(reference_x-x,2.0) + pow(reference_y-y,2.0));
    range_standard_deviation = 0.6;
    angle_standard_deviation = 10.0;

    angle_measurement = -atan2(positioning.x,positioning.z+0.15)*180.0/M_PI;
    estimated_angle = atan2((reference_y-y),(reference_x-x))*180.0/M_PI-yaw;
    
    if(estimated_angle > 180.0){
       estimated_angle = estimated_angle - 360.0;
    }else if (estimated_angle < -180.0){
       estimated_angle = estimated_angle + 360.0;
    }
        
    angle_deviation = angle_measurement - estimated_angle;
    if(angle_deviation > 180.0){
       angle_deviation = angle_deviation - 360.0;
    }else if (estimated_angle < -180.0){
       angle_deviation = angle_deviation + 360.0;
    }        

    if(abs(angle_deviation) < angle_standard_deviation * 3 && abs(range_measurement - estimated_range) < range_standard_deviation * 3){
       weight = weight * exp(pow(3.0,2.0)/2-pow(abs(range_measurement - estimated_range),2.0)/(2*pow(range_standard_deviation,2.0)));
       weight = weight * exp(pow(3.0,2.0)/2-pow(angle_deviation,2.0)/(2*pow(angle_standard_deviation,2.0)));
    }
    return weight;
}

//calculating weight by positioning measurements from the other robot to itself
double Localizer::calc_w_relative_positioning2(geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &reference_pose, bool show_result)
{
    double weight, x, y, yaw;
    double reference_x, reference_y, reference_yaw;
    double range_measurement, estimated_range, range_standard_deviation;
    double angle_measurement, estimated_angle, angle_standard_deviation;
    double angle_deviation;

    weight = 1.0;
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    yaw = create_yaw_from_msg(pose.pose.orientation)*180.0/M_PI;  
    
    reference_x = reference_pose.pose.position.x;
    reference_y = reference_pose.pose.position.y;
    reference_yaw = create_yaw_from_msg(reference_pose.pose.orientation)*180.0/M_PI;

    range_measurement = sqrt(pow(positioning2.x,2.0) + pow(positioning2.z,2.0)) + 0.15;
    estimated_range = sqrt(pow(reference_x-x,2.0) + pow(reference_y-y,2.0));
    range_standard_deviation = 0.6;
    angle_standard_deviation = 10.0;

    angle_measurement = -atan2(positioning2.x,positioning2.z+0.15)*180.0/M_PI;
    estimated_angle = atan2((y-reference_y),(x-reference_x))*180.0/M_PI-reference_yaw;
    
    if(estimated_angle > 180.0){
       estimated_angle = estimated_angle - 360.0;
    }else if (estimated_angle < -180.0){
       estimated_angle = estimated_angle + 360.0;
    }
        
    angle_deviation = angle_measurement - estimated_angle;
    if(angle_deviation > 180.0){
       angle_deviation = angle_deviation - 360.0;
    }else if (estimated_angle < -180.0){
       angle_deviation = angle_deviation + 360.0;
    }        

    if(abs(angle_deviation) < angle_standard_deviation * 3 && abs(range_measurement - estimated_range) < range_standard_deviation * 3){
       weight = weight * exp(pow(3.0,2.0)/2-pow(abs(range_measurement - estimated_range),2.0)/(2*pow(range_standard_deviation,2.0)));
       weight = weight * exp(pow(3.0,2.0)/2-pow(angle_deviation,2.0)/(2*pow(angle_standard_deviation,2.0)));
    }
    return weight;
}

double  Localizer::adjust_yaw(double yaw)
{
    while(yaw > M_PI){yaw -= 2*M_PI;}
    while(yaw < -M_PI){yaw += 2*M_PI;}

    return yaw;
}

void  Localizer::calc_standard_deviations(geometry_msgs::PoseArray &pose_array, double *std)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_yaw = 0.0;
    double sum_dx = 0.0;
    double sum_dy = 0.0;
    double sum_dyaw = 0.0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_yaw = 0.0;
    double std_x = 0.0;
    double std_y = 0.0;
    double std_yaw = 0.0;
    for(int i=0; i<particle_number; ++i){
        sum_x += pose_array.poses[i].position.x;
        sum_y += pose_array.poses[i].position.y;
        sum_yaw += AngleTrim(create_yaw_from_msg(pose_array.poses[i].orientation)*180.0/M_PI - create_yaw_from_msg(pose_array.poses[0].orientation)*180.0/M_PI);
    }
    mean_x = sum_x/particle_number;
    mean_y = sum_y/particle_number;
    mean_yaw = AngleTrim(sum_yaw/particle_number+create_yaw_from_msg(pose_array.poses[0].orientation)*180.0/M_PI);
    for(int i=0; i<particle_number; ++i){
        sum_dx = pose_array.poses[i].position.x - mean_x;
        std_x += pow(sum_dx, 2.0);
        sum_dy = pose_array.poses[i].position.y - mean_y;
        std_y += pow(sum_dy, 2.0);
        sum_dyaw = AngleTrim(create_yaw_from_msg(pose_array.poses[i].orientation)*180.0/M_PI - mean_yaw);
        std_yaw += pow(sum_dyaw, 2.0);
    }
    std_x = sqrt(std_x/particle_number);
    std_y = sqrt(std_y/particle_number);
    std_yaw = sqrt(std_yaw/particle_number);
    std[0] = std_x;
    std[1] = std_y;
    std[2] = std_yaw;
}

double  Localizer::AngleTrim(double input){
    double output = input;
    while(abs(output) > 180.0){
        if(output > 0.0)  output = output - 360.0;
        else  output = output + 360.0;
    }

    return output;
}

double  Localizer::AngleTrim360(double input){
    double output = input;

    while(1){
        if(output > 360.0)  output-= 360.0;
        else if(output < 0.0) output += 360.0;
        else return output;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Localizer");
    Localizer localizer;
    localizer.process();
    return 0;
}

