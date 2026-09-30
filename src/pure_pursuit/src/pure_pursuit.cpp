#include "pure_pursuit/pure_pursuit.hpp"
#include "pure_pursuit/dynamic_window_pure_pursuit_functions.hpp"

PurePursuit::PurePursuit()
{
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    timer_ = nh_.createTimer(ros::Duration(0.05), &PurePursuit::timerCallback, this);
    pose_sub_ = nh_.subscribe("turtle1/pose", 10, &PurePursuit::poseCallback, this);    
    
    path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_marker", 1);
    path_pub_L_ = nh_.advertise<nav_msgs::Path>("/path", 1);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal",10 , &PurePursuit::goalCallback, this);

    target_pub_ = nh_.advertise<visualization_msgs::Marker>("target_marker", 1);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);

    
    x_=0;
    y_=0;
    theta_=0;
    v_ = 0.05;

    generatePath_LC();

    // f_ = boost::bind(&PurePursuit::reconfigureCallback, this, _1, _2);
    // server_.setCallback(f_);
}

void PurePursuit::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    x_ = msg->x;
    y_ = msg->y;
    theta_ = msg->theta;

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.orientation.w = 1.0;

    trajectory_.poses.push_back(pose);

    double keep_time = 15.0; // 최근 몇 초만 유지

    while (!trajectory_.poses.empty())
    {
        if ((ros::Time::now() - trajectory_.poses.front().header.stamp).toSec() > keep_time)
        {
            trajectory_.poses.erase(trajectory_.poses.begin());
        }
        else
        {
            break;
        }
    }
    trajectory_.header.stamp = ros::Time::now();
    trajectory_.header.frame_id = "map";

    trajectory_pub_.publish(trajectory_);

    // ROS_INFO("x=%.2f y=%.2f theta=%.2f", x_, y_, theta_);
}

void PurePursuit::timerCallback(const ros::TimerEvent& event)
{
    publishPath();
    control_rpp();
    publishTurtleTF();
    ROS_INFO("x=%.2f y=%.2f theta=%.2f v=%.2f Ld=%.2f", x_, y_, theta_, v_, Ld_);
}

void PurePursuit::control_pp()
{
    if (path_.empty())
    {
        return;
    }
    
    v_ = 1.5; // 전진

    Ld_ = 1.0; // Lookahead distance 전방 주시 거리

    int path_size = path_.size();

    int window_size = 50; // 현위치에서 앞으로 20개 점만 탐색
    int nearest_idx = prev_nearest_idx_; //초깃값=0
    double min_dist = std::numeric_limits<double>::max(); // ㅈㄴ큰값으로초기화
    
    // 경로에서 가장 가까운 곳 찾는 것,
    for (int i=0; i<window_size; ++i) 
    {
        int curr_idx = (prev_nearest_idx_ + i) % path_size; 
        
        double dist = std::hypot(path_[curr_idx].x - x_, path_[curr_idx].y - y_);

        if (dist < min_dist)
        {
            min_dist = dist; // 가장 짧은 거리 남음
            nearest_idx = curr_idx; // 가장 가까운 점 번호 남음
        }
    }   

    prev_nearest_idx_ = nearest_idx; //  인덱스업데이트

    geometry_msgs::Point target_point = path_[nearest_idx];
    double actual_dist = min_dist;

    // 가까운 점에서 앞으로 하나씩 검사, Ld_ 보다 큰 거 나오면 거기가 타겟
    for (int i=0; i<window_size; ++i)
    {
        int curr_idx = (nearest_idx+i) % path_size;
        double dist = std::hypot(path_[curr_idx].x - x_, path_[curr_idx].y - y_);

        if (dist >= Ld_)
        {
            target_point = path_[curr_idx];
            actual_dist = dist;
            break;
        }
    }

    double dx = target_point.x - x_;
    double dy = target_point.y - y_;

    double local_x =  dx * cos(theta_) + dy * sin(theta_);
    double local_y = -dx * sin(theta_) + dy * cos(theta_);

    geometry_msgs::Twist cmd;
    
    double curvature = (2.0 * local_y) / (actual_dist * actual_dist); // (Ld_ * Ld_)

    cmd.linear.x = v_;
    cmd.angular.z = v_ * curvature; // 각속도=선속도x곡률

    publishTargetPoint(target_point);
    cmd_pub_.publish(cmd);
     
}


void PurePursuit::control_s()
{
    if (path_.empty())
    {
        return;
    }

    v_ = 1.5; // 전진 속도
    int path_size = path_.size();

    // Stanley 제어기는 차량의 Front Axle(전방 축) 기준으로 계산할 때 훨씬 안정적입니다.
    double d_front = 0.2; // 차량 중심에서 앞쪽으로의 거리
    double fx = x_ + d_front * std::cos(theta_);
    double fy = y_ + d_front * std::sin(theta_);

    // 1. Front Axle 기준 가장 가까운 경로 점 탐색 (Window Search)
    int window_size = 50; 
    int nearest_idx = prev_nearest_idx_; 
    double min_dist = std::numeric_limits<double>::max(); 

    for (int i = 0; i < window_size; ++i) 
    {
        int curr_idx = (prev_nearest_idx_ + i) % path_size;
        double dist = std::hypot(path_[curr_idx].x - fx, path_[curr_idx].y - fy);

        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = curr_idx;
        }
    }
    prev_nearest_idx_ = nearest_idx;

    // 2. 경로의 접선 Heading 계산 (점 간격이 촘촘하므로 +5 ~ +10번째 앞의 점을 바라봄)
    int look_ahead_idx = (nearest_idx + 5) % path_size;
    
    double path_yaw = std::atan2(path_[look_ahead_idx].y - path_[nearest_idx].y,
                                 path_[look_ahead_idx].x - path_[nearest_idx].x);

    // 3. Heading Error (경로 방향과 로봇 헤딩의 차이)
    double heading_error = path_yaw - theta_;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error)); // -파이~파이 정규화
        
    // 4. Local 좌표계 변환을 통한 CTE 계산
    // 차량 전방 축 기준 경로점의 상대 위치
    double map_dx = path_[nearest_idx].x - fx;
    double map_dy = path_[nearest_idx].y - fy;

    // 차량 Local 좌표계 변환 (local_y > 0 이면 경로점이 차량의 '왼쪽'에 위치)
    // double local_x =  map_dx * std::cos(theta_) + map_dy * std::sin(theta_);
    double local_y = -map_dx * std::sin(theta_) + map_dy * std::cos(theta_);

    // 경로점이 왼쪽에 있으면 local_y > 0 이고, 로봇은 '좌회전(+)'을 해야 경로로 복귀함
    double cte = local_y; 

    // 5. Stanley Steering Angle 계산
    double k = 2.0;      // Stanley Gain (경로 복귀 강도)
    double k_soft = 0.01; // 저속 분모 안정화 상수
    
    // Stanley 횡오차 보정항 (CTE 보정)
    double cte_term = std::atan2(k * cte, v_ + k_soft);

    // 최종 목표 조향각 (Heading 보정 + CTE 보정)
    double delta = heading_error + cte_term;
    delta = std::atan2(std::sin(delta), std::cos(delta)); // [-PI, PI] 범위 제한

    // 6. Twist 명령 발행 및 타겟 포인트 마커 표시
    geometry_msgs::Twist cmd;
    cmd.linear.x = v_;
    
    // Differential Drive 각속도 P 제어
    double k_angular = 1.5; 
    cmd.angular.z = k_angular * delta; 


    // Target Marker 표시 (현재 추종중인 경로점 확인용)
    publishTargetPoint(path_[nearest_idx]);
    cmd_pub_.publish(cmd);
}

void PurePursuit::control_app()
{
    
    if (path_.empty())
    {
        return;
    }
    
    // 어댑티브 Lookahead
    double K_v = 0.5;
    double Ld_min = 0.3;
    double Ld_max = 2.0;
    Ld_ = K_v * v_ + Ld_min;
    Ld_ = std::max(Ld_min, std::min(Ld_, Ld_max));

    // Ld_ = 1.5;

    int path_size = path_.size();

    int window_size = 30; // 현위치에서 앞으로 20개 점만 탐색
    int nearest_idx = prev_nearest_idx_; //초깃값=0
    double min_dist = std::numeric_limits<double>::max(); // 큰값으로초기화
    
    // 경로에서 가장 가까운 곳 찾는 것,
    for (int i=0; i<window_size; ++i) 
    {
        int curr_idx = (prev_nearest_idx_ + i) % path_size; 
        
        double dist = std::hypot(path_[curr_idx].x - x_, path_[curr_idx].y - y_);

        if (dist < min_dist)
        {
            min_dist = dist; // 가장 짧은 거리 남음
            nearest_idx = curr_idx; // 가장 가까운 점 번호 남음
        }
    }   

    prev_nearest_idx_ = nearest_idx; //  인덱스업데이트

    geometry_msgs::Point target_point = path_[nearest_idx];
    double actual_dist = min_dist;

    // 가까운 점에서 앞으로 하나씩 검사, Ld_ 보다 큰 거 나오면 거기가 타겟
    for (int i=0; i<window_size+70; ++i)
    {
        int curr_idx = (nearest_idx+i) % path_size;
        double dist = std::hypot(path_[curr_idx].x - x_, path_[curr_idx].y - y_);

        if (dist >= Ld_)
        {
            target_point = path_[curr_idx];
            actual_dist = dist;
            break;
        }
    }

    double dx = target_point.x - x_;
    double dy = target_point.y - y_;

    double local_x =  dx * cos(theta_) + dy * sin(theta_);
    double local_y = -dx * sin(theta_) + dy * cos(theta_);

    geometry_msgs::Twist cmd;
    
    double curvature = (2.0 * local_y) / (actual_dist * actual_dist); // (Ld_ * Ld_)

    double max_speed = 1.5;
    v_ = max_speed / (1.0 + 2.0 * fabs(curvature));

    double angular_gain = 1.5;

    cmd.linear.x = v_;
    cmd.angular.z = angular_gain * v_ * curvature; // 각속도=선속도x곡률

    publishTargetPoint(target_point);
    cmd_pub_.publish(cmd); 
}

// void PurePursuit::control_rpp() // 속도를 각각 계산하고 제일 낮은 거 선택함
// {
// // Regulated Pure Pursuit
// // 1. Path Pruning 
// // 2. Adaptive Lookahead 
// // 3. Target Point Search 
// // 4. Curvature Calculation 
// // 5. Velocity Regulation
// //    ├─ Curvature Constraint 
// //    ├─ Obstacle(Cost) Constraint
// //    ├─ Goal Approach Constraint 
// //    └─ User Max Speed
// // 6. Collision Checking
// // 7. Rotate to Path Heading
// // 8. Rotate to Goal Heading
// // 9. cmd_vel 출력    

//     if (path_.empty())
//     {
//         return;
//     }

//     //골도착멈춤
//     double goal_dist = std::hypot(goal_.x - x_, goal_.y - y_);
    
//     if (goal_dist < 0.01) // 
//     {
//         geometry_msgs::Twist cmd;
//         cmd.linear.x = 0.0;
//         cmd.angular.z = 0.0;
//         cmd_pub_.publish(cmd);
//         return;
//     }

//     // Lookahead
//     double K_v = 0.5;
//     double Ld_min = 0.3;
//     double Ld_max = 2.0;
//     Ld_ = K_v * v_ + Ld_min;
//     Ld_ = std::max(Ld_min, std::min(Ld_, Ld_max));

//     // Ld_ = 1.5;

//     int path_size = path_.size();

//     if (remaining_dist < Ld_)
//     {
//         target_point = gaol_;
//         actual_dist = goal_dist;
//         target_idx_ = path_size - 1;
//     }
//     else
//     {
//         int window_size = 30; // 현위치에서 앞으로 20개 점만 탐색
//         int nearest_idx = prev_nearest_idx_; //초깃값=0
//         double min_dist = std::numeric_limits<double>::max(); // ㅋ큰값으로초기화
        
//         // 경로에서 가장 가까운 곳 찾는 것,
//         for (int i=0; i<window_size; ++i) 
//         {
//             // int curr_idx = (prev_nearest_idx_ + i) % path_size; 
//             int curr_idx = (prev_nearest_idx_ + i); 
            
//             double dist = std::hypot(path_[curr_idx].x - x_, path_[curr_idx].y - y_);

//             if (dist < min_dist)
//             {
//                 min_dist = dist; // 가장 짧은 거리 남음
//                 nearest_idx = curr_idx; // 가장 가까운 점 번호 남음
//             }
//         }   

//         prev_nearest_idx_ = nearest_idx; //  인덱스업데이트

//         geometry_msgs::Point target_point = path_[nearest_idx];
//         double actual_dist = min_dist;

//         int search_start = std::max(nearest_idx, target_idx_); //target_idx 초깃값 0

//         // 가까운 점에서 앞으로 하나씩 검사, Ld_ 보다 큰 거 나오면 거기가 타겟임
//         for (int curr_idx = search_start; curr_idx < std::min(search_start+window_size, path_size); ++curr_idx)
//         {
//             double dist = std::hypot(path_[curr_idx].x - x_, path_[curr_idx].y - y_);

//             if (dist >= Ld_)
//             {
//                 target_point = path_[curr_idx];
//                 actual_dist = dist;
//                 target_idx_ = curr_idx;
//                 break;
//             }
//         }
//     }

//     ROS_INFO("nearest=%d target=%d path=%d Ld=%.2f",
//         nearest_idx,
//         target_idx_,
//         path_size,
//         Ld_);

//     // 남은 거리
//     double remaining_dist = 0.0;

//     for(int i = nearest_idx; i < path_size-1; i++)
//     {
//         remaining_dist += std::hypot(path_[i+1].x - path_[i].x, path_[i+1].y - path_[i].y);
//     }
//     ROS_INFO("remaining distance=%.2f", remaining_dist);

//     double dx = target_point.x - x_;
//     double dy = target_point.y - y_;

//     double local_x =  dx * cos(theta_) + dy * sin(theta_);
//     double local_y = -dx * sin(theta_) + dy * cos(theta_);

//     geometry_msgs::Twist cmd;
    
//     double curvature = (2.0 * local_y) / (actual_dist * actual_dist); // 곡률=2y/L_d^2

//     double min_speed = 0.05;
//     double max_speed = 1.5;

//     double angular_gain = 1.5;

//     double v_curve = v_;
//     double v_goal = max_speed;
//     double slow_dist = 3.0;

//     v_ = max_speed / (1.0 + 2.0 * fabs(curvature));
//     cmd.angular.z = angular_gain * v_ * curvature; // 각속도=선속도x곡률

//     if (remaining_dist < slow_dist)
//     {
//         v_goal = min_speed + (max_speed - min_speed) * (remaining_dist / slow_dist);
//         v_ = std::min(v_curve, v_goal);
//         cmd.linear.x = v_;
//     }
//     else
//     {
//     cmd.linear.x = v_; 
//     }

//     publishTargetPoint(target_point);
//     cmd_pub_.publish(cmd); 
// }

void PurePursuit::control_rpp() // 속도를 각각 계산하고 제일 낮은 거 선택함
{
// Regulated Pure Pursuit
// 1. Path Pruning
// 2. Remaining Path Distance
// 3. Adaptive Lookahead
// 4. Target Point Search
// 5. Curvature Calculation
// 6. Velocity Regulation
//    ├─ Curvature Constraint
//    ├─ Obstacle(Cost) Constraint
//    ├─ Goal Approach Constraint
//    └─ User Max Speed
// 7. Collision Checking
// 8. Rotate to Path Heading
// 9. Rotate to Goal Heading
// 10. cmd_vel 출력

    if (path_.empty())
    {
        return;
    }

    int path_size = path_.size();

    // 골까지 직선거리
    double goal_dist = std::hypot(goal_.x - x_, goal_.y - y_);

    // 골도착하면 멈춤
    if (goal_dist < 0.01)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_.publish(cmd);
        return;
    }

    // 1. Path Pruning (Nearest Point Search)=================================================
    int window_size = 30;

    int nearest_idx = prev_nearest_idx_; // 초깃값 0
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < window_size; ++i)
    {
        int curr_idx = prev_nearest_idx_ + i;

        if (curr_idx >= path_size)
            break;

        double dist = std::hypot(path_[curr_idx].x - x_,
                                 path_[curr_idx].y - y_);

        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = curr_idx;
        }
    }

    prev_nearest_idx_ = nearest_idx;

    // 2. Remaining Path Distance==================================================
    // 남은 경로 길이로 골에 가까운지 확인함
    double remaining_dist = 0.0;

    for (int i = nearest_idx; i < path_size - 1; ++i)
    {
        remaining_dist += std::hypot(path_[i + 1].x - path_[i].x,
                                     path_[i + 1].y - path_[i].y);
    }

    // 3. Adaptive Lookahead=======================================================
    // 속도에 비례해서 L_d 조절
    double K_v = 0.5;
    double Ld_min = 0.3;
    double Ld_max = 2.0;

    Ld_ = K_v * v_ + Ld_min;
    Ld_ = std::max(Ld_min, std::min(Ld_, Ld_max));

    // 4. Target Point Search=========================================================
    // L_d랑 가까운 타겟 찾기
    geometry_msgs::Point target_point;
    double actual_dist = 0.0;

    
    if (remaining_dist < Ld_) //목적지 근처면 그 목적지 점이 타겟
    {
        target_point = goal_;
        actual_dist = goal_dist;
        target_idx_ = path_size - 1;
    }
    else
    {
        target_point = path_[nearest_idx];
        actual_dist = min_dist;

        int search_start = std::max(nearest_idx, target_idx_);

        for (int curr_idx = search_start;
             curr_idx < std::min(search_start + window_size, path_size);
             ++curr_idx)
        {
            double dist = std::hypot(path_[curr_idx].x - x_,
                                     path_[curr_idx].y - y_);

            if (dist >= Ld_)
            {
                target_point = path_[curr_idx];
                actual_dist = dist;
                target_idx_ = curr_idx;
                break;
            }
        }
    }

    // 5. Curvature Calculation==========================================================
    // 타겟과의 곡률 계산
    double dx = target_point.x - x_;
    double dy = target_point.y - y_;

    double local_x =  dx * cos(theta_) + dy * sin(theta_);
    double local_y = -dx * sin(theta_) + dy * cos(theta_);

    double curvature = (2.0 * local_y) / (actual_dist * actual_dist); // 곡률 = 2y/L_d^2

    // 6. Velocity Regulation========================================================
    double min_speed = 0.05;
    double max_speed = 1.5;
    double slow_dist = 3.0;

    // Curvature Constraint
    // 커브구간에서 감속
    double v_curve = max_speed / (1.0 + 2.0 * fabs(curvature));

    // Goal Approach Constraint
    // 목표에 다가갈수록 감속
    double v_goal = max_speed;

    if (remaining_dist < slow_dist)
    {
        v_goal = min_speed +
                 (max_speed - min_speed) *
                 (remaining_dist / slow_dist);
    }

    // Obstacle(Cost) Constraint
    //장애물 가까워질수록 감속
    double v_cost = 99.99;

    // User Max Speed
    v_ = std::min({max_speed, v_curve, v_goal, v_cost});


    // 7. Collision Checking=========================================================
    // 경로 충돌 유무 검사

    // 8. Rotate to Path Heading======================================================
    // 경로와 헤딩 차이 크면 제자리 회전
    double target_yaw = std::atan2(target_point.y - y_, target_point.x - x_);
    double heading_error = target_yaw - theta_;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
    // double heading_error = tf2NormalizeAngle(target_yaw - theta_);

    double max_heading_error = 0.785; // 45도
    if (std::abs(heading_error) > max_heading_error)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0; 
        
        double k_p_rotate = 0.5;
        cmd.angular.z = std::clamp(k_p_rotate * heading_error, -2.0, 2.0);
        
        cmd_pub_.publish(cmd);
        return; 
    }

    // 9. Rotate to Goal Heading========================================================
    // 목표와 헤딩 차이 크면 제자리 회전



    //===================================================================================
    // Dynamic Window Pure Pursuit
    //===================================================================================

    // ... (1~6단계: 기존 RPP 로직으로 target_point, curvature, v_ (regulated_linear_vel) 계산 완료 상태)

    // -----------------------------------------------------------------------------
    // [추가] DWPP 적용 파트
    // -----------------------------------------------------------------------------
    // 1. DWPP에 넘겨줄 current_speed 변수 정의 (ROS 1 타입) (임시 조치)
    geometry_msgs::Twist current_speed;
    current_speed.linear.x = v_;       // 현재/이전 계산된 목표 선속도 (또는 odom 선속도)
    current_speed.angular.z = v_ * curvature; // 현재/이전 계산된 목표 각속도

    // 파라미터/제약 조건 설정
    double max_linear_vel = 1.5;
    double min_linear_vel = 0.0;
    double max_angular_vel = 2.0;
    double min_angular_vel = -2.0;

    double max_linear_accel = 0.5; // 최대 선가속도 (m/s^2)
    double max_linear_decel = -1.0; // 최대 선감속도 (m/s^2)
    double max_angular_accel = 1.0; // 최대 각가속도 (rad/s^2)
    double max_angular_decel = -1.5; // 최대 각감속도 (rad/s^2)

    double dt = 0.05; // 제어 주기 (예: 20Hz -> 0.05초)
    double x_vel_sign = 1.0; // 전진 주행

    // DWPP 계산 실행
    auto [optimal_v, optimal_w] = 
        nav2_regulated_pure_pursuit_controller::dynamic_window_pure_pursuit::computeDynamicWindowVelocities(
            current_speed, // 현재 로봇 속도 (geometry_msgs::msg::Twist)
            max_linear_vel, min_linear_vel,
            max_angular_vel, min_angular_vel,
            max_linear_accel, max_linear_decel,
            max_angular_accel, max_angular_decel,
            v_, // RPP가 감속 제약으로 1차 계산한 선속도
            curvature, // RPP가 계산한 곡률
            x_vel_sign,
            dt
        );
    // -----------------------------------------------------------------------------
    // cmd_vel Publish
    // -----------------------------------------------------------------------------
    geometry_msgs::Twist cmd;
    cmd.linear.x = optimal_v;  // DWPP로 가/감속도 제약이 반영된 최종 선속도
    cmd.angular.z = optimal_w; // DWPP로 산출된 최종 각속도

    publishTargetPoint(target_point);
    cmd_pub_.publish(cmd);

    // 10. cmd_vel Publish==================================================================
    // 최종 속도 발행
    // geometry_msgs::Twist cmd;

    // cmd.linear.x = v_;

    // double angular_gain = 1.5;
    // cmd.angular.z = angular_gain * v_ * curvature;

    // publishTargetPoint(target_point);
    // cmd_pub_.publish(cmd);

    ROS_INFO("nearest=%d target=%d remaining=%.2f Ld=%.2f v=%.2f",
             nearest_idx,
             target_idx_,
             remaining_dist,
             Ld_,
             v_);
}


void PurePursuit::publishTurtleTF()
{
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp = ros::Time::now();

    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "turtle1";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_msg);
}

void PurePursuit::generatePath_C()
{
    path_.clear();
    double center_x = 5.5; 
    double center_y = 5.5; 
    double radius = 3.0;  

    for(double t = 0; t <= 2 * M_PI + 0.05; t += 0.05)
    {
        geometry_msgs::Point p;
        p.x = center_x + radius * cos(t);
        p.y = center_y + radius * sin(t);
        path_.push_back(p);
    }
}

void PurePursuit::generatePath_LC()
{
    path_.clear();
    for(double t = 0.1; t <= 2.0*M_PI-0.1; t += 0.01)
    {
        geometry_msgs::Point p;
        p.x = 5.5 + 4.0 * sin(3*t);
        p.y = 5.5 + 4.0 * sin(4*t);
        path_.push_back(p); // .push_back 파이썬 append 같은 거 
    }
    goal_ = path_.back(); //마지막 점이 도착점
}

void PurePursuit::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    waypoints_.push_back(msg->pose.position);

    path_L.poses.clear();

    path_L.header.frame_id = "map";
    path_L.header.stamp = ros::Time::now();

    double interval = 0.05;

    for(int i=0;i<waypoints_.size()-1;i++)
    {
        geometry_msgs::Point start = waypoints_[i];
        geometry_msgs::Point end   = waypoints_[i+1];

        double dx = end.x-start.x;
        double dy = end.y-start.y;

        double dist = hypot(dx,dy);

        int steps = std::max(1,(int)(dist/interval));

        for(int j=0;j<=steps;j++)
        {
            double t=(double)j/steps;

            geometry_msgs::PoseStamped pose;

            pose.header.frame_id="map";

            pose.pose.position.x=start.x+t*dx;
            pose.pose.position.y=start.y+t*dy;

            path_L.poses.push_back(pose);
        }
        
        path_.clear();

        for(auto &p : path_L.poses)
        {
            path_.push_back(p.pose.position);
        }
    }


    path_pub_L_.publish(path_L);
}

// void PurePursuit::generatePath_L()
void PurePursuit::publishPath()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "pure_pursuit";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;

    for(const auto& p : path_)
    {
        marker.points.push_back(p); 
    }
    path_pub_.publish(marker);
}

void PurePursuit::publishTargetPoint(const geometry_msgs::Point& target_p)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "pure_pursuit_target";
    marker.id = 1; 

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = target_p;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0; 
    marker.color.b = 0.0;

    target_pub_.publish(marker);
}

// void PurePursuit::reconfigureCallback(pure_pursuit::PurePursuitConfig &config, uint32_t level)
// {
//     v_ = config.v;
//     Ld_ = config.Ld;
//     ROS_INFO("Update Params v: %.2f, Ld: %.2f", v_, Ld_);
// }