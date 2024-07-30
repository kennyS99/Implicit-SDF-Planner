#include <plan_manager/plan_manager.hpp>
#include <thread>
#include <utils/Visualization.hpp>

using namespace std;
using namespace ros;
using namespace Eigen;

#define USE_MIASTAR  1
#define USE_SE3_RRT  2
#define USE_DYNASTAR 3

int main(int argc, char **argv) {
    ros::init(argc, argv, "plan_manager");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv("~");

    debug_publisher::init(nh_);
    ros::Duration(0.5).sleep();

    debug_publisher::DBSendNew("plan_manager", "Program start");
    planner_manager.reset(new PlannerManager);
    planner_manager->init(nh_, nh_priv);

    debug_assistant.reset(new DebugAssistant);
    debug_assistant->init(nh_, planner_manager);
    debug_publisher::DBSendNew("plan_manager", "Program init done!");

    freopen("/catkin_ws/Implicit-SDF-Planner/output.log", "a", stdout);

    double test_rate = planner_manager->config.testRate;
    if (test_rate > 0.0) {
        ros::Rate lr(test_rate);
        while (ros::ok()) {
            planner_manager->process();
            ros::spinOnce();
            lr.sleep();
        }
    } else {
        ros::spin();
    }
    ros::spin();
    debug_assistant->callback_thread.join();
    return 0;
}
void PlannerManager::resetrandommap(pcl::PointCloud<pcl::PointXYZ> &global_map_pcl_cloud) {
    double x, y, z;
    double len, wid, hig;
    double cloud_resolution = 0.6;
    geneWall(0, 0, 0.2, 0.2, 3.0, global_map_pcl_cloud);
    geneWall(60, 60, 35.0, 0.2, 0.2, 3.0, global_map_pcl_cloud);

    double num = 200;
    srand(time(NULL));
    pcl::PointXYZ s_point;
    for (int i = 0; i < num; i++) {
        x = (rand() % 400 + 100) / 10;
        y = (rand() % 400 + 100) / 10;
        z = (rand() % 250 + 50) / 10;

        len = 2.0 * cloud_resolution;
        wid = 2.0 * cloud_resolution;
        hig = 2.0 * cloud_resolution;

        geneWall(x, y, z, len, wid, hig, global_map_pcl_cloud);
    }
}

void PlannerManager::geneWall(double ori_x, double ori_y, double ori_z, double length, double width, double height, pcl::PointCloud<pcl::PointXYZ> &global_map_pcl_cloud, double tor) {
    pcl::PointXYZ s_point;
    double cloud_resolution = 0.6;
    Eigen::Vector3d pos_eva, gradp_rel;
    double tf = 0.0;
    for (double t_z = ori_z; t_z < height + ori_z; t_z += cloud_resolution) {
        for (double t_y = ori_y; t_y < ori_y + width; t_y += cloud_resolution) {
            for (double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution) {
                s_point.x = t_x + (rand() % 10) / 250.0;
                s_point.y = t_y + (rand() % 10) / 250.0;
                s_point.z = t_z + (rand() % 10) / 800.0;
                pos_eva = Eigen::Vector3d{s_point.x, s_point.y, s_point.z};
                if (planner_manager->sv_manager->getSDFofSweptVolume<false>(pos_eva, tf, gradp_rel, false) <= tor) {
                    std::cout << "continue" << std::endl;
                    continue;
                }
                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void PlannerManager::geneWall(double ori_x, double ori_y, double length, double width, double height, pcl::PointCloud<pcl::PointXYZ> &global_map_pcl_cloud, double tor) {
    pcl::PointXYZ s_point;
    double cloud_resolution = 0.6;
    Eigen::Vector3d pos_eva, gradp_rel;
    double tf = 0.0;
    for (double t_z = 0.0; t_z < height; t_z += cloud_resolution) {
        for (double t_y = ori_y; t_y < ori_y + width; t_y += cloud_resolution) {
            for (double t_x = ori_x; t_x < ori_x + length; t_x += cloud_resolution) {
                s_point.x = t_x + (rand() % 10) / 250.0;
                s_point.y = t_y + (rand() % 10) / 250.0;
                s_point.z = t_z + (rand() % 10) / 800.0;
                pos_eva = Eigen::Vector3d{s_point.x, s_point.y, s_point.z};
                if (planner_manager->sv_manager->getSDFofSweptVolume<false>(pos_eva, tf, gradp_rel, false) <= tor) {
                    std::cout << "continue" << std::endl;
                    continue;
                }

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void PlannerManager::init(ros::NodeHandle &nh, ros::NodeHandle &nh_prev) {
    debug_publisher::DBSendNew("plan_manager", "planner_manager init start");
    config.loadParameters(nh_prev);

    bdx = config.kernel_size * config.occupancy_resolution;  // The length, width, and height of the bounding box
    bdy = config.kernel_size * config.occupancy_resolution;  // The length, width, and height of the bounding box
    bdz = config.kernel_size * config.occupancy_resolution;  // The length, width, and height of the bounding box

    // Initialize Point Cloud Subscription Map Manager
    // PCSmapManager:
    // It is responsible for managing the point cloud map, mainly used for A* path search and visualization. Configuration parameters need to be passed during initialization.
    pcsmap_manager.reset(new PCSmapManager(config));
    pcsmap_manager->init(nh);
    debug_publisher::DBSendNew("plan_manager", "pcsmap_manager init done");

    // Initialize Swept Volume Manager
    // SweptVolumeManager: This module is used to manage swept
    // volume, mainly for collision detection. In addition to configuration parameters, ROS node handle nh needs to be passed during initialization.
    sv_manager.reset(new SweptVolumeManager(config));
    sv_manager->init(nh, config);
    debug_publisher::DBSendNew("plan_manager", "sv_manager init done");

    // Initialize A* Path Searcher
    // AstarPathSearcher:
    // This is an A* path searcher, used to search for a collision-free path in the point cloud map given the start and end points.
    astar_searcher.reset(new AstarPathSearcher);
    astar_searcher->init(nh);

    debug_publisher::DBSendNew("plan_manager", "front_end init done");

    // Initialize Trajectory Optimizer
    // TrajOptimizer:
    // This module is responsible for trajectory optimization, that is, generating a more optimal collision-free trajectory given the initial trajectory. The swept volume manager needs to be set
    // during initialization for collision detection.
    traj_parlength = 3.0;
    minco_traj_optimizer.reset(new TrajOptimizer);
    minco_traj_optimizer->setParam(nh, config);
    minco_traj_optimizer->setEnvironment(sv_manager);

    debug_publisher::DBSendNew("plan_manager", "back_end init done");

    // Initialize Original Trajectory Generator
    // OriTraj:
    // This is the original trajectory generator, used to generate an initial trajectory from the start to the end point, providing a good starting point for subsequent optimization.
    ori_traj_generator.reset(new OriTraj);
    ori_traj_generator->setParam(nh, config);

    current_front_end = USE_MIASTAR;

    visulizer.reset(new vis::Visualization(nh));
    target_sub = nh.subscribe("/goal", 1, &PlannerManager::targetRcvCallBack, this);
    rcvmap_signal_sub = nh.subscribe("/rcvmap_signal", 1, &PlannerManager::mapRcvCallBack, this);
    setting_sub = nh.subscribe("/setting_receive", 1, &PlannerManager::settingRcvCallBack, this);
    // vis_timer = nh.createTimer(ros::Duration(1), &PlannerManager::viscallback,
    // this); // Periodically clear or re-visualize
    rs = nh.subscribe("/reshow", 1, &PlannerManager::reShowTraj, this);

    debug_publisher::DBSendNew("plan_manager", "planner_manager init done");

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("aabb_visited_points", 1);
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("aabb_center_points", 1);
}

void PlannerManager::viscallback(const ros::TimerEvent &event) {
    // std::cout<<"visualization"<<std::endl;
    // minco_traj_optimizer->renderAABBpoints();
}
bool PlannerManager::generatePath(Vector3d start, Vector3d end) {
    if (current_front_end == USE_MIASTAR) {
        astar_searcher->AstarPathSearch(start, end);
        if (astar_searcher->success_flag) {
            recent_path = astar_searcher->getPath();
            visulizer->visR3Path("front_end_path", recent_path);
            recent_se3_path = astar_searcher->getastarSE3Path();
            visulizer->visSE3Path("SE3path", sv_manager->getRobotShapeMesh(), recent_se3_path);
            ROS_WARN("[A*] search success.");
        } else {
            ROS_WARN("[A*] search failed.");
        }
        astar_searcher->reset();
        return (astar_searcher->success_flag);
    }
}

void PlannerManager::generateTraj(const vector<Vector3d> &path) {
    int trajectory_pieces;
    int path_size = path.size();
    // traj_parlength{1.0};
    double temp_traj_parlength = traj_parlength;
    int index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));
    // grid_resolution = 1.0
    // index_gap =1
    while (index_gap >= path_size - 1) {
        temp_traj_parlength /= 1.5;
        index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));
    }

#ifdef USE_DYN_A_STAR
    index_gap = 1;
#endif

    bool ret_opt;
    // 3x3 matrix
    Matrix3d initState = Matrix3d::Zero();
    Matrix3d finalState = Matrix3d::Zero();
    initState.col(0) = path.front();
    finalState.col(0) = path.back();

    vector<Vector3d> Intermediate_waypoints;
    vector<Vector3d> acc_list;
    Vector3d wp;
    Vector3d last_wayPoint{999, 999, 999};
    Vector3d dir;
    Matrix3d rotate;
    vector<Matrix3d> rotatelist;
    minco_traj_optimizer->pcsmap_manager->aabb_points.clear();
    // offset = 0
    const Eigen::Vector3d offset = Eigen::Vector3d(config.offsetAABBbox[0], config.offsetAABBbox[1], config.offsetAABBbox[2]);
    cout << "\033[32mOffset for AABB box:\033[0m" << offset.transpose() << std::endl;
    // index_gap = 1;

    for (const auto &list : path) {
        cout << "list:" << list.transpose() << endl;
    }

    // for (int ind = index_gap; ind < path_size - 1; ind += index_gap) {
    //     // wp = path[ind];
    //     wp = Vector3d(7.5, 18.5, 9.5);
    //     rotate = recent_se3_path[ind].getRotMatrix();
    //     dir = rotate * Eigen::Vector3d(0, 0, 1);
    //     Intermediate_waypoints.push_back(wp);
    //     rotatelist.push_back(rotate);
    //     acc_list.push_back(dir);
    //     // find nearby occupied grid center and saved to aabb_points
    //     // bdx = bdy = bdz = 15
    //     // Generate a bounding box centered at wp, find all the occupied grid center world coordinates.
    //     cout << "current check way point is:" << wp.transpose() << endl;
    //     cout << "last check way point is:" << last_wayPoint.transpose() << endl;
    //     minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(wp, last_wayPoint, bdx / 3.0, bdy / 3.0, bdz / 3.0, offset);

    //     last_wayPoint = wp;
    // }

    // test purpose

    wp = Vector3d(7.5, 18.5, 9.5);
    rotate = recent_se3_path[2].getRotMatrix();
    dir = rotate * Eigen::Vector3d(0, 0, 1);
    Intermediate_waypoints.push_back(wp);
    rotatelist.push_back(rotate);
    acc_list.push_back(dir);
    // find nearby occupied grid center and saved to aabb_points
    // bdx = bdy = bdz = 15
    // Generate a bounding box centered at wp, find all the occupied grid center world coordinates.
    cout << "current check way point is:" << wp.transpose() << endl;
    cout << "last check way point is:" << last_wayPoint.transpose() << endl;
    minco_traj_optimizer->pcsmap_manager->getPointsInAABBOutOfLastOne(wp, last_wayPoint, bdx / 3.0, bdy / 3.0, bdz / 3.0, offset);

    last_wayPoint = wp;

    publishPointCloud(pcsmap_manager->AABB_visited_points, point_cloud_pub);
    publishSinglePoint(pcsmap_manager->AAbb_center_point, point_pub);

    minco_traj_optimizer->parallel_points.clear();
    // 设置优化的障碍物点云
    for (const auto &pair : minco_traj_optimizer->pcsmap_manager->aabb_points) {
        // pair.first Key
        // pair.second Value (the center of the obstacle)
        minco_traj_optimizer->parallel_points.push_back(pair.second);  // 便于多线程加速,每次可能不一样？
    }

    minco_traj_optimizer->parallel_points_num = minco_traj_optimizer->pcsmap_manager->aabb_points.size();
    cout << "\033[32m=========parallel_points_num(occupied grid numbers in the wayPoint):==========\033" << minco_traj_optimizer->parallel_points_num << endl;
    // create a vector of size parallel_points_num, and initialize all elements to 0.0
    minco_traj_optimizer->lastTstar = vector<double>(minco_traj_optimizer->parallel_points_num, 0.0);

    trajectory_pieces = Intermediate_waypoints.size() + 1;

    Matrix3Xd inPts = MatrixXd::Zero(3, trajectory_pieces);
    for (int i = 0; i < trajectory_pieces - 1; i++) {
        inPts.col(i) = Intermediate_waypoints[i];
    }
    cout << "inPts matrix:" << endl << inPts << endl;
    // config.inittime = 2.5
    VectorXd ts = config.inittime * VectorXd::Ones(trajectory_pieces);
    cout << "ts matrix:" << endl << ts.transpose() << endl;
    // ts = 2.5 2.5 2.5 2.5
    bool ret, ret2;
    bool mid_ret;
    ros::Time t1;
    // 调用OriTraj生成初始轨迹
    visulizer->visSE3Vec("path_vec", Intermediate_waypoints, acc_list, 156468);
    Eigen::VectorXd opt_x;
    // updated recent_traj is not used in the following code, this function only used for calcuate opt_x
    mid_ret = ori_traj_generator->getOriTraj(initState, finalState, Intermediate_waypoints, ts, acc_list, rotatelist, trajectory_pieces, recent_traj, opt_x);

    // 将初始轨迹传递给swept volume manager
    // sv_manager->updateTraj(recent_traj);
    // clear();
    std::cout << "[planner manager] Ori trajectory generated successfully!" << std::endl;

    if (mid_ret) {
        minco_traj_optimizer->integration_sequential_ms = 0.0;
        minco_traj_optimizer->integration_parallel_ms = 0.0;
        minco_traj_optimizer->discrete_sequential_ms = 0.0;
        minco_traj_optimizer->discrete_parallel_ms = 0.0;
        minco_traj_optimizer->cost_iter = 0;
        t1 = ros::Time::now();

        cout << "\033[32m=========使用lmbm优化========\033[0m" << endl;
        cout << "\033[32m=========使用lmbm优化========\033[0m" << endl;
        cout << "\033[32m=========使用lmbm优化========\033[0m" << endl;

        ret = minco_traj_optimizer->optimize_traj_lmbm(initState, finalState, opt_x, trajectory_pieces, recent_traj);

        cout << "\033[32m最终parallel Integration total time s:\033[0m" << minco_traj_optimizer->integration_parallel_ms / 1000.0 << endl;
        cout << "\033[32m最终parallel Integration average time ms:\033[0m" << minco_traj_optimizer->integration_parallel_ms / (minco_traj_optimizer->cost_iter) << endl;
        cout << "\033[32m最终parallel Discrete total time s:\033[0m" << minco_traj_optimizer->discrete_parallel_ms / 1000.0 << endl;
        cout << "\033[32m最终parallel Discrete average time ms:\033[0m" << minco_traj_optimizer->discrete_parallel_ms / (minco_traj_optimizer->cost_iter) << endl;
    }

    if (1) {
        std::cout << "[planner manager] Trajectory optimization is successful! " << std::endl;
        ros::Time t2 = ros::Time::now();
        std::cout << "Optimization time cost = " << (t2 - t1).toSec() * 1000 << " ms" << std::endl;
        visulizer->visTraj("traj", recent_traj);
        std::cout << " tmax for traj: " << recent_traj.getTotalDuration() << std::endl;
        sv_manager->updateTraj(recent_traj);

        if (sv_manager->isTrajCollide()) {
            ROS_WARN("traj collide.");
        }
        sv_manager->setTrajStamp(ros::Time::now().toSec());
        sv_manager->process(recent_traj);
    }
}

// SE3 front_end ver.
void PlannerManager::generateTraj(const vector<SE3State> &path) {
    debug_publisher::DBSendNew("plan_manager", "Try to generate trajectory based on R3 path.");
    int N;
    int path_size = path.size();
    double temp_traj_parlength = traj_parlength;
    int index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));

    while (index_gap >= path_size - 1) {
        temp_traj_parlength /= 1.5;
        index_gap = ceil(temp_traj_parlength / ((pcsmap_manager->occupancy_map)->grid_resolution));
    }

    bool ret_opt;
    Matrix3d initState = Matrix3d::Zero();
    Matrix3d finalState = Matrix3d::Zero();
    initState.col(0) = path.front().position;
    finalState.col(0) = path.back().position;

    vector<Vector3d> Q;
    Vector3d wp;
    for (int ind = index_gap; ind < path_size - 1; ind += index_gap) {
        wp = path[ind].position;
        Q.push_back(wp);
    }

    N = Q.size() + 1;

    Matrix3Xd inPts = MatrixXd::Zero(3, N);
    for (int i = 0; i < N; i++) {
        inPts.col(i) = Q[i];
    }
    VectorXd ts = config.inittime * VectorXd::Ones(N);

    minco::MINCO_S3NU minimal_jerk;
    minimal_jerk.setConditions(initState, finalState, N);
    minimal_jerk.setParameters(inPts, ts);
    minimal_jerk.getTrajectory(recent_traj);

    sv_manager->updateTraj(recent_traj);
    clear();

    std::cout << "[planner manager] Ori trajectory generated successfully!" << std::endl;
    debug_publisher::DBSendNew("plan_manager", "Ori trajectory generated.");

    ros::Time t1 = ros::Time::now();

    bool ret;

    ret = minco_traj_optimizer->optimize_traj_lmbm(initState, finalState, Q, ts, N, recent_traj);

    if (ret) {
        debug_publisher::DBSendNew("plan_manager", "Final trajectory optimization success!");
    } else {
        debug_publisher::DBSendNew("plan_manager", "Final trajectory optimization fail!");
    }
    std::cout << "[planner manager] Trajectory optimization is done! " << std::endl;
    ros::Time t2 = ros::Time::now();
    std::cout << "Optimization time cost = " << (t2 - t1).toSec() * 1000 << " ms" << std::endl;
    visulizer->visTraj("traj", recent_traj);
    sv_manager->updateTraj(recent_traj);
    if (sv_manager->isTrajCollide()) {
        ROS_WARN("traj collide.");
    }

    sv_manager->setTrajStamp(ros::Time::now().toSec());
    sv_manager->process(recent_traj);
}

void PlannerManager::clear() {
    planner_manager->sv_manager->swept_cal->sweptvolumeclear();
}

void PlannerManager::mapRcvCallBack(const std_msgs::Empty &msg) {
    debug_publisher::DBSendNew("plan_manager", "Try to gene byte kernel");
    uint8_t *map_kernel = pcsmap_manager->generateMapKernel(config.kernel_size);
    debug_publisher::DBSendNew("plan_manager", "gene byte kernel done!");

    int sizeX = pcsmap_manager->occupancy_map->X_size;
    int sizeY = pcsmap_manager->occupancy_map->Y_size;
    int sizeZ = pcsmap_manager->occupancy_map->Z_size;
    sv_manager->setMapKernel(map_kernel, sizeX, sizeY, sizeZ);
    debug_publisher::DBSendNew("plan_manager", "set byte kernel done!");
    astar_searcher->initGridMap(pcsmap_manager, sv_manager);
    minco_traj_optimizer->setGridMap(pcsmap_manager);
    cout << "init map A* --" << endl;
}

void PlannerManager::settingRcvCallBack(const std_msgs::Int8MultiArray &msg) {
#define CLEAR_MAP         1
#define SETTING_FRONT_END 2
#define SETTING_SHAPE     3

    if (msg.data.size() < 2) {
        return;
    }
    int head = msg.data[0];

    if (head == CLEAR_MAP) {
        pcsmap_manager->clearMap();
        std::cout << "[planner manager] clear map!" << std::endl;
    }
}
/**
 * @attention 记得更新可视化clear清除一些变量等等
 */
void PlannerManager::targetRcvCallBack(const geometry_msgs::PoseStamped &msg) {
    clear();
    if (step_state == STEP_NOPOINT) {
        start_pos(0) = msg.pose.position.x;
        start_pos(1) = msg.pose.position.y;
        start_pos(2) = msg.pose.position.z;
        step_state = STEP_HAVE_START;
        std::cout << "[plan_manager] Get start position! " << std::endl;
        debug_publisher::DBSendNew("plan_manager", "Get start position!");
    } else if (step_state == STEP_HAVE_START) {
        end_pos(0) = msg.pose.position.x;
        end_pos(1) = msg.pose.position.y;
        end_pos(2) = msg.pose.position.z;
        step_state = STEP_HAVE_TARGET;
        std::cout << "[plan_manager] Get target position! " << std::endl;

        debug_publisher::DBSendNew("plan_manager", "Get target position!");
    } else if (step_state == STEP_HAVE_TARGET) {
        start_pos(0) = msg.pose.position.x;
        start_pos(1) = msg.pose.position.y;
        start_pos(2) = msg.pose.position.z;
        step_state = STEP_HAVE_START;
        std::cout << "[plan_manager] Get start position! " << std::endl;
        debug_publisher::DBSendNew("plan_manager", "Get target position!");
    }

    if (step_state == STEP_HAVE_TARGET) {
        start_pos(0) = 3.447742;
        start_pos(1) = 23.630600;
        start_pos(2) = 6.539995;
        end_pos(0) = 13.432585;
        end_pos(1) = 20.269485;
        end_pos(2) = 9.960022;
        working = true;
        std::cout << "[plan_manager] Try to generate path. " << std::endl;
        debug_publisher::DBSendNew("plan_manager", "Try to generate path.");
        if (generatePath(start_pos, end_pos)) {
            std::cout << "[plan_manager] Path generated successfully! " << std::endl;
            debug_publisher::DBSendNew("plan_manager", "Path generated successfully!");

            if (current_front_end == USE_MIASTAR || current_front_end == USE_DYNASTAR) {
                generateTraj(recent_path);
            }

            if (current_front_end == USE_SE3_RRT) {
                generateTraj(recent_se3_path);
            }
        }
        working = false;
    }
}

void PlannerManager::process() {
    this->sv_manager->process(recent_traj);
}

void PlannerManager::reShowTraj(const std_msgs::Empty::Ptr msg) {
    sv_manager->setTrajStamp(ros::Time::now().toSec());
    sv_manager->process(recent_traj);
}

void PlannerManager::publishPointCloud(const std::vector<Eigen::Vector3d> &points, ros::Publisher &pub) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "map";  // or any other frame id

    for (const auto &point : points) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        cloud.points.push_back(pcl_point);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    pub.publish(output);
}

void PlannerManager::publishSinglePoint(const Eigen::Vector3d &point, ros::Publisher &pub) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "map";  // or any other frame id

    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    cloud.points.push_back(pcl_point);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    pub.publish(output);
}

void DebugAssistant::debugMsgcallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    planner_manager->sv_manager->swept_cal->sweptvolumeclear();

    if (msg->data.size() < 1) {
        return;
    }

    // stop optimization
    if (msg->data[0] == 21) {
        planner_manager->minco_traj_optimizer->exit = true;
    }

    // pause
    if (msg->data[0] == 22) {
        planner_manager->minco_traj_optimizer->pause = msg->data[1];
    }

    if (msg->data[0] == 101) {
        Config conf = planner_manager->config;
        vector<Vector3d> kernel;
        int xkernel_size = floor(2 * conf.kernel_max_roll / conf.kernel_ang_res) + 1;
        int ykernel_size = floor(2 * conf.kernel_max_pitch / conf.kernel_ang_res) + 1;
        int zero_pose_i = (xkernel_size - 1) / 2;
        int zero_pose_j = (ykernel_size - 1) / 2;

        Vector3d offset = Vector3d(-1, -1, -1);
        double res = conf.occupancy_resolution;

        kernel.clear();

        for (size_t x = 0; x < conf.kernel_size; x++) {
            for (size_t y = 0; y < conf.kernel_size; y++) {
                for (size_t z = 0; z < conf.kernel_size; z++) {
                    if (planner_manager->sv_manager->current_robot_shape->byte_shape_kernels[zero_pose_i * ykernel_size + zero_pose_j].getOccupied(x, y, z) == true)
                    // if (planner_manager -> sv_manager -> current_robot_shape ->
                    // byte_shape_kernels[zero_pose_i*ykernel_size +
                    // zero_pose_j].getOccupied(x,y,z) == true) if (planner_manager ->
                    // sv_manager -> current_robot_shape ->
                    // shape_kernels[zero_pose_i*ykernel_size +
                    // zero_pose_j].getOccupied(x,y,z) ==
                    // true)//这部分ok，从shape_kernels到byte_shape_kernels有问题
                    {
                        kernel.push_back(offset + Vector3d(x, y, z) * res);
                    }
                }
            }
        }
        planner_manager->visulizer->visPointcloudByVector(kernel, "zero_kernel_vis");
    }

    if (msg->data[0] == 102) {
        if (planner_manager->recent_path.size() == 0) {
            std::cout << "Error no trajectory now" << std::endl;
            return;
        }
        planner_manager->sv_manager->swept_cal->sweptvolumeclear();
        Eigen::MatrixXd U_;
        Eigen::MatrixXi G_;
        planner_manager->sv_manager->calculateSwept(U_, G_);
        return;
    }

    if (msg->data[0] == 103) {
        if (planner_manager->recent_path.size() == 0) {
            std::cout << "Error no trajectory now" << std::endl;
            return;
        }
        planner_manager->sv_manager->setTrajStamp(ros::Time::now().toSec());
        planner_manager->sv_manager->process(planner_manager->recent_traj);
        return;
    }
}

void DebugAssistant::init(ros::NodeHandle &nh, PlannerManager::Ptr pm) {
    debug_publisher::DBSendNew("plan_manager", "debug_assistant init start");

    planner_manager = pm;
    debug_publisher::DBSendNew("plan_manager", "debug_assistant init done");

    callback_thread = (std::thread([&]() {
        ros::NodeHandle private_nh("~");
        ros::Subscriber sub2 = private_nh.subscribe("/debug_cmd", 10, &DebugAssistant::debugMsgcallback, this);
        ros::Rate lr(10);

        while (ros::ok()) {
            ros::spinOnce();
            lr.sleep();
        }
    }));
}