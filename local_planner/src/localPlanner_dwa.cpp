#include <math.h>
#include <time.h>
#include <iostream>
#include <map>
#include <unordered_map>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/synchronizer.h>


//在速度和角速度空间采样。
double max_speed = 0.2; // ]m/s]
double mix_speed = -0.1; // [m/s]
double max_yaw_rate = 40.0 * M_PI / 180.0; // [rad]
double max_accel = 0.1; //[m/ss]
double max_delta_yaw_rate = 40.0 * M_PI/180.0; //[rad/ss]
double v_step = 0.02; // [m/s]
double yaw_step = 0.01 * M_PI/180.0; // [rad/s]
double predict_time = 3; // [s]
double dt = 0.02;
double resolution = 1.0; //[m]c
int layernums = 8;

std::unordered_map<std::string, std::string> layers_search = {{"bc", "tra_label_0_conf50"},
                                                              {"ac", "tra_label_1_conf50"},
                                                              {"ab", "tra_label_2_conf50"},
                                                              {"aa", "tra_label_3_conf50"},
                                                              {"ba", "tra_label_4_conf50"},
                                                              {"ca", "tra_label_5_conf50"},
                                                              {"cb", "tra_label_6_conf50"},
                                                              {"cc", "tra_label_7_conf50"}};

struct scout_state{
  double x = 0.0; //[m]
  double y = 0.0; //[m]
  double yaw = 0.0; //[rad]
  double v = 0.0; // [m/s]
  double omega = 0.0;// [rad/s]
};

ros::Publisher pubPath;
ros::Publisher costmapPub;
ros::Publisher car_modelPub;
scout_state car_model; bool car_model_flag = 0;

grid_map_msgs::GridMap gridmap; bool gridmap_flag = 0;

geometry_msgs::PoseStamped goal; bool goal_flag = 0;

nav_msgs::Path path;
nav_msgs::OccupancyGrid costmap;
nav_msgs::Odometry car_modelodom;
std::vector<double> dw(4);

void gridmap2costmap(const grid_map_msgs::GridMap &map){
  costmap.info.height = int (map.info.length_y/resolution);
  costmap.info.width = int (map.info.length_x/resolution);
  costmap.info.resolution = map.info.resolution;
  costmap.header.frame_id = map.info.header.frame_id;
  // std::cout << " costmap.header.frame_id = !" << costmap.header.frame_id << std::endl;

  costmap.info.origin.position.x = map.info.pose.position.x - map.info.length_x/2.0;
  costmap.info.origin.position.y = map.info.pose.position.y - map.info.length_y/2.0;
  
  int indexi = 6;
  int indexGridMap = map.data[indexi].data.size();
  costmap.data.clear();
  for(int y = 0; y < costmap.info.height; ++y)
    for(int x = 0; x < costmap.info.width; ++x){
      int indexcostmap = x + y * map.info.length_x/resolution;
      double cost = map.data[indexi].data[indexGridMap - indexcostmap - 1];
      if(cost < 0.5)
        costmap.data.push_back(0);
      else
        costmap.data.push_back(101);
      
    }
  costmapPub.publish(costmap);
}

// void Odom2MapCB(const grid_map_msgs::GridMap::ConstPtr &map, const nav_msgs::Odometry::ConstPtr &odom){
//   //path frame_id init
//   path.header.frame_id = odom->header.frame_id;

//   //car_model update 
//   car_model.x = odom->pose.pose.position.x;
//   car_model.y = odom->pose.pose.position.y;
//   double roll, pitch, yaw;
//   geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
//   tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
//   car_model.yaw = yaw;
//   car_model.v = odom->twist.twist.linear.x;
//   car_model.omega = odom->twist.twist.angular.z;
//   car_model_flag = 1;

//   // z = 0 odom visulization
//   car_modelodom = *odom;
//   car_modelodom.pose.pose.position.z = 0;
//   car_modelPub.publish(car_modelodom);

//   //grid update
//   gridmap = *map;
//   resolution = gridmap.info.resolution;
//   gridmap_flag = 1;
//   gridmap2costmap(*map);

// }

void odometryCB(const nav_msgs::OdometryConstPtr& msg)
{
  // car_model.x = msg.pose.pose.position.x;
  // car_model.y = msg.pose.pose.position.y;
  path.header.frame_id = msg->header.frame_id;
  // std::cout << " odometryCB.frame_id !" << msg->header.frame_id<<std::endl;

  car_model.x = msg->pose.pose.position.x;
  car_model.y = msg->pose.pose.position.y;
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  car_model.yaw = yaw;
  car_model.v = msg->twist.twist.linear.x;
  car_model.omega = msg->twist.twist.angular.z;
  car_model_flag = 1;
  // std::cout << " subscribe odom !" << std::endl;
  car_modelodom = *msg;

  car_modelodom.pose.pose.position.z = 0;
  car_modelPub.publish(car_modelodom);


}

void goalptCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // std::cout << " subscribe goal ! - 1" << std::endl;
  goal.pose.position.x = (*msg).pose.position.x;
  goal.pose.position.y = (*msg).pose.position.y;
  goal_flag = 1;
  // std::cout << " car_model.x  !" << car_model.x <<std::endl;
  // std::cout << " car_model.y  !"  << car_model.y<<std::endl;

  std::cout << " goal.pose.position.x goal !" << goal.pose.position.x<<std::endl;
  std::cout << " goal.pose.position.y goal !" << goal.pose.position.y<<std::endl;
  // std::cout << " goal.header.frame_id !" << goal.header.frame_id<<std::endl;
}



void gridmapCB(const grid_map_msgs::GridMapConstPtr& msg)
{
  gridmap = *msg;
  resolution = gridmap.info.resolution;
  gridmap_flag = 1;
  // gridmap2costmap(*msg);

  // std::cout << " subscribe map !" << std::endl;

}

void update(std::vector<double> &dw){
  // 0 - vmin, 1 - vmax, 2 - yawmin,3 - yawmax;
  dw[0] = std::max(car_model.v - max_accel*1.0, mix_speed);
  dw[1] = std::min(car_model.v + max_accel*1.0, max_speed);
  dw[2] = std::max(car_model.omega - max_delta_yaw_rate*1.0, -max_yaw_rate);
  dw[3] = std::min(car_model.omega + max_delta_yaw_rate*1.0, max_yaw_rate);

  return;
}

bool calTrajectory(std::vector<scout_state> &trajetory, double vel, double yaw_rate){

  scout_state pre_car = car_model;
  // std::cout << "vel = "<<vel << std::endl;
  // std::cout << "yaw_rate = "<<yaw_rate << std::endl;

  int flag = 1;
  if(vel < 0)
    flag = -1;
  
  pre_car.x = pre_car.x + 0.4*flag*cos(pre_car.yaw);
  pre_car.y = pre_car.y + 0.4*flag*sin(pre_car.yaw);

  trajetory.clear();
  trajetory.push_back(pre_car);
  double time = 0.0;
  while (time < predict_time)
  {
    pre_car.yaw += yaw_rate*dt;
    pre_car.x += vel * dt *cos(pre_car.yaw);
    pre_car.y += vel * dt *sin(pre_car.yaw);

    pre_car.v = vel;
    pre_car.omega = yaw_rate;
    trajetory.push_back(pre_car);
    // std::cout << "pre_car.yaw = "<<pre_car.yaw << std::endl;
    // std::cout << "pre_car.x = "<<pre_car.x << std::endl;
    // std::cout << "pre_car.y = "<<pre_car.y << std::endl;
    // std::cout << "pre_car.v = "<<pre_car.v << std::endl;
    // std::cout << "pre_car.omega = "<<pre_car.omega << std::endl;
    // std::cout << "trajetory.size() = "<<trajetory.size() << std::endl;

    time += dt;

  }

  return true;
}

double HeuGoaldis(const std::vector<scout_state> &trajetory, const geometry_msgs::PoseStamped &goal){
  scout_state car = trajetory[trajetory.size() - 1];
  int flag = 1;
  if(car_model.v < 0)
    flag = -1;


  double dx = car.x + 0.4*flag*cos(car_model.yaw) - goal.pose.position.x;
  double dy = car.y + 0.4*flag*sin(car_model.yaw) - goal.pose.position.y;
  // std::cout << "car.x " << car.x << std::endl;
  // std::cout << "car.y " << car.y << std::endl;
  double dis = sqrt(dx*dx + dy*dy);
  // std::cout << "dis" << dis << std::endl;


  return dis;
}

std::vector<int> Coord2index(double ptx, double pty){

  int indexptx = (ptx - (gridmap.info.pose.position.x - gridmap.info.length_x/2.0))/resolution;
  int indexpty = (pty - (gridmap.info.pose.position.y - gridmap.info.length_y/2.0))/resolution;
  // std::cout << "indexptx =  "<< indexptx <<std::endl;
  // std::cout << "indexpty  "<< indexpty <<std::endl;

  std::vector<int> index(2);

  index[0] = indexptx;
  index[1] = indexpty;
  // std::cout << "index[0]  "<< index[0] <<std::endl;
  // std::cout << "index[1]  "<< index[1] <<std::endl;
  return index;

}

int Getlayeri(const grid_map_msgs::GridMap& map, std::string laryername){
  int layerIndex;
  for (size_t i = 0; i < map.layers.size(); i++)
  {

      if (map.layers[i] == laryername)
      {
          layerIndex = i;
          break;
      }
  }
  return layerIndex;
}

bool Checktravesibility(const grid_map_msgs::GridMap& map, std::vector<int> index, int erx, int ery){
  if(erx == 0 && ery == 0)
    return true;

  // std::cout << "index[0] = "<< index[0]  <<std::endl;
  // std::cout << "index[1] = "<< index[1]  <<std::endl;
  // check all ;
  int layeri;
  int INX;
  double cost;
  INX = index[1] * gridmap.info.length_x/resolution + index[0];
  INX = gridmap.data[layeri].data.size() - INX - 1;

  // layeri = Getlayeri(map, "tra_label_2_conf50");


  // cost = map.data[layeri].data[INX];
  // if(cost < 0.5)
  //   return false;
  // else
  //   return true;
  //check all end;
  std::string layerkey;

  char key = erx == -1? 'a':erx == 0? 'b':'c';
  layerkey.push_back(key);

  key = ery == -1? 'a':ery == 0? 'b':'c';
  layerkey.push_back(key);

  layeri = Getlayeri(map, layers_search.at(layerkey));
  // std::cout << "layeri = "<< layeri<<std::endl;
  // std::cout << "layerkey = "<< layerkey<<std::endl;
  // std::cout << "erx = "<< erx<<std::endl;
  // std::cout << "ery = "<< ery<<std::endl;
  // std::cout << "layers_search.at(layerkey) = "<< layers_search.at(layerkey)<<std::endl;

  cost = map.data[layeri].data[INX];
  // std::cout << "cost = "<< cost<<std::endl;
  if(cost < 0.5){
    
    // std::cout << "layeri = "<< layeri<<std::endl;
    // std::cout << "layerkey = "<< layerkey<<std::endl;
    // std::cout << "erx = "<< erx<<std::endl;
    // std::cout << "ery = "<< ery<<std::endl;
    // std::cout << "layers_search.at(layerkey) = "<< layers_search.at(layerkey)<<std::endl;
    return false;


  }
  
  return true;

}

double HeuCollitiondis(const std::vector<scout_state> &trajetory, const grid_map_msgs::GridMap& map){

  double cost = 0.0;
  scout_state state1;
  scout_state state2;
  // std::cout << "trajetory.size() = "<< trajetory.size()<<std::endl;
  // ros::Rate checkrate(2);

  for(auto iter = trajetory.begin(); iter != trajetory.end(); ++iter){
    if(iter+1 == trajetory.end())
      continue;
    
    state1 = *iter;
    state2 = *(iter+1);
    // std::cout << "state1.x = "<< state1.x<<std::endl;
    // std::cout << "state1.y = "<< state1.y<<std::endl;
    // std::cout << "state2.x = "<< state2.x<<std::endl;
    // std::cout << "state2.y = "<< state2.y<<std::endl;

    std::vector<int> state1index = Coord2index(state1.x,state1.y);
    std::vector<int> state2index = Coord2index(state2.x,state2.y);

    // std::cout << "state1index = "<< state1index[0]<<std::endl;
    // std::cout << "state1index = "<< state1index[1]<<std::endl;
    // std::cout << "state2index = "<< state2index[0]<<std::endl;
    // std::cout << "state2index = "<< state2index[1]<<std::endl;
    // checkrate.sleep();
    
    int errorx = state2index[0] - state1index[0];
    int errory = state2index[1] - state1index[1];

    if(errorx == 0 && errory == 0){
      continue;
    }
    // std::cout << "errorx = "<< errorx<<std::endl;
    // std::cout << "errory = "<< errory<<std::endl;
    for(int i = -0; i <= 0; ++i)
      for(int j = -0; j <= 0; ++j)
      {
        state1index[0] += i;
        state1index[1] += j;
        if(!Checktravesibility(map, state1index, errorx, errory)){
          cost = 999999.0;
          break;
        }
      }

  }

  return cost;
}

double HeuerroU(const scout_state &car_model, const std::vector<double> &u){
  double cost = 0.0;

  double dv = car_model.v - u[0];
  double domega = car_model.omega - u[1];

  cost = sqrt(dv*dv + domega*domega);
  return cost;
}

double calpenalty(const std::vector<scout_state> &trajetory, const std::vector<double> &u){
  double penalty = 0.0;
  double c1 = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;
  double c4 = 0.0;

  c1 = HeuGoaldis(trajetory,goal);
  c2 = HeuCollitiondis(trajetory, gridmap);
  // c3 = HeuerroU(car_model, u);
  // std::cout << "c1 = " << c1 << std::endl;  
  // std::cout << "c2 = " << c2 << std::endl;  
  // std::cout << "c3 = " << c3 << std::endl;  
  penalty = 2.0*c1 + 1.0*c2 + 1.0*c3 + 1.0*c4;

  return penalty;
}

geometry_msgs::Twist Input2Twist(std::vector<double> input){
  geometry_msgs::Twist twist;
  twist.linear.x = input[0];

  twist.angular.z = input[1];
  std::cout << twist << std::endl;
  return twist;
}

void PubPath(const std::vector<scout_state> &restrajectory){
  if(restrajectory.size() != 0){
    path.poses.clear();
    for(auto &iter : restrajectory){
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = iter.x ;
      pose.pose.position.y = iter.y ;
      // std::cout << "pose.pose.position.x = " << pose.pose.position.x<< std::endl;
      // std::cout << "pose.pose.position.y = " << pose.pose.position.y<< std::endl;
      path.poses.push_back(pose);
    }
    // std::cout <<"path.poses[path.poses.size()-1].pose.position.x = " << path.poses[path.poses.size()-1].pose.position.x << std::endl;
    // std::cout <<"path.poses[path.poses.size()-1].pose.position.x = " << path.poses[path.poses.size()-1].pose.position.y << std::endl;
  }
  // std::cout << std::endl;
  pubPath.publish(path);
}

bool checkarrive(){
  
  int flag = 1;
  if(car_model.v < 0)
    flag = -1;
  
  double x = car_model.x + 0.4*flag*cos(car_model.yaw);
  double y = car_model.y + 0.4*flag*sin(car_model.yaw);


  double dx = x - goal.pose.position.x;
  double dy = y - goal.pose.position.y;

  double dis = sqrt(dx*dx + dy*dy);

  if(dis < 0.2){
    std::cout << "goal!"<< std::endl;
    goal_flag == 0;
    return true;
  }

  return false;
}

bool Getinput(geometry_msgs::Twist &Twist){
  if(car_model_flag == 0 || gridmap_flag == 0 || goal_flag == 0) return false;
  if(checkarrive()){
    std::vector<double> iu = {0.0,0.0};
    Twist = Input2Twist(iu);
    return true;
  }

  car_model_flag = 0;
  gridmap_flag = 0;
  // goal_flag = 0;

  //update dynamic windows;
  // std::cout << "gridmap.info.length_x = " <<gridmap.info.length_x << std::endl;
  // std::cout << "gridmap.info.length_y = " <<gridmap.info.length_y << std::endl;
  // std::cout << "resolution = " <<resolution<< std::endl;
  // std::cout << "gridmap.info.pose.position.x = " <<gridmap.info.pose.position.x<< std::endl;
  // std::cout << "gridmap.info.pose.position.y = " <<gridmap.info.pose.position.y<< std::endl;
  // std::cout << "car_model.x = " <<car_model.x<< std::endl;
  // std::cout << "car_model.y = " <<car_model.y<< std::endl;
  update(dw);
  // std::cout <<"update success " << std::endl;


  //search for mincost trajectory;
  double mincost = INFINITY;
  std::vector<double> resinput;
  std::vector<scout_state> restrajectory;

  // std::cout <<" start" << std::endl;
  // std::cout << "dw[0] "<<dw[0] <<std::endl;
  // std::cout << "dw[1] "<<dw[1] <<std::endl;
  // std::cout << "dw[2] "<<dw[2] <<std::endl;
  // std::cout << "dw[3] "<<dw[3] <<std::endl;

  for(double v = dw[0]; v <= dw[1]; v += v_step){
    for(double yaw_rate = dw[2]; yaw_rate <= dw[3]; yaw_rate+=yaw_step){
      std::vector<scout_state> trajectory;
      std::vector<double> input(2);
      input[0] = v;
      input[1] = yaw_rate;
      // std::cout <<" input[0] = " << input[0]<< std::endl;
      // std::cout <<" input[1] = " << input[1]<< std::endl;

      calTrajectory(trajectory, input[0], input[1]);
      // std::cout << "trajectory.size() Outside = "<<trajectory.size() << std::endl;
      // std::cout <<" get trajectory success " << std::endl;

      double cost = calpenalty(trajectory, input);
      // std::cout <<" cost = " << cost <<std::endl;
      // PubPath(trajectory);
    
      if(cost < mincost){
        resinput = input;
        mincost = cost;
        restrajectory = trajectory;
        // std::cout <<" resinput[0] " << resinput[0]<< std::endl;
        // std::cout <<" resinput[1] " << resinput[1]<< std::endl;
        // std::cout <<" mincost " << mincost << std::endl;

      }

    }
  }

  if(mincost > 90000.0){
    std::vector<double> iu;
    
    if(car_model.v > 0)
      iu = {0.0,0.0};
    else
      iu = {0.0,0.0};

    Twist = Input2Twist(iu);
    std::cout << " no path !"<< std::endl;
    return false;
  }
  
  std::cout <<" resinput[0] " << resinput[0]<< std::endl;
  std::cout <<" resinput[1] " << resinput[1]<< std::endl;
  std::cout <<" mincost " << mincost << std::endl;
  // std::cout <<" dynamic windows success " << std::endl;

  //path visualization 
  PubPath(restrajectory);

  //renew car_model
  car_model.v = resinput[0];
  car_model.omega = resinput[1];


  //return input u;
  Twist = Input2Twist(resinput);
  return true;
}

int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");
  setlocale(LC_CTYPE,"zh_CN.utf8");

  ros::init(argc,argv,"localPlanner_dwa");

  ros::NodeHandle nh;
  //messages creat
  // message_filters::Subscriber<grid_map_msgs::GridMap> subTerrainmapfilters(nh,"/MTraMap_local",1);
  // message_filters::Subscriber<nav_msgs::Odometry> subOdomfilters(nh,"/wamv/robot_localization/odometry/filtered",1);
  // typedef message_filters::sync_policies::ApproximateTime<grid_map_msgs::GridMap, nav_msgs::Odometry> SyncPolicy;
  
  // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), subTerrainmapfilters, subOdomfilters);
  // sync.registerCallback(boost::bind(&Odom2MapCB, _1, _2));

  ros::Subscriber subTerrainmap = nh.subscribe<grid_map_msgs::GridMap>("/MTraMap_local", 1, gridmapCB);
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/wamv/robot_localization/odometry/filtered", 1, odometryCB); 


  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 1, goalptCB);

  pubPath = nh.advertise<nav_msgs::Path> ("/path", 1);
  costmapPub = nh.advertise<nav_msgs::OccupancyGrid> ("/costmap",1);
  car_modelPub = nh.advertise<nav_msgs::Odometry>("/odomdomdom",1);

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 5);

  geometry_msgs::Twist cmdvel;
  ros::Rate rate(10);

  while(ros::ok()){
    ros::spinOnce();

    if(Getinput(cmdvel)){
      pubPath.publish(path);
    }
    pubSpeed.publish(cmdvel);
    rate.sleep();
  }
}