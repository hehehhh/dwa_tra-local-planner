#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>
#include <map>
#include <unordered_map>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

double resolution = 1.0; //[m]

string pathFolder = "/home/helll/ht_cmu_scout_dwa_ws/src/local_planner/paths";
const int pathNum = 343;
const int groupNum = 7;
double dirThre = 90.0;
int layernums = 8;
double layerstep = 2.0*M_PI/layernums;
double pathScale = 1.0;
bool twoWayDrive = true;
double goaldir = 1.0;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
double odomTime = 0;
// pose position and orientation
double vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
double vehicleX = 0, vehicleY = 0, vehicleZ = 0;
// goal 
double goalX = -100.0;
double goalY = 100.0;
double goalZ = 50;
//layers_search.
std::unordered_map<int, std::string> layers_search;
//core datas
grid_map_msgs::GridMap gridmap; bool gridmap_flag = 0;
//each grouID of path in pathNum。
int pathList[pathNum] = {0};
//angle about endpoints.
float endDirPathList[pathNum] = {0};
//pb List
vector<double> PbCost(36 * groupNum, 0.0);
//angdiff List
vector<double> DiffangCost(36 * groupNum, 0.0);
//followed path.
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
//all path needed to check.
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
//path which is reachable.
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
// published path.
nav_msgs::Path path;
nav_msgs::Path freepathnav;

vector<vector<double>> endPointList(pathNum, vector<double>(3));

double chechGoalbesind(){
  double goalangle = atan2(goalY - vehicleY, goalX - vehicleX);
  double yaw = vehicleYaw;
  while(goalangle < 0)
    goalangle += 2*M_PI;
  
  while(yaw < 0)
    yaw += 2*M_PI;
  // std::cout << "goalangle = "<< goalangle <<std::endl;
  // std::cout << "yaw = "<< yaw <<std::endl;

  double angle = abs(goalangle - yaw);

  if((angle > M_PI ? (2.0*M_PI - angle):angle)> (150.0 * M_PI/180.0)){
    // vehicleYaw = -vehicleYaw;
    return 1.0;
  }
  else{
    return 0.0;  
  }
  
}

void gridmapCB(const grid_map_msgs::GridMapConstPtr& msg)
{
  gridmap = *msg;
  resolution = gridmap.info.resolution;
  gridmap_flag = 1;
  // std::cout << "gridmap.data[layeri].data.size() = "<<  gridmap.data[8].data.size() <<std::endl;

}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = odom->header.stamp.toSec();

  //可以学习，这三行代码。
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  //IMU相对于车辆行进中心的距离xy 为 sensorOffsetX 和 sensorOffsetY。
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;

  goaldir = chechGoalbesind();
  // std::cout << "goaldir " << goaldir << endl;
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalZ = goal->point.z;
  cout << "goal accept" << endl;
}

void GoalCB2D(const geometry_msgs::PoseStamped::ConstPtr& goal){
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  goalZ = goal->pose.position.z;
  cout << "goal accept" << endl;
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      //根据文件格式进行读取操作，具有一定的规范。val用于存储返回的数据的数量，pointNum用于返回该值。而 第二个参数是对数据类型的指定。一般来说文件的读取是根据空格来急性分割。
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = atan2(endY, endX);

      endPointList[pathID][0] = endX;
      endPointList[pathID][1] = endY;
      endPointList[pathID][2] = endZ;
    }
  }
  
  fclose(filePtr);
}

void initsearchmap(std::unordered_map<int, std::string> &searchmap){
  for(int i = 0; i < layernums; ++i){
    searchmap[i] = "tra_label_" + to_string(i) + "_conf50";
    // cout << searchmap.at(i)<< endl;
  }
}
void remain2Pi(double &Angle){
  while(Angle < 0.0)
    Angle += 2.0*M_PI;
  
  while(Angle > 2.0*M_PI)
    Angle -= 2.0*M_PI;
}

void CalgoalDiff(int ID, double rotAng){
  int groudID = pathList[ID % pathNum];
  int angleID = int(ID / pathNum);

  int chosedPath= groupNum * angleID + groudID;

  double goalAngle = atan2(goalY - vehicleY, goalX - vehicleX);
  // cout << "goalangle = " << goalAngle <<  endl;
  double pathendAngle = endDirPathList[ID % pathNum] + rotAng + vehicleYaw + goaldir * M_PI;
  // cout << "pathendAngle = " << pathendAngle <<  endl;
  remain2Pi(goalAngle);
  remain2Pi(pathendAngle);
  DiffangCost[chosedPath] += fabs(goalAngle - pathendAngle);
  // cout << "DiffangCost[chosedPath] = " << DiffangCost[chosedPath] <<  endl;
  // cout << "chosedPath = " << chosedPath <<  endl;

}

std::vector<int> Coord2index(double ptx, double pty){

  int indexptx = (ptx - (gridmap.info.pose.position.x - gridmap.info.length_x/2.0))/resolution;
  int indexpty = (pty - (gridmap.info.pose.position.y - gridmap.info.length_y/2.0))/resolution;
  // std::cout << "indexptx =  " << indexptx <<std::endl;
  // std::cout << "indexpty  " << indexpty <<std::endl;

  std::vector<int> index(2);

  index[0] = indexptx;
  index[1] = indexpty;
  // std::cout << "index[0]  "<< index[0] <<std::endl;
  // std::cout << "index[1]  "<< index[1] <<std::endl;
  return index;

}

std::vector<int> Coord2indexlocal(double localx, double localy){
  std::vector<int> ret(2);
  double relativeangle = vehicleYaw;
  double l = sqrt(localx * localx + localy * localy);
  double x = localx * cos(relativeangle) - localy * sin(relativeangle);
  double y = localy * cos(relativeangle) + localx * sin(relativeangle) ;
  ret[0] = (x + gridmap.info.length_x/2.0) / resolution;
  ret[1] = (y + gridmap.info.length_y/2.0) / resolution;
  return ret;
}

pcl::PointXYZI PointWithrelative(const pcl::PointXYZI &point, const double &rotAng){
  
  pcl::PointXYZI res;
  res = point;
  res.x = pathScale * (cos(goaldir *M_PI + rotAng) * point.x - sin(goaldir *M_PI + rotAng) * point.y);
  res.y = pathScale * (sin(goaldir *M_PI + rotAng) * point.x + cos(goaldir *M_PI + rotAng) * point.y);
  vector<int> pointindex = Coord2indexlocal(res.x, res.y);
  int INX = pointindex[1] * gridmap.info.length_x/resolution + pointindex[0];
  INX = gridmap.data[0].data.size() - INX - 1;

  // res.z =  double (gridmap.data[0].data[INX]) - vehicleZ;
  res.z = 0.0;
  res.intensity = 0.5;
  
  return res;
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

bool Checktravesibility(const grid_map_msgs::GridMap& map, int indexX,double indexY,double yawangle){
  // key to find graph
  while(yawangle < 0.0)
    yawangle += 2.0*M_PI;
  while(yawangle > 2.0*M_PI)
    yawangle -= 2.0*M_PI;

  int laykey = (yawangle)/layerstep;
  // std::cout << "laykey = "<< laykey <<std::endl;
  int layeri; // index of graph
  int INX; // index of point
  //get cost of gridmap.
  layeri = Getlayeri(gridmap, layers_search.at(laykey));
  int indexGridMap = map.data[layeri].data.size();
  INX = indexY * gridmap.info.length_x/resolution + indexX;
  INX = indexGridMap- INX - 1;
  // layeri = 2; // 可以解开这个注释查看只有2图层的检测效果。
  double cost = map.data[layeri].data[INX];
  // std::cout << "cost = "<< cost <<std::endl;

  if(cost != 1.0){
    // std::cout << "layeri = "<< layeri <<std::endl;
    // std::cout << "laykey = "<< laykey <<std::endl;
    // std::cout << "indexX = "<< indexX <<std::endl;
    // std::cout << "indexY = "<< indexY <<std::endl; 
    // std::cout << "layers_search.at(layerkey) = " << layers_search.at(laykey)<< std::endl;
    return false;
  }
  // std::cout << "layeri = "<< layeri <<std::endl;
  // std::cout << "laykey = "<< laykey <<std::endl;
  // std::cout << "indexX = "<< indexX <<std::endl;
  // std::cout << "indexY = "<< indexY <<std::endl; 
  // std::cout << "layers_search.at(layerkey) = " << layers_search.at(laykey)<< std::endl;
  // std::cout << "cost = "<< cost <<std::endl;

  return true;
}


void Checkcollition(int ID, double rotAng){
  int selectedID = ID % pathNum;

  int pathlength = paths[selectedID]->points.size() - 1;
  for(int i = 0; i < pathlength; i++){
    double dx = paths[selectedID]->points[i + 1].x - paths[selectedID]->points[i].x;
    double dy = paths[selectedID]->points[i + 1].y - paths[selectedID]->points[i].y;
    //absoluted angle.
    double yawangle = atan2(dy,dx) + rotAng + vehicleYaw;
    // cout << "yawangle = "<< yawangle * 180.0/M_PI<<endl;
    double localX = pathScale * (cos(goaldir * M_PI + rotAng) * paths[selectedID]->points[i].x - sin(goaldir * M_PI + rotAng) * paths[selectedID]->points[i].y);
    double localY = pathScale * (sin(goaldir * M_PI + rotAng) * paths[selectedID]->points[i].x + cos(goaldir * M_PI + rotAng) * paths[selectedID]->points[i].y);
    // cout << "localX = "<< localX << endl;
    // cout << "localY = "<< localY << endl;

    vector<int> lobalIndex = Coord2indexlocal(localX, localY);
    // cout << "lobalIndex0 = "<< lobalIndex[0]<<endl;
    // cout << "lobalIndex1 = "<< lobalIndex[1] <<endl;

    if(!Checktravesibility(gridmap, lobalIndex[0], lobalIndex[1], yawangle)){
      // cout << "selectedID = "<< selectedID << endl;
      return;
    }
    //free points
    pcl::PointXYZI point;
    point = PointWithrelative(paths[ID % pathNum]->points[i], rotAng);
    freePaths->push_back(point);
  }
  // cout << "selectedID = "<< selectedID << endl;
  int groudID = pathList[ID % pathNum];
  int angleID = int(ID / pathNum);

  int chosedPath= groupNum * angleID + groudID;

  PbCost[chosedPath] += 1;
  // cout  << "PbCost[chosedPath] = " << PbCost[chosedPath] << endl;
  // cout  << "chosedPath = " << chosedPath << endl;
  // ros::Rate slee(100);
  // slee.sleep();

  // //all reachable path
  // pcl::PointXYZI point;

  // int pathilength = paths[ID % pathNum]->size();
  // for(int j = 0; j < pathilength; ++j){
  //   point = PointWithrelative(paths[ID % pathNum]->points[j], rotAng);
  //   freePaths->push_back(point);

  // }
}

void Calpernalty(int i,double rotAng){
  //update DiffangCost 
  CalgoalDiff(i, rotAng);
  Checkcollition(i, rotAng);

}

nav_msgs::Path convertPath(int selectedID, double selectedrotAng){
  selectedID = pathList[selectedID % pathNum];
  nav_msgs::Path res;

  int selectedPathLength = startPaths[selectedID]->points.size();
  // cout << " selectedID = " << selectedID << endl;
  // cout << " selectedrotAng = " << selectedrotAng << endl;
  res.poses.resize(selectedPathLength);
  for(int i = 0; i < selectedPathLength; ++i){
    float x = startPaths[selectedID]->points[i].x;
    float y = startPaths[selectedID]->points[i].y;
    float z = startPaths[selectedID]->points[i].z;

    res.poses[i].pose.position.x = (cos(goaldir * M_PI + selectedrotAng) * x - sin(goaldir * M_PI + selectedrotAng) * y);
    res.poses[i].pose.position.y = (sin(goaldir * M_PI + selectedrotAng) * x + cos(goaldir * M_PI + selectedrotAng) * y);
    res.poses[i].pose.position.z = z;
  }
  res.header.stamp = ros::Time().fromSec(odomTime);
  res.header.frame_id = "base_link";

  return res;
}

bool checkGoal(){
  double dx = vehicleX - goalX;
  double dy = vehicleY - goalY;
  double dis = sqrt(dx*dx + dy*dy);

  if(dis < 0.5)
    return true;
  else
    return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  //subcriber 
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/wamv/robot_localization/odometry/filtered", 5, odometryHandler);
  
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);
  ros::Subscriber subGoaltest = nh.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 5, GoalCB2D);

  ros::Subscriber subTramap = nh.subscribe<grid_map_msgs::GridMap>("/MTraMap_local", 5, gridmapCB);
  //publish
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2> ("/free_paths", 2);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);
  ros::Publisher pubfreePath = nh.advertise<nav_msgs::Path> ("/freepath_nav", 5);

  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  //feasible path
  readStartPaths();
  //prediction path
  readPaths();
  //endpoint of each path
  readPathList();
  //initial la(layers_search);
  initsearchmap(layers_search);

  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    sensor_msgs::PointCloud2 totalpath2;
    freePaths->clear();

    vector<double> emptyVector(36 * groupNum, 0.0);
    PbCost = emptyVector;
    DiffangCost = emptyVector;

    if(gridmap_flag == 1){
      gridmap_flag = 0;


      for(int i = 0; i < 36 * pathNum; ++i){
        int rotDir = int (i / pathNum);
        double rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
        // rotAng = M_PI/2.0;
        // if (fabs(10.0 * rotDir - 180.0) > dirThre  || ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre))){
        //   continue;
        // }
        // ignore unreachable path
        if (fabs(10.0 * rotDir - 180.0) > dirThre){
          continue;
        }
        //selected mincost path
        Calpernalty(i, rotAng);
      }

      int selectedID = -1;
      double maxcost = 0.0;
      int rotDir;
      double selectedrotAng;
      for(int i = 0; i < 36 * pathNum; ++i){
        rotDir = int (i / pathNum);

        if (fabs(10.0 * rotDir - 180.0) > dirThre){
          continue;
        }
        int groudID = pathList[i % pathNum];
        int angleID = int(i / pathNum);
        int chosedPath= groupNum * angleID + groudID;

        double k1 = 0.02;
        double k2 = 0.01;
        double cost = k1 * PbCost[chosedPath] + (1.0 - k2 * DiffangCost[chosedPath]);

        // double cost = PbCost[chosedPath];
        // double cost = (1.0 - k2 * DiffangCost[chosedPath]);
        // cout << "cost = "<< cost << endl;
        // cout << "PbCost[i] = "<< PbCost[chosedPath] << endl;

        if(cost > maxcost){
          maxcost = cost;

          selectedrotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
          selectedID = i;

          // cout << "maxcost = "<< maxcost << endl;
          // cout << "selectedrotAng= "<< selectedrotAng << endl;
          // cout << "PbCost[i]= "<< PbCost[chosedPath] << endl;
          // cout << "DiffangCost[i]= "<< DiffangCost[chosedPath] << endl;
        }
      }

      //publish free path
      pcl::toROSMsg(*freePaths,totalpath2);
      totalpath2.header.stamp = ros::Time().fromSec(odomTime);
      totalpath2.header.frame_id = "base_link";
      pubFreePaths.publish(totalpath2);
      std::cout << "freepath pub " << totalpath2.data.size() << endl;
      //goal
      if(checkGoal()){
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "base_link";
        pubPath.publish(path);
        continue;
      }
      //publish selected path
      path = convertPath(selectedID, selectedrotAng);
      pubPath.publish(path);

    
  }

}

    rate.sleep();
  }
  
