#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include "tf/transform_datatypes.h"

#include "serial.h"


#include <string>
#include <fstream>



enum NODE_STATE{
  NONE, OPEN, CLOSED
};

enum MAP_STATE{
  UNKNOWN = -1,
  FLAT = 0,
  WALL = 100,
  INFLATE_WALL = 200
};



// 2次元座標保持クラス
class PointInt
{
public:
  PointInt():x(0),y(0){}
  int x,y;
};


// Astarでの各マスのステータス保持クラス
class CoronNode
{
public:
  // parent用にデフォルトコンストラクタ
  CoronNode()
  {
    c_ = -1;
    status_ = NONE;
  };

  CoronNode(int x, int y, int sx, int sy, int gx, int gy)
  {
    x_ = x;
    y_ = y;
    c_ = abs(x - sx) + abs(y - sy); // 実コスト計算
    h_ = abs(x - gx) + abs(y - gy); //
    status_ = NONE;
  };

  ~CoronNode() {};

  void addParent(int px, int py) { px_ = px; py_ = py; };
  int getCost() { return c_ + h_; };


  int x_, y_;
  int px_, py_; // 親の座標
  int status_;
  int c_; // 実コスト　スタート地点からの距離
  int h_; // 推定コスト　このノードからゴール地点までの

//  CoronNode parent_;
private:

};


class CoronAstar
{
public:
  CoronAstar();
  ~CoronAstar();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  void initDirection();

  std::vector<std::pair<int,int> > dir;

  geometry_msgs::Point calcGrid( double pos_x, double pos_y );

  // tfとmapでのoffsetを計算（座標変換）
  PointInt transMap2TF( int pos_x, int pos_y );
  PointInt transTF2Map( int pos_x, int pos_y );

  geometry_msgs::Point calcGrid2Pos( int pos_x, int pos_y );
  PointInt calcPos2Grid( double pos_x, double pos_y );

  geometry_msgs::PoseStamped createPoseStampe( int map_x, int map_y, ros::Time rt );

  ros::Publisher path_pub_;
  ros::Publisher map_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Timer timer_;

  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped goal_;
  nav_msgs::Path path_;

  nav_msgs::OccupancyGrid map_, grid_; // -1:unkown 100:wall 0:
  geometry_msgs::Point cell_;

  PointInt map_offset_;

  bool ready_map_, ready_odom_, ready_goal_;
  bool pub_flg_;  // debug
  double map_resolution_ ;

  int magnification_; // 壁をどれくらい膨張させるかの倍率

  PointInt st_; // スタート地点
  PointInt go_; // ゴール地点


  std::vector<CoronNode> open_nodes_;
  void openNode(CoronNode base);
  void publishPath(int base_index);
  CoronNode getNode(int x, int y);

  void setStartAndGoal();
  bool isReadyAstar();
  int searchMinCostNode();
  void executeAstar();

  nav_msgs::OccupancyGrid expansionWall( const nav_msgs::OccupancyGrid in);
  std::vector<std::pair<int,int> > createDirection( int magni );
};


// constructor
CoronAstar::CoronAstar():
  ready_map_(false)
  ,ready_odom_(false)
  ,ready_goal_(false)
  ,pub_flg_(true)
  ,magnification_(5)
{
  std::cout << "CoronAstar() called!!" << std::endl;
  ros::NodeHandle nh;

  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &CoronAstar::goalCallback, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &CoronAstar::odomCallback, this);
  map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &CoronAstar::mapCallback, this);

  path_pub_ = nh.advertise<nav_msgs::Path>("coron/path", 10, true);
  map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("coron/map", 10, true);  // debug

  // Astar計算を行うタイマー　とりあえず３秒おき
  timer_ = nh.createTimer(ros::Duration(3.0), boost::bind(&CoronAstar::executeAstar, this));


  // ready to Astar
  initDirection();
}


void CoronAstar::publishPath( int goal_index )
{
  CoronNode tmp = open_nodes_[goal_index];

  path_.poses.clear();
  path_.header.frame_id = "map";
//  path_.header.frame_id = "coron_path";
  path_.header.stamp = ros::Time::now();

  // ゴール地点を追加
  std::vector<geometry_msgs::PoseStamped>::iterator it = path_.poses.begin();
  path_.poses.insert(it, createPoseStampe(tmp.x_,tmp.y_,path_.header.stamp));  // 先頭へ追加


  // スタート地点までさかのぼる
  while (!(tmp.x_== st_.x && tmp.y_ == st_.y ))
  {
//    std::cout << "( " << tmp.x_ << ", " << tmp.y_ << " )" << std::endl;

    // 次のノードを探す
    for (unsigned int i = 0; i < open_nodes_.size(); i++)
    {
      if (open_nodes_[i].status_ == OPEN) continue;

      // 親ノード？
      if (open_nodes_[i].x_ == tmp.px_ && open_nodes_[i].y_ == tmp.py_)
      {
        tmp = open_nodes_[i];

        // 細かいパスには追従できないので、間引く
        if( i % 2 == 0 )
        {
          // パスを格納
          it = path_.poses.begin();
          path_.poses.insert(it, createPoseStampe(tmp.x_,tmp.y_,path_.header.stamp));  // 先頭へ追加
        }
        break;
      }
    }
  }
 // std::cout << "( " << tmp.x_ << ", " << tmp.y_ << " )" << std::endl;

  // debug
//  for(int i = 0; i < path_.poses.size(); i++ )
//  {
//    std::cout << "( " << path_.poses[i].pose.position.x << ", " << path_.poses[i].pose.position.y << " )" << std::endl;
//    //std::cout << path_.poses[i] << std::cout;
//  }

  if( pub_flg_ )
  {
//    pub_flg_ = false;
    path_pub_.publish(path_);
  }
}


geometry_msgs::PoseStamped CoronAstar::createPoseStampe( int map_x, int map_y, ros::Time rt )
{
  geometry_msgs::PoseStamped tmp_pose;
  tmp_pose.header.frame_id = "tmp_pose";
  tmp_pose.header.stamp = rt;

  PointInt p = transMap2TF(map_x,map_y);          // map座標系からtf（world）座標系へ変換
  geometry_msgs::Point pos = calcGrid2Pos(p.x,p.y); // grid表現をpos表現へ変換
  tmp_pose.pose.position.x = pos.x;
  tmp_pose.pose.position.y = pos.y;
  tmp_pose.pose.orientation.x = 0;
  tmp_pose.pose.orientation.y = 0;
  tmp_pose.pose.orientation.z = 0;
  tmp_pose.pose.orientation.w = 1;

  return tmp_pose;
}


bool CoronAstar::isReadyAstar()
{
  if( ready_map_ && ready_odom_ && ready_goal_ ) return true;
  return false;
}


void CoronAstar::executeAstar()
{
  if( isReadyAstar() )
  {
    setStartAndGoal();

    open_nodes_.clear();

    // スタート地点をオープンする
    CoronNode tmp_node(st_.x, st_.y, st_.x, st_.y, go_.x , go_.y);
    tmp_node.status_ = OPEN;
    open_nodes_.push_back(tmp_node);

    // 周囲のノードオープンしてオープンノードに加える
    openNode(open_nodes_[0]);
    open_nodes_[0].status_ = CLOSED;

    bool is_end = false;
    int cnt = 0;  // debug 計算回数カウンター
    while (!is_end)
    {
      int base_index = searchMinCostNode();

      // オープンノードがまだあるかチェック
      if(base_index == -1)
      {
        is_end = true;
        //std::cout << "sorry. could not find path." << std::endl;
      }
      // ゴールチェック
      else if (open_nodes_[base_index].x_ == go_.x
          && open_nodes_[base_index].y_ == go_.y )
      {
        is_end = true;
        //std::cout << "find path!!" << std::endl;
        publishPath(base_index);
      }
      // ゴールしてなかったから探索
      else
      {
        //std::cout << cnt << ":searching!!" << std::endl;
        cnt++;
        openNode(open_nodes_[base_index]);
        open_nodes_[base_index].status_ = CLOSED;
      }
    }

//  ros::Time end_time = ros::Time::now();
//  double calc_time = (end_time.sec + end_time.nsec/1e9)-(start_time.sec + start_time.nsec/1e9);
//  printf("Astar calc time: %.10f\n", calc_time);
  }
  else
  {
    //printf("Not Ready to Astar\n");
  }
}


// TODO:検索失敗の場合の処理も書く
CoronNode CoronAstar::getNode(int x, int y )
{
  CoronNode tmp;

  // 次のノードを探す
  for (unsigned int i = 0; i < open_nodes_.size(); i++)
  {
    // 親ノード？
    if (open_nodes_[i].x_ == x && open_nodes_[i].y_ == y)
    {
      tmp = open_nodes_[i];
      break;
    }
  }
  return tmp;
}


void CoronAstar::openNode(CoronNode base)
{
  //std::cout << "base: " << base.x_ << ", "<< base.y_ << std::endl;

  for (unsigned int i = 0; i < dir.size(); i++ )
  {
    int x = base.x_ + dir[i].first;
    int y = base.y_ + dir[i].second;

    // マップへアクセス
    // 範囲チェック
    if ( x < 0 || x >= grid_.info.width
      || y < 0 || y >= grid_.info.height )
      continue;

    // すでにあけられてないかチェック
    if (getNode(x, y).status_ == NONE)
    {
      // 対象のノードが更地でかつまだ一回もあけられてないノードなら追加
      if ( grid_.data[x + y * grid_.info.width ] == FLAT )
      {
        CoronNode tmp(x, y, st_.x , st_.y, go_.x , go_.y);
        tmp.status_ = OPEN;
        tmp.addParent(base.x_, base.y_);
        open_nodes_.push_back(tmp);

        //std::cout << "open node: "  << tmp.x_ << ", "<< tmp.y_ << std::endl;
      }
    }
  }
//  std::cout << std::endl;
}


void CoronAstar::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
 // std::cout << *msg << std::endl;
  odom_ = *msg;
  ready_odom_ = true;
}


// 周囲八方向へアクセスする為の配列
void CoronAstar::initDirection()
{
  dir.push_back(std::make_pair(-1,-1));
  dir.push_back(std::make_pair( 0,-1 ));
  dir.push_back(std::make_pair( 1,-1 ));
  dir.push_back(std::make_pair(-1, 0 ));
  dir.push_back(std::make_pair( 1, 0));
  dir.push_back(std::make_pair(-1, 1 ));
  dir.push_back(std::make_pair( 0, 1 ));
  dir.push_back(std::make_pair( 1, 1 ));
}


// マスの周囲へ座標を作る
// e.g. magniが2なら周囲3*3-1=8マス 3なら周囲5*5-1=24マス

// 引数は倍率
std::vector<std::pair<int,int> > CoronAstar::createDirection( int magni )
{
  std::vector<std::pair<int,int> > res;

  magni--;

  for( int i = -magni; i <= magni; i++ )
  {
    for( int j = -magni; j <= magni; j++ )
    {
      if( i == 0 && j == 0 ) continue;  // 中心は除外

      res.push_back(std::make_pair(j,i));
    }
  }

  // debug
//  for( int i = 0; i < res.size(); i++ )  std::cout << res[i].first << ", "<< res[i].second << std::endl;

  return res;
}


// パスが壁に近づき過ぎないように、仮想壁を作成する
nav_msgs::OccupancyGrid CoronAstar::expansionWall( const nav_msgs::OccupancyGrid in )
{
  nav_msgs::OccupancyGrid out = in;

  std::vector<std::pair<int,int> > dir2 = createDirection(magnification_);

  for( int i = 0; i < out.info.height; i++ )
  {
    for( int j = 0; j < out.info.width; j++ )
    {
      // 今回の場所が壁か？
      if( out.data[j + i*out.info.width] == WALL )
      {
        // 周囲を調べてアンノウンか平地ならINFLATE_WALLを入れる
        for (unsigned int k = 0; k < dir2.size(); k++ )
        {
          int x = j + dir2[k].first;
          int y = i + dir2[k].second;

          // マップへアクセス
          // 範囲チェック
          if ( x < 0 || x >= out.info.width
            || y < 0 || y >= out.info.height )
            continue;

          if( out.data[x + y*out.info.width] == FLAT
              || out.data[x + y*out.info.width] == UNKNOWN )
          {
            out.data[x + y*out.info.width] = INFLATE_WALL;
          }
        }
      }
    }
  }

  // view result
  map_pub_.publish(out);
  return out;
}


void CoronAstar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  // debug
  map_sub_.shutdown();
  ready_map_ = true;

  //  std::cout << *msg << std::endl;
  map_ = *msg;

  // 壁膨張
  grid_ = expansionWall(map_);

  // 誤差切り捨て
  map_resolution_ = floor(grid_.info.resolution * 100.0) / 100.0;
  //map_resolution_ = (double)grid_.info.resolution;

//  printf("grid_.info.resolution: %f\n",grid_.info.resolution);

  map_offset_.x = (int)ceil(fabs(grid_.info.origin.position.x / map_resolution_));
  map_offset_.y = (int)ceil(fabs(grid_.info.origin.position.y / map_resolution_));


//  std::cout << map_offset_ << std::endl;
//  printf("map_offset: %d %d \n", map_offset_.x,map_offset_.y);
}


void CoronAstar::setStartAndGoal()
{
  // 座標をグリッドに変換
  PointInt pp = calcPos2Grid(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  st_ = transTF2Map(pp.x,pp.y); // ワールド座標系からマップ座標系へ変換
  pp = calcPos2Grid(goal_.pose.position.x, goal_.pose.position.y);
  go_ = transTF2Map(pp.x,pp.y);

  printf("odom_: %f, %f\n", odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  printf("goal_: %f, %f\n", goal_.pose.position.x, goal_.pose.position.y);
  printf("st_: %d, %d\n", st_.x,st_.y);
  printf("go_: %d, %d\n", go_.x,go_.y);
  printf("\n");
}



// -1で探索失敗
int CoronAstar::searchMinCostNode()
{
  // 探索ノードの決定
  int min_cost = 99999;
  int base_index = -1;

  for (unsigned int i = 0; i < open_nodes_.size(); i++ )
  {
    if (open_nodes_[i].status_ == CLOSED) continue;

    if (min_cost > open_nodes_[i].getCost())
    {
      min_cost = open_nodes_[i].getCost();
      base_index = i;
    }
  }
  return base_index;
}


void CoronAstar::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//  std::cout << *msg << std::endl;
  goal_ = *msg;
  ready_goal_ = true;
}


CoronAstar::~CoronAstar()
{
  std::cout << "~CoronAstar() called!" << std::endl;
}


// マップ座標系からワールド座標系へ変換
PointInt CoronAstar::transMap2TF( int pos_x, int pos_y )
{
  PointInt res;
  res.x = pos_x - map_offset_.x;
  res.y = pos_y - map_offset_.y;
  return res;
}


// astarの計算前はこれ呼んで　グリッド化する
// ワールド座標系からマップ座標系へ変換
PointInt CoronAstar::transTF2Map( int pos_x, int pos_y )
{
  PointInt res;
  res.x = pos_x + map_offset_.x;
  res.y = pos_y + map_offset_.y;
  return res;
}


// グリッドから座標へ変換
geometry_msgs::Point CoronAstar::calcGrid2Pos( int pos_x, int pos_y )
{
  geometry_msgs::Point res;
  res.x = (double)pos_x * map_resolution_ + map_resolution_/2;
  res.y = (double)pos_y * map_resolution_ + map_resolution_/2;
  return res;
}


// 座標からグリッドの形へ変換
PointInt CoronAstar::calcPos2Grid( double pos_x, double pos_y )
{
  PointInt res;
  //printf("map_resolution:_: %f\n",map_resolution_);
  res.x = floor(pos_x / map_resolution_);
  res.y = floor(pos_y / map_resolution_);
  return res;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "coron_astar");
  CoronAstar ca;

  ros::spin();
}
