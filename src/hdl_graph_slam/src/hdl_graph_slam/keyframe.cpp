#include <hdl_graph_slam/keyframe.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <yaml.h>

namespace hdl_graph_slam {

KeyFrame::KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud)
  : stamp(stamp),
    odom(odom),
    accum_distance(accum_distance),
    cloud(cloud),
    node(nullptr)
{}

KeyFrame::~KeyFrame() {

}

/*  
void KeyFrame::dump(const std::string& directory) {
  if(!boost::filesystem::is_directory(directory)) {
    boost::filesystem::create_directory(directory);
  }

  std::ofstream ofs(directory + "/data");
  ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";

  ofs << "estimate\n";
  ofs << node->estimate().matrix() << "\n";

  ofs << "odom\n";
  ofs << odom.matrix() << "\n";

  ofs << "accum_distance " << accum_distance << "\n";

  if(floor_coeffs) {
    ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
  }

  if(node) {
    ofs << "id " << node->id() << "\n";
  }

  pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *cloud);

}
*/

void KeyFrame::dump(const std::string& directory) {
  if(!boost::filesystem::is_directory(directory)) {
    boost::filesystem::create_directory(directory);
  }

  std::ofstream ofs(directory + "/data.yaml");
  
  YAML::Node data;
  
  auto estimateMatrix = node->estimate().matrix();
  std::vector<double> estimateVector(estimateMatrix.data(), estimateMatrix.data()+estimateMatrix.size());
  data["estimate"] = estimateVector;
  
  auto odomMatrix = odom.matrix();
  std::vector<double> odomVector(odomMatrix.data(), odomMatrix.data()+odomMatrix.size());
  data["odom"] = odomVector;

  data["accum_distance"] = accum_distance ;

  if(node) 
    data["id"] = node->id() ;
    
  ofs << data ;
  ofs.close();

  pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *cloud);
}


KeyFrameSnapshot::KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud)
  : pose(pose),
    cloud(cloud)
{}

KeyFrameSnapshot::KeyFrameSnapshot(const KeyFrame::Ptr& key)
  : pose(key->node->estimate()),
    cloud(key->cloud)
{}


KeyFrameSnapshot::~KeyFrameSnapshot() {

}

}
