#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Merge {
public:
  Merge(void);

  bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);
  void laserCloudFullResHandler(
      const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg);

  auto &laserCloud() { return _laserCloud; }
  void spin(void);
  void process(void);
  void reset();
  bool hasNewData();

  void mergeCloud(pcl::PointCloud<pcl::PointXYZI> &cloud_merged);
  void savePcdFile(const std::string &c_output_file_path,
                   const pcl::PointCloud<pcl::PointXYZI> &c_cloud_output);

  void MergePcd(const std::string &c_file_names);

private:
  ros::Subscriber
      _subLaserCloudFullRes; ///< full resolution cloud message subscriber
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud; ///< full resolution cloud

  ros::Time _timeLaserCloudFullRes; ///< time of current full resolution cloud
  bool _newLaserCloudFullRes; ///< flag if a new full resolution cloud has been
                              ///< received

  bool _first;
  bool _toggle;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> _clouds;
};

Merge::Merge(void)
    : _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()), _first(false),
      _toggle(false) {}

bool Merge::hasNewData() { return _newLaserCloudFullRes; }

bool Merge::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {
  _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_cloud_registered", 2, &Merge::laserCloudFullResHandler, this);
  return true;
}

void Merge::laserCloudFullResHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

  laserCloud()->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *laserCloud());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);
  _newLaserCloudFullRes = true;
}

void Merge::mergeCloud(pcl::PointCloud<pcl::PointXYZI> &cloud_merged) {
  for (auto &cloud : _clouds) {
    if (&cloud - &_clouds[0] == 0)
      cloud_merged = cloud;
    cloud_merged += cloud;
  }
  return;
}
void Merge::MergePcd(const std::string &c_file_names) {
#if 1
  pcl::PointCloud<pcl::PointXYZI> c;
  {
    if (pcl::io::loadPCDFile(c_file_names, c) == -1) {
      std::printf("shutdown...\n");
      exit(0);
    }
    _clouds.push_back(c);
  }
#endif
}

void Merge::savePcdFile(const std::string &c_output_file_path,
                        const pcl::PointCloud<pcl::PointXYZI> &c_cloud_output) {
  try {
    // pcl::io::savePCDFileBinary(c_output_file_path, c_cloud_output);
    pcl::io::savePCDFileASCII(c_output_file_path, c_cloud_output);
  } catch (const pcl::IOException &e) {
    std::cerr << e.what() << '\n';
    exit(0);
  }
  std::printf("save merged clouds to %s\n", c_output_file_path.c_str());
  return;
}
void Merge::reset() { _newLaserCloudFullRes = false; }

void Merge::process() {
  if (!hasNewData()) {
    return; // waiting for new data to arrive.
  }
  // ROS_INFO("cyc: %s", __FUNCTION__);

  _clouds.push_back(*_laserCloud);
#if 1
  if (!_first) {
    pcl::PointCloud<pcl::PointXYZI> cloud_merged;
    mergeCloud(cloud_merged);
    std::string path = "/tmp/pcd/a.pcd";
    savePcdFile(path, cloud_merged);
    _first = true;
  } else {
    pcl::PointCloud<pcl::PointXYZI> cloud_merged;

    if (!_toggle) {
      std::string in_path = "/tmp/pcd/a.pcd";
      std::string path = "/tmp/pcd/b.pcd";

      MergePcd(in_path);
      mergeCloud(cloud_merged);
      savePcdFile(path, cloud_merged);
    } else {
      std::string in_path = "/tmp/pcd/b.pcd";
      std::string path = "/tmp/pcd/a.pcd";

      MergePcd(in_path);
      mergeCloud(cloud_merged);
      savePcdFile(path, cloud_merged);
    }
    _toggle ^= 1;
  }
#endif
  _clouds.clear();
  reset(); // reset flags, etc.
}

void Merge::spin() {
  ros::Rate rate(1);
  bool status = ros::ok();

  // loop until shutdown
  while (status) {
    ros::spinOnce();

    // try processing new data
    process();

    status = ros::ok();
    rate.sleep();
  }
}

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "merge");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  Merge m;
  if (m.setup(node, privateNode)) {
    m.spin();
  }

  return 0;
}
