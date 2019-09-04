/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 23/04/2018
 *
 */

// Include guard
#ifndef dbscan_H
#define dbscan_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <queue>
#include <helper/tools.h>
#include <helper/ObjectArray.h>
#include <tf/transform_listener.h>
#include <numeric>

// Namespaces
namespace detection{

using namespace sensor_msgs;
using namespace helper;

struct ObjectAttributes{
	float side_min;
	float side_max;
	float height_min;
	float height_max;
	float semantic_min;
};

struct Parameter{

	float grid_range_max;
	float grid_cell_size;

	ObjectAttributes boat_spawn;
	ObjectAttributes boat_update;
	ObjectAttributes scu_spawn;
	ObjectAttributes scu_update;
};

// Semantic information of a cluster
struct Semantic{

	std::map<int,int> classes;
	float confidence;
	int id;
	std::string name;
	int diff_counter;
};

// Geometric information of a cluster
struct Geometric{

	float x;
	float y;
	float z;
	float width;
	float length;
	float height;
	float orientation;

	std::vector<cv::Point> cells;
	int num_cells;
};

// Information of a cluster
struct Cluster{

	// ID
	int id;

	// Kernel size
	int kernel;

	// Semantic information
	Semantic semantic;

	// Geometric information
	Geometric geometric;

	// Misc stuff
	cv::RotatedRect rect;
	cv::Scalar color;
	bool is_new_track;
	bool has_adjacent_free_space;

	// int * class_num;
	std::vector<int> class_num;

};

class DbScan{

public:

	// Default constructor
	DbScan(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~DbScan();

	virtual void process(const Image::ConstPtr & image_detection_grid);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	std::vector<Cluster> clusters_;
	ObjectArray object_array_;
	int number_of_clusters_;
	int time_frame_;
	Parameter params_;
	Tools * tools_;
	tf::TransformListener listener_;

	// Subscriber
	ros::Subscriber image_detection_grid_sub_;

	// Publisher
	ros::Publisher object_array_pub_;

	// Class functions
	void runDbScan(cv::Mat grid);
	void filterClusters(const cv::Mat grid);
	void fillObjectList();
	void addObject(const Cluster & c);
	bool spawnScu(const Cluster & c);
	bool spawnBoat(const Cluster & c);
	bool updateScu(const Cluster & c);
	bool updateBoat(const Cluster & c);
	bool isValidSemantic(const int semantic_class);
	void printCluster(const Cluster & c);
	void printObject(const Object & o);

};

} // namespace detection

#endif // dbscan_H
