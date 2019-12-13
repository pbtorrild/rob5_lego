#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>

#include <cv_bridge/cv_bridge.h>

#include <zero_point_cal/GetMarker.h>

class markers{
private:
  // camera calibration, is found using http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  // and the topic /camera/color/came_info
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 614.7054443359375, 0.0, 315.4218444824219, 0.0, 614.9154052734375, 243.67068481445312, 0.0, 0.0, 1.0);
  cv::Mat distCoeffs= (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  double to_degree = 180/3.14159265359;
  int num_markers=14;
  int marker_bits_size=4; //X by X bits
  cv::Ptr<cv::aruco::Dictionary> dictionary= cv::aruco::generateCustomDictionary(num_markers, marker_bits_size);
  float x_avg,rx_avg;
  float y_avg,ry_avg;
  float z_avg,rz_avg;
  int seen =30;


protected:


public:
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("tf/marker_frames", 1);

  void broadcast_frame(geometry_msgs::TransformStamped transformStamped) {
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transformStamped);
    pub.publish(transformStamped);
  }

  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    cv::Mat image;
    //import image from msg
    try
    {
      //convert compressed image data to cv::Mat
      image = cv::imdecode(cv::Mat(msg->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
      //if that didnt work, display error in terminal
      ROS_ERROR("Could not convert to image!");
    }
    //Your code here
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);
    // if at least one marker detected
    std::vector<cv::Vec3d> rvecs, tvecs;

    std_msgs::Header header=msg->header;
    if (ids.size() > 0){
      cv::aruco::estimatePoseSingleMarkers(corners, 0.051, cameraMatrix, distCoeffs, rvecs, tvecs);

      for (int i = 0; i < ids.size(); i++) {
        geometry_msgs::TransformStamped frame;
        //find the corect frame id
        std::string id_num= std::to_string(ids[i]);
        std::string frame_id = "marker_"+id_num;
        //add the camera header
        frame.header.stamp = msg->header.stamp;
        frame.header.frame_id = "camera_color_optical_frame";
        frame.child_frame_id = frame_id;
        frame.transform.translation.x = tvecs[i][0];
        frame.transform.translation.y = tvecs[i][1];
        frame.transform.translation.z = tvecs[i][2];
        //find RPY
        float rx = rvecs[i][0];
        float ry = rvecs[i][1];
        float rz = rvecs[i][2];
        //write as Quaternion
        tf2::Quaternion q;
        q.setRPY(rx,ry,ry);
        frame.transform.rotation.x = q.x();
        frame.transform.rotation.y = q.y();
        frame.transform.rotation.z = q.z();
        frame.transform.rotation.w = q.w();
        broadcast_frame(frame);
      }
    }
  }

  bool service_get_marker(zero_point_cal::GetMarker::Request &req,zero_point_cal::GetMarker::Response &res) {
    std::string path = req.path;
    cv::Mat marker;
    for (int id_num = 0; id_num < num_markers; id_num++) {
      cv::aruco::drawMarker(dictionary, id_num, 200, marker, 1);
      //save marker is pdf
      std::string number = std::to_string(id_num);
      std::string img_type = ".png";
      std::string slash ="/";
      std::string full_path = path+slash+number+img_type;
      //Define params to save image
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);
      try {
          cv::imwrite(full_path, marker, compression_params);
          ROS_INFO("Saved marker:%s",full_path.c_str());
          res.finished =true;
      }
      catch (cv::Exception& e) {
          ROS_ERROR("Error converting image to PNG format");
          res.finished = false;
      }
    }
    return true;
  }
};



int main(int argc, char **argv)
{
  //Ros init stuff:
  ros::init(argc, argv, "marker_detection");

  //Define class instance
  markers instance;

  //Setup publisher and subscriber

  ros::Subscriber sub = instance.nh.subscribe("/camera/color/image_raw/compressed", 30, &markers::imageCallback,&instance);
  ros::ServiceServer service = instance.nh.advertiseService("get_marker", &markers::service_get_marker,&instance);

  ros::spin();
}
