#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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

  //Make a vector to take avg so that:
  //avg[id][0].x = tx
  //avg[id][1].x = rx
  std::vector<cv::Point3f> Point = decltype(Point)(2);
  std::vector<std::vector<cv::Point3f>> avg = decltype(avg)(num_markers,Point); //Create vector of ids containg a vector of size 4 each containing a cv point 3f
  std::vector<int> counter =decltype(counter)(num_markers);
protected:


public:
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("tf/marker_frames", 1);

  void broadcast_frame(double rx, double ry, double rz, double tx, double ty, double tz, std::string id_num) {
    std::string frame_id = "marker_"+id_num;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_color_optical_frame";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform.translation.x = tx;
    transformStamped.transform.translation.y = ty;
    transformStamped.transform.translation.z = tz;
    tf2::Quaternion q;
    q.setRPY(rz,ry,rx);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

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
    find_marker(image);
  }

  void find_marker(cv::Mat image) {
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);
    // if at least one marker detected
    std::vector<cv::Vec3d> rvecs, tvecs;

    if (ids.size() > 0){
      cv::aruco::estimatePoseSingleMarkers(corners, 0.051, cameraMatrix, distCoeffs, rvecs, tvecs);

      for (int i = 0; i < ids.size(); i++) {
        float tx = tvecs[i][0];
        float ty = tvecs[i][1];
        float tz = tvecs[i][2];
        float rx = rvecs[i][0];
        float ry = rvecs[i][1];
        float rz = rvecs[i][2];
        //add avg trans
        avg[ids[i]][0].x=avg[ids[i]][0].x+tx;
        avg[ids[i]][0].y=avg[ids[i]][0].y+ty;
        avg[ids[i]][0].z=avg[ids[i]][0].z+tz;
        //add avg rotation
        avg[ids[i]][1].x=avg[ids[i]][1].x+rx;
        avg[ids[i]][1].y=avg[ids[i]][1].y+ry;
        avg[ids[i]][1].z=avg[ids[i]][1].z+rz;
        //Set the counter for the given id
        counter[ids[i]]=counter[ids[i]]+1;

        //Find the afv and send that pos to the broadcaster
        if (counter[ids[i]]==seen) {
          //Send to bradcaster
          std::string id = std::to_string(ids[i]);
          //set avg trans
          float tx_avg=avg[ids[i]][0].x/seen;
          float ty_avg=avg[ids[i]][0].y/seen;
          float tz_avg=avg[ids[i]][0].z/seen;
          //set avg rotation
          float rx_avg=avg[ids[i]][1].x/seen;
          float ry_avg=avg[ids[i]][1].y/seen;
          float rz_avg=avg[ids[i]][1].z/seen;

          //Send to broadcaster
          broadcast_frame( rx_avg, ry_avg, rz_avg, tx_avg, ty_avg, tz_avg, id);
          //Reset all
          counter[ids[i]]=0;
          avg[ids[i]][0].x=0;
          avg[ids[i]][0].y=0;
          avg[ids[i]][0].z=0;
          //set avg rotation
          avg[ids[i]][1].x=0;
          avg[ids[i]][1].y=0;
          avg[ids[i]][1].z=0;
        }
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

  ros::Subscriber sub = instance.nh.subscribe("/camera/color/image_raw/compressed", 1, &markers::imageCallback,&instance);
  ros::ServiceServer service = instance.nh.advertiseService("get_marker", &markers::service_get_marker,&instance);

  ros::spin();
}
