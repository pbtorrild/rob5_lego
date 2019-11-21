#include <ros/ros.h>
#include <iostream>
#include <vector>

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/mat.hpp>

#include <cv_bridge/cv_bridge.h>


class markers{
private:
  // camera calibration, is found using http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  // and the topic /camera/color/came_info
  cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << 614.7054443359375, 0.0, 320.0, 0.0, 614.9154052734375, 240.0, 0.0, 0.0, 1.0);
  cv::Mat distCoeffs= (cv::Mat_<float>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

  float to_degree = 180/3.14159265359;
  int num_markers=14;
  int marker_bits_size=4; //X by X bits
  cv::Ptr<cv::aruco::Dictionary> dictionary= cv::aruco::generateCustomDictionary(num_markers, marker_bits_size);

  float x_avg;
  float y_avg;
  float z_avg;
  int seen =5; //How many times a marker should be seen b4 we use the values

protected:
  std::vector<cv::Point3f> corner = decltype(corner)(4);
  std::vector<std::vector<cv::Point3f>> avg = decltype(avg)(num_markers,corner); //Create vector of ids containg a vector of size 4 each containing a cv point 3f
  std::vector<int> counter =decltype(counter)(num_markers);
public:
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("tf/marker_frames", 1);

  void prossesing(const sensor_msgs::CompressedImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
  {
    //Empty holders for images
    cv::Mat color_img;
    cv::Mat depth_img;
    //import image from msg
    try
    {
      //convert compressed image data to cv::Mat
      color_img = cv::imdecode(cv::Mat(color->data),CV_LOAD_IMAGE_COLOR);
    }
    catch (cv_bridge::Exception& e)
    {
      //if that didnt work, display error in terminal
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //We do the same but are using the cv_bridge for depth
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      //This allows us to control the image encoding this case is a 32bit float on 1 channel
      //thereby providing a TYPE_32FC1
      cv_ptr = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //Set the image with the correct image type
    depth_img=cv::Mat(cv_ptr->image.size(), cv_ptr->image.type());
    cv_ptr->image.copyTo(depth_img);

    //Get color center for calculating the relative distance between the two image centers
    float color_width = color_img.cols;
    float color_height = color_img.rows;
    float color_center_x = color_width/2;
    float color_center_y = color_height/2;
    //Get depth_center - same purpose as b4
    int depth_width = depth_img.cols;
    int depth_height = depth_img.rows;
    int depth_center_x = depth_width/2;
    int depth_center_y = depth_height/2;

    //We load the distance to the center this is used to find the range to each point
    float center_dist = depth_img.at<float>(depth_center_x,depth_center_y);

    //The FoV is calculated from the cameta matrix
    //We use this to find the angles in x and why directions
    float FoV_x = 2*atan(color_width/(2*614.7054443359375));
    float FoV_y = 2*atan(color_height/(2*614.9154052734375));
    float degree_x = FoV_x/color_width;
    float degree_y = FoV_y/color_height;

    //Find markers - here is a vector containing each id number
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners_on_img; //A vector of a vector containing the corner points

    //Aruco marker detection algorithm see https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html
    cv::aruco::detectMarkers(color_img, dictionary, corners_on_img, ids);

    //function to draw the marker on the color image - for illustartion only
    cv::aruco::drawDetectedMarkers(color_img, corners_on_img, ids);

    //Boolian value that check if the calculations on each corner has been done correctly
    std::vector<bool> new_point=decltype(new_point)(4,false); // of size 4 and all false values
    std::vector<std::vector<bool>> cal_check = decltype(cal_check)(num_markers,new_point); //a vector of size num_markers each containg a vector like new_point
    //Only run code if one marker is found
    if (ids.size()!=0) {
      //For every marker found
      for (int i = 0; i < ids.size(); i++) {
        for (int j = 0; j < 4; j++) {
          //redefine distance to center
          float x=corners_on_img[i][j].x-color_center_x; //this is a distance in px
          float y=corners_on_img[i][j].y-color_center_y; //this is a distance in px
          float r=depth_img.at<float>(depth_center_x+x,depth_center_y+y); //this is a distance in mm
          //redefine as angles
          //We use azimuth angles to calculate - see https://en.wikipedia.org/wiki/Spherical_coordinate_system
          float b_sq = abs((center_dist*center_dist)-(r*r));
          float b = sqrt(b_sq);
          float phi=atan2(y,x);
          float theta=asin(b/r);
          //calculate cartesian coordinates
          float X = r * sin(theta)*cos(phi);
          float Y = r * sin(theta)*sin(phi);
          float Z = r * cos(theta);
          //see if our coordinates are actually a number orher whise skip
          if (std::isnan(X) !=1 ||std::isnan(Y) !=1 ||std::isnan(Z) !=1) {
            //add to avg value
            avg[ids[i]][j].x=X+avg[ids[i]][j].x;
            avg[ids[i]][j].y=Y+avg[ids[i]][j].y;
            avg[ids[i]][j].z=Y+avg[ids[i]][j].z;
            //set the check for this specific point to true
            cal_check[ids[i]][j]=true;
          }


        }
        //check that all four corners are numbers - they are multiplied so if one is false all will be
        bool total_check = cal_check[ids[i]][0]*cal_check[ids[i]][1]*cal_check[ids[i]][2]*cal_check[ids[i]][3];
        //If the all corners are numbers add one to the counter
        if (total_check ==true) {
          counter[ids[i]]=counter[ids[i]]+1;
        }
        //When we have seen the marker "seen" number of times - find the avg position of the marker center
        if (counter[ids[i]]==seen) {
          //Add all of the x,y and z values together
          float x_avg, y_avg, z_avg;
          for (int j = 0; j < 4; j++) {
            x_avg = x_avg+avg[ids[i]][j].x;
            y_avg = y_avg+avg[ids[i]][j].y;
            z_avg = z_avg+avg[ids[i]][j].z;
          }
          //Devide by 4 to get the avg
          x_avg=x_avg/4; y_avg=y_avg/4; z_avg=z_avg/4;
          //Devide by the times it has been seen
          x_avg=x_avg/seen; y_avg=y_avg/seen; z_avg=z_avg/seen;

          //ROS_INFO the id of the marker and the pos in 3d space
          ROS_INFO("ID: %d @(%f,%f,%f)",ids[i],x_avg,y_avg,z_avg);

          //Get the id number as a string
          std::string id_num = std::to_string(ids[i]);
          //find roll pitch yall
          float rz = atan((avg[ids[i]][0].y-avg[ids[i]][1].y)/(avg[ids[i]][0].x-avg[ids[i]][1].x));
          float ry = atan((avg[ids[i]][0].z-avg[ids[i]][1].z)/(avg[ids[i]][0].x-avg[ids[i]][1].x));
          float rx = atan((avg[ids[i]][1].z-avg[ids[i]][3].z)/(avg[ids[i]][1].y-avg[ids[i]][3].y));

          //Broadcast the frame on the TF broadcaster
          broadcast_frame(rz,ry,rx,x_avg/1000,y_avg/1000,z_avg/1000,id_num);

          //Reset to 0
          x_avg=0;
          y_avg=0;
          z_avg=0;
          counter[ids[i]]=0;
          for (int j = 0; j < 4; j++) {
            avg[ids[i]][j].x=0;
            avg[ids[i]][j].y=0;
            avg[ids[i]][j].z=0;
          }
        }//End of id counter equal to 30
      }//End of id forloop
    }//End of ids.size >0 statement;
  }//End void

  void broadcast_frame(float rz, float ry, float rx, float tx, float ty, float tz, std::string id_num) {
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





};



int main(int argc, char **argv)
{
  //Ros init stuff:
  ros::init(argc, argv, "image_listener");


  //Begin to show image
  //cv::namedWindow("view");
  //cv::startWindowThread();

  //Define class instance
  markers instance;

  //Setup publisher and subscriber

  message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub(instance.nh, "/camera/color/image_raw/compressed", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(instance.nh, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::TimeSynchronizer<sensor_msgs::CompressedImage,sensor_msgs::Image> sync(image_sub, depth_sub, 2);
  sync.registerCallback(&markers::prossesing,&instance);

  //ros::Subscriber color_sub = instance.nh.subscribe("", 1, &markers::imageCallback,&instance);
  //ros::Subscriber depth_sub = instance.nh.subscribe("", 1, &markers::depthCallback,&instance);


  ros::spin();
  cv::destroyAllWindows();
}
