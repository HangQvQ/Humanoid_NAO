// This is a serevice to detect bottles

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/matx.hpp>
#include <aruco/aruco.h>
// Service files
#include "nao_control/BottleDetector.h"
#include "nao_control/LookAround.h"
#include "nao_control/GetPos.h"

using namespace std;

static const std::string OPENCV_WINDOW1 = "ROI";
static const std::string OPENCV_WINDOW2 = "Backprojection";
static const std::string OPENCV_WINDOW3 = "Blob extraction";
#define DELTA_HEAD_ANGLE  0.1
#define ORIGIN_HEAD_ANGLE 0

class bottle_detection
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_top;
  image_transport::Publisher image_pub_;
  
  // detection
  cv::Mat h_hist;
  float range[2] = { 0, 180 };
  int histSize[1] = { 50 };
  const float* histRange[1] = { range };
  cv::Rect rectangle;
  cv::RotatedRect rot_rectangle;
  
  // client for look around node
	ros::ServiceClient look_around_client;
  
  // client for get the head angle
  ros::ServiceClient get_pos_client;
  
public:
  // Public attributes
  bool detection; // Indicates if a bottle was found
  bool end;       // Indicates if the process is finished
  float current_head_angle;
  float current_head_pitch;
  int time;       // Indicates the amount of seconds for searching the bottle
  float blob_size;  // Indicates the siye of the circle drawn by the blob detector
  cv::Point2f blob_pt;

	uint8_t TIMER;
	ros::Timer timer;

  bottle_detection()
    : it_(nh_)
  {
    // Subscribe to input top video feed and show the 3 formats (RGB, grey, binary)
    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    
    // Initialize the detection (Get ROI in the template image and compute its histogram etc.)
    initDetection();
    detection= false;
    end= false;
    time= 0;

    image_sub_top = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &bottle_detection::topImageCb, this);
    
    // Create a timer for having control of the process time
		timer= nh_.createTimer(ros::Duration(1.0), &bottle_detection::timerCallback, this);

    // publisher for look around node
		look_around_client = nh_.serviceClient <nao_control::LookAround>("lookaround");

    // service for getting the angle of the head
    get_pos_client = nh_.serviceClient <nao_control::GetPos>("getposition");

		timer.stop(); // Must start in OFF
		TIMER= 0;

    current_head_angle= 0;
    current_head_pitch= 0;
    blob_size= 0;
    blob_pt.x= 0;
    blob_pt.y= 0;
  }

  ~bottle_detection()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
  }

  void timerCallback(const ros::TimerEvent&)
	{

		ROS_INFO("[BOTTLE] %d seconds", TIMER);

		TIMER++;
	}

  void headMovement(string name, float angle)
  {
    /* THIS TRIGGERS THE LOOK AROUND NODE */
    nao_control::LookAround srv;
    srv.request.name= name;
    srv.request.theta= angle;

    if (look_around_client.call(srv))
    {
      ROS_INFO("[BOTTLE] succesfully to call look around service");
    }
    else
    {
      ROS_ERROR("[BOTTLE] Failed to call look around service");
    }
  }

  void headPosition(string name)
  {
    /* THIS TRIGGERS THE LOOK AROUND NODE */
    nao_control::GetPos srv;
    srv.request.joint_name = name;

    if (get_pos_client.call(srv))
    {
        ROS_INFO("[BOTTLE] succesfully to call get head angle service");
        current_head_angle= srv.response.cur_pos.angular.z;

    }
    else
    {
        ROS_ERROR("[BOTTLE] Failed to call get head angle service");
    }    
  }

  void initDetection()
  {    
    // read template image
    cv::Mat img;
    img = cv::imread("/home/hrse/HRS_Group_E/ws_project/resources/templateImg.png");
    
    // draw ROI to template image
    cv::Mat img_roi = img.clone();
    rectangle = cv::Rect(162,36,36,52);
		//rot_rectangle = cv::RotatedRect(cv::Point2f(68,88),cv::Size2f(95,93),30);
		cv::rectangle(img_roi, rectangle,cv::Scalar(0,0,255),2,8,0);

    // extract ROI 
    cv::Mat roi = img(cv::Range(36,88),cv::Range(162,198));
    cv::imshow(OPENCV_WINDOW1,roi);
    cv::moveWindow(OPENCV_WINDOW1, 900, 400);

    // convert to HSV color space and split channels
    cv::Mat hsvImage;
    cv::cvtColor(roi, hsvImage, CV_BGR2HSV);
    cv::Mat hsv_channels[3];
    cv::split(hsvImage, hsv_channels);

    // mask out low saturated pixels by thresholding
    cv::Mat mask, binary_mask;
    cv::threshold(hsv_channels[1], mask, 170, 255, cv::THRESH_TOZERO);
    cv::threshold(hsv_channels[1], binary_mask, 100, 1, cv::THRESH_BINARY);
    
    // Compute histogram
    cv::calcHist(&hsv_channels[0],1,0,binary_mask,h_hist,1,histSize,histRange,true,false);
    
    // normalize
    cv::normalize(h_hist, h_hist, 0, 180, cv::NORM_MINMAX);

  }

  // For checking if a point is inside a rectangle
  bool pointInRectangle(cv::Point point, cv::Point2f* vertices)
  {
    double area_APD = abs((  point.x     * vertices[0].y - vertices[0].x *   point.y)     + (vertices[3].x *   point.y     -   point.x     * vertices[3].y) + (vertices[0].x * vertices[3].y - vertices[3].x * vertices[0].y)) / 2;
    double area_DPC = abs((  point.x     * vertices[3].y - vertices[3].x *   point.y)     + (vertices[2].x *   point.y     -   point.x     * vertices[2].y) + (vertices[3].x * vertices[2].y - vertices[2].x * vertices[3].y)) / 2;
    double area_CPB = abs((  point.x     * vertices[2].y - vertices[2].x *   point.y)     + (vertices[1].x *   point.y     -   point.x     * vertices[1].y) + (vertices[2].x * vertices[1].y - vertices[1].x * vertices[2].y)) / 2;
    double area_PBA = abs((vertices[1].x *   point.y     -   point.x     * vertices[1].y) + (vertices[0].x * vertices[1].y - vertices[1].x * vertices[0].y) + (  point.x     * vertices[0].y - vertices[0].x *     point.y  )) / 2;

    double sum_triangles= area_APD + area_DPC + area_CPB + area_PBA;

    double aux= crossProduct(vertices[1]-vertices[0], vertices[2]-vertices[1]);
    double sum_rectangle= abs(aux);

    if(sum_triangles > sum_rectangle)
      return false;

    return true;
  }

  double crossProduct(cv::Point u, cv::Point v) 
  {
    return (u.x * v.y - u.y * v.x);
  }

  void blobDetection(cv::Mat hsvImage)
  {
      cv::Mat binaryImage1, binaryImage2, binaryImage;
      
      // color extraction
      cv::inRange(hsvImage, cv::Scalar(0, 100, 20), cv::Scalar(10,255,255), binaryImage1);
      cv::inRange(hsvImage, cv::Scalar(160,100,20), cv::Scalar(179,255,255), binaryImage2);
      binaryImage = binaryImage1 + binaryImage2;
      
      // dilate/erodate
      cv::Mat elementErode, erodedImage, elementDilate, dilatedImage;
      elementErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
      elementDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6,6));
      cv::erode(binaryImage, erodedImage, elementErode);
      cv::dilate(erodedImage, dilatedImage, elementDilate);
      
      // Set up the detector with default parameters.
      cv::SimpleBlobDetector::Params params;
      // Change thresholds
      params.minThreshold = 10;
      params.maxThreshold = 200;
      params.filterByColor = true;
      params.blobColor = 255;

      // Filter by Area.
      params.filterByArea = true;
      params.minArea = 200;
      params.maxArea = 10000000;

      cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
      
      // Detect blobs.
      std::vector<cv::KeyPoint> keypoints;
      detector->detect(dilatedImage, keypoints);

      int max_index = 0;
      for(int i = 0; i < keypoints.size(); i++) {
          if(keypoints[max_index].size < keypoints[i].size){
              max_index = i;
          }
      }
      
      cv::Mat imageWithKeypoints;
      std::vector<cv::KeyPoint> maxKeypoints;
      
      if(keypoints.size()){
          maxKeypoints.push_back(keypoints[max_index]);
          blob_pt = keypoints[max_index].pt;
          blob_size = keypoints[max_index].size;
      } 
      
      // Draw detected blobs as red circles.
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
      cv::drawKeypoints(dilatedImage, maxKeypoints, imageWithKeypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      cv::drawKeypoints(imageWithKeypoints, maxKeypoints, imageWithKeypoints, cv::Scalar(255,0,0));

      cv::imshow(OPENCV_WINDOW3, imageWithKeypoints);
      cv::moveWindow(OPENCV_WINDOW3, 1500, 400);

      cv::waitKey(3);
  }

  void runDetection(cv_bridge::CvImagePtr cv_ptr)
  {
    timer.start();

    // convert to HSV color space and split channels
    cv::Mat hsvImage;
    cv::cvtColor(cv_ptr->image, hsvImage, CV_BGR2HSV);
    cv::Mat hsv_channels[3];
    cv::split(hsvImage, hsv_channels);
    
    cv::Mat binary_mask;
    cv::threshold(hsv_channels[0], binary_mask, 0, 10, cv::THRESH_BINARY);

    // Backproject
    cv::Mat backProjection;
    calcBackProject(&hsv_channels[0], 1, 0, h_hist, backProjection, histRange, 1, true);
    backProjection = hsv_channels[1] & backProjection;
    
    // Mean Value
    cv::Scalar tempVal = cv::mean( backProjection );
    float myMAtMean = tempVal.val[0];
    
    cv::imshow(OPENCV_WINDOW2,backProjection);
    cv::moveWindow(OPENCV_WINDOW2, 900, 700); //900, 400

    // Draw rotated rectangle
    if(myMAtMean > 1.2) // If there is enough key points in the backprojection matrix
    {
      TIMER= 0;

      // mean shift / cam shift
      cv::Point2f vertices[4];
      cv::TermCriteria term_crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1);
      cv::Size subPixWinSize(10,10), winSize(31,31);
      // cv::meanShift(backProjection, rectangle, term_crit);
      rot_rectangle = cv::CamShift(backProjection, rectangle, term_crit);
      rot_rectangle.points(vertices);

      blobDetection(hsvImage);
      if(pointInRectangle(blob_pt, vertices) && blob_size > 0) // Rotated rectangle located in the target
      {
        cout << "[BOTTLE] circle= INSIDE" << endl;
        
        for (int i = 0; i < 4; i++)
          cv::line(cv_ptr->image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);

         //get error of image center and marker center
        if(rot_rectangle.center.x < 150)
        {
          
          cout << "[BOTTLE] it is too " << "RIGHT" << endl;
          
          // ALIGN THE OBJECT TO THE CENTER OF THE VIEW        
          headPosition("HeadYaw");
          headMovement("HeadYaw", current_head_angle + DELTA_HEAD_ANGLE);

        }
        else if(rot_rectangle.center.x > 170)
        {      
          cout << "[BOTTLE] it is too " << "LEFT" << endl;

          // ALIGN THE OBJECT TO THE CENTER OF THE VIEW        
          headPosition("HeadYaw");
          headMovement("HeadYaw", current_head_angle - DELTA_HEAD_ANGLE);
        }
        else
        {
          cout << "[BOTTLE] center: " << "CENTER" << endl;
          cout << "[BOTTLE] size: " << rot_rectangle.size << endl;

          // END THE SERVICE
          headMovement("HeadYaw", ORIGIN_HEAD_ANGLE);
          detection= true;          
          end= true;
        }                  
      }
      else  // Rotated rectangle not located in the target, we move the HeadYaw in that direction
      {
        cout << "[BOTTLE] circle= OUTSIDE" << endl;

        if(blob_pt.x < 150)
        {
          
          cout << "[BOTTLE] searching bottle on the " << "RIGHT" << endl;
          
          // ALIGN THE OBJECT TO THE CENTER OF THE VIEW        
          headPosition("HeadYaw");
          headMovement("HeadYaw", current_head_angle + DELTA_HEAD_ANGLE);

        }
        else if(blob_pt.x > 170)
        {      
          cout << "[BOTTLE] searching bottle on the " << "LEFT" << endl;

          // ALIGN THE OBJECT TO THE CENTER OF THE VIEW        
          headPosition("HeadYaw");
          headMovement("HeadYaw", current_head_angle - DELTA_HEAD_ANGLE);
        }
      }
    }
    else
    {
      if(TIMER > 5)
      {
        headMovement("HeadYaw", ORIGIN_HEAD_ANGLE);
        detection= false;
        end= true;
        timer.stop();
        TIMER= 0;
      }
    }

    cv::imshow(OPENCV_WINDOW1, cv_ptr->image);
    cv::moveWindow(OPENCV_WINDOW1, 900, 400);
  }

  void topImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      //////// OBJECT DETECTION ////////
      runDetection(cv_ptr);

      cv::waitKey(3);
  }
};

bool serviceCallback(nao_control::BottleDetector::Request  &req, nao_control::BottleDetector::Response &res)
{
    // Creates an instance for looking a bottle
    bottle_detection ic;
    ic.time= req.time;

    // Waits until a bottle is found
    while(!ic.end)
      ros::spinOnce();

    // Sends the response to the client
    if(ic.detection)
    {
      res.reply= true;
      res.angle= ic.current_head_angle;
      res.blob_size = ic.blob_size;
    }
    else
    {
      res.reply= false;
    }
    
    ros::Duration(1).sleep();

    return true;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "bottle_detector");
    ros::NodeHandle n;
    
    // Service for bottle detector
    ros::ServiceServer service = n.advertiseService("bottledetector", serviceCallback);

    ros::spin();
    

    return 0;
}
