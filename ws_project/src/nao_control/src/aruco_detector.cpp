// This is a service to detect aruco marker

// INCLUDES
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/time.h>
#include <math.h>       /* atan2 */

// SERVICES FILES
#include "sensor_msgs/JointState.h"
#include "nao_control/ArucoNavigation.h"
#include "nao_control/ArucoDetector.h"
#include "nao_control/LookAround.h"
#include "nao_control/GetPos.h"

// DEFINES
#define DELTA_HEAD_ANGLE  0.1
#define ORIGIN_HEAD_ANGLE 0


static const std::string OPENCV_WINDOW = "marker position";

class aruco_marker
{
    public:
        // ros handler
        ros::NodeHandle nh_;

        // Image transport and subscriber:
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;

        cv::Mat image;
        cv::Mat dist;
        cv::Mat cameraP;

        // Aruco camera parameters
        aruco::CameraParameters cameraParameters;

        // Aruco marker parameters
        float aruco_x, aruco_y, aruco_z, aruco_a_x, aruco_a_y, aruco_a_z;
        float markerSize;

        bool end;
        bool detection;
        int time;       // Indicates the amount of seconds for searching the bottle
        int ID;
        float current_head_angle;
        float aruco_area;

        // client for look around node
        ros::ServiceClient look_around_client;

        // client for get the head angle
        ros::ServiceClient get_pos_client;

        uint8_t TIMER;
        uint8_t TIMER2;
	    ros::Timer timer;

        std::vector<aruco::Marker> arucoMarkers;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;


        aruco_marker() : it_(nh_)
        {
            image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 500, &aruco_marker::imageCallBack, this);

            // Create a timer for having control of the process time
    		timer= nh_.createTimer(ros::Duration(1.0), &aruco_marker::timerCallback, this);

            // publisher for look around node
            look_around_client = nh_.serviceClient <nao_control::LookAround>("lookaround");

            // service for getting the angle of the head
            get_pos_client = nh_.serviceClient <nao_control::GetPos>("getposition");

            detection= false;
            end= false;
            time= 0;
            timer.stop(); // Must start in OFF
		    TIMER= 0;
            TIMER2= 0;
            current_head_angle= 0;
            aruco_area= 0;
        }

        ~aruco_marker()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }


        void rotationMatrixToEulerAngles(cv::Mat R)
        {
            float sy= (float) sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));

            if(sy > 1e-6)
            {
                aruco_a_x = (float) atan2(R.at<double>(2,1) , R.at<double>(2,2));
                aruco_a_y = (float) atan2(-R.at<double>(2,0), sy);
                aruco_a_z = (float) atan2(R.at<double>(1,0), R.at<double>(0,0));
            }
            else
            {
                aruco_a_x = (float)  atan2(-R.at<double>(1,2), R.at<double>(1,1));
                aruco_a_y = (float) atan2(-R.at<double>(2,0), sy);
                aruco_a_z = (float) 0;
            }

            return;
        }

        void timerCallback(const ros::TimerEvent&)
        {
            ROS_INFO("[ARUCO] %d seconds", TIMER);
            TIMER++;
        }

        // For moving the nao's head
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

        // For getting the head position
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

        //Image callback function is executed when a new image is received:
        void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
        {
            timer.start();

            // ROS_INFO_STREAM("Image callback execution");
            cv_bridge::CvImageConstPtr cvImagePointer;
            try
            {
                cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
                image = cvImagePointer->image.clone();
            }
            catch (cv_bridge::Exception& except)
            {
                ROS_ERROR("[ARUCO] cv_bridge exception: %s", except.what());
                    return;
            }

            // Distortion matrix:
            dist = (cv::Mat_<double>(4, 1) <<
                    -0.0870160932911717,
                    0.128210165050533,
                    0.003379500659424,
                    -0.00106205540818586);

            // Camera matrix:
            cameraP = (cv::Mat_<double>(3, 3) <<
                    274.139508945831, 0.0, 141.184472810944,
                    0.0, 275.741846757374, 106.693773654172,
                    0.0, 0.0, 1.0);

            cameraParameters.setParams(cameraP,dist,cv::Size(640,480));

            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            aruco::MarkerDetector arucoDetector;
            arucoDetector.detect(image, arucoMarkers);
            cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejectedCandidates);


            if (arucoMarkers.size() > 0)
            {
                for(int i=0; i<=arucoMarkers.size(); i++)
                {
                    ROS_INFO("trying to see ID %d", ID);
                    if (arucoMarkers[i].id == ID)
                    {
                        TIMER= 0;
                        cv::Point2f marker_center = arucoMarkers[i].getCenter();

                        if(marker_center.x < 150)
                        {
                            cout << "[ARUCO] it is too " << "RIGHT" << endl;

                            // ALIGN THE OBJECT TO THE CENTER OF THE VIEW
                            headPosition("HeadYaw");
                            headMovement("HeadYaw", current_head_angle + DELTA_HEAD_ANGLE/2);

                        }
                        else if(marker_center.x > 170)
                        {
                            cout << "[ARUCO] it is too " << "LEFT" << endl;

                            // ALIGN THE OBJECT TO THE CENTER OF THE VIEW
                            headPosition("HeadYaw");
                            headMovement("HeadYaw", current_head_angle - DELTA_HEAD_ANGLE/2);
                        }
                        else
                        {
                            cout << "[ARUCO] center: " << "CENTER" << endl;
                            ROS_WARN_STREAM("[ARUCO] I could detect "<<arucoMarkers.size()<<" aruco marker(s).");
                            arucoMarkers[i].draw(image, cv::Scalar(0,0,255), 2);

                            // Transformation of the rotated aruco area
                            std::vector<cv::Vec3d> rvecs, tvecs, objPoints;
                            cv::Mat R(3, 3, CV_64F);
                            /* We look for the orientation coordinates */
                            cv::aruco::estimatePoseSingleMarkers(corners,  0.085, cameraP, dist, rvecs, tvecs, objPoints);
                            cout<< "cal angles " << rvecs[0] << tvecs[0] << endl;
                            for(int i=0; i<ids.size(); i++) {
                                if(ids[i] == ID)
                                {
                                    cv::aruco::drawAxis(image, cameraP, dist, rvecs[i], tvecs[i], 0.1);
                                    Rodrigues(rvecs[i], R);
                                    ROS_INFO("Rodrigues");
                                    Rodrigues(rvecs[i], R);
                                    rotationMatrixToEulerAngles(R);
                                }
                            }
                            float sy= (float) sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));

                            if(sy > 1e-6)
                            {
                                aruco_a_x = (float) atan2(R.at<double>(2,1) , R.at<double>(2,2));
                                aruco_a_y = (float) atan2(-R.at<double>(2,0), sy);
                                aruco_a_z = (float) atan2(R.at<double>(1,0), R.at<double>(0,0));
                            }
                            else
                            {
                                aruco_a_x = (float) atan2(-R.at<double>(1,2), R.at<double>(1,1));
                                aruco_a_y = (float) atan2(-R.at<double>(2,0), sy);
                                aruco_a_z = (float) 0;
                            }
                            
                            aruco_a_y = (float) atan2(-R.at<double>(2,0), sy);

                            // END THE SERVICE
                            aruco_area= arucoMarkers[i].getArea()/(cos(aruco_a_y)*cos(aruco_a_y));
                            headMovement("HeadYaw", ORIGIN_HEAD_ANGLE);
                            
                            if(!isnan(aruco_area))
                            {
                                detection= true;
                                end= true;
                            }                            
                        }
                    }
                }
            }
            else
            {
                if(TIMER > 5)
                {
                    headMovement("HeadYaw", ORIGIN_HEAD_ANGLE);
                    ROS_WARN_STREAM("[ARUCO] I could detect "<<arucoMarkers.size()<<" aruco marker(s).");
                    detection= false;
                    end= true;
                    timer.stop();
                    TIMER= 0;
                }
            }

            // Display marker
            cv::imshow(OPENCV_WINDOW,image);
            cv::moveWindow(OPENCV_WINDOW, 1300, 200);
            cv::waitKey(3);
        }
};

bool serviceCallback(nao_control::ArucoDetector::Request  &req, nao_control::ArucoDetector::Response &res)
{
    // Creates an instance for searching the aruco marker
    aruco_marker ic;
    ic.time= req.time;
    ic.ID = req.id;

    // Waits until a aruco marker is found
    while(!ic.end)
      ros::spinOnce();

    // Sends the response to the client
    if(ic.detection)
    {
        res.reply= true;
        res.angle= ic.current_head_angle;
        res.aruco_area = ic.aruco_area;
    }
    else
    {
      res.reply= false;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arucodetector_node");
    ros::NodeHandle n;

    // Service for aruco detector
    ros::ServiceServer service = n.advertiseService("arucodetector", serviceCallback);

    ros::spin();
    return 0;

}
