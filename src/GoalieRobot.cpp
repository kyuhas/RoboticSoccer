#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

static const std::string OPENCV_WINDOW = "Image window";

//constants used throughout the program
#define RGB_FOCAL_LEN_MM 138.90625 // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 203.2         // 8" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0     // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0        // in pixels
#define IMG_WIDTH_PX 640.0         // in pixels
#define MAX_BOT_VEL 0.65           // max speed TurtleBot is capable of
#define MIN_BOT_VEL 0.2            // the min speed I want the TurtleBot to go
#define MIN_RADIUS 0
#define MAX_RADIUS 0
#define X 0
#define Y 1
#define R 2

class GoalieRobot
{
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;
    ros::Subscriber gameSub_ = nodeHandle_.subscribe("/gameCommands", 10, &GoalieRobot::gameCommandCallback, this);
    ros::Publisher velPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist twistMsg;
    geometry_msgs::Pose goaliePos;

    cv::Scalar black = (0, 255, 5), blue = (200, 200, 250);

    bool inGame;

    double objDist = 0.0, botVelX = 0.0, alignError = 0.0, rotation = 0.5;
    int xCoord, yCoord, radius;

    std::vector<int> objCoord = {0, 0, 0};

  public:
    //constructor
    GoalieRobot() : imageTransport_(nodeHandle_)
    {
        // real camera
        imageSub_ = imageTransport_.subscribe("/usb_cam/image_raw", 10, &GoalieRobot::playSoccer, this);
        imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

        //initialize bools
        inGame = false;
    }
    //destructor
    ~GoalieRobot() {}

    // method to find the distance the robot is from the ball
    double distFromObj(int objSize)
    {
        double distMeters = (((RGB_FOCAL_LEN_MM * BALL_DIAM_MM * IMG_HEIGHT_PX) / (objSize * CAMERA_HEIGHT_MM)) / 1000);
        return floor((distMeters * 10 + 0.5)) / 10;
    }

    void moveForNumOfSecs(double seconds)
    {
        ros::Time start = ros::Time::now();

        while (ros::Time::now() - start < ros::Duration(seconds))
            velPub.publish(twistMsg);
    }

    // method to have the robot move so that it can see the ball
    void moveTurtleBot(double alignError, double objDist)
    {
        if (objDist > 1.5)
        {
            twistMsg.linear.x = 0.0;
            twistMsg.angular.z = alignError;
            velPub.publish(twistMsg);
            return;
        }

        // travels 1.25 meters. To try and cut off the ball
        twistMsg.linear.x = 0.5;
        // accentuate ang vel to account for the ball moving past it while tracking is lost
        twistMsg.angular.z = 2.0 * alignError;

        // the number of seconds can be adjusted as needed to shorten the dist traveled
        moveForNumOfSecs(2.5);

        // pause between motions
        ros::Duration(2).sleep();

        // follows the reverse path
        twistMsg.linear.x = -0.5;
        twistMsg.angular.z = -2.0 * alignError;

        moveForNumOfSecs(2.75);
    }

    void trackBall(std::vector<cv::Vec3f> circleIMG, cv::Mat srcIMG)
    {
        if (circleIMG.empty())
        {
            objDist = 99.0;
            moveTurtleBot(0.0, objDist);
        }

        else
            for (size_t i = 0; i < circleIMG.size(); i++)
            {
                // center coordinates of circle, and the radius
                objCoord[X] = static_cast<int>(round(circleIMG[i][0]));
                objCoord[Y] = static_cast<int>(round(circleIMG[i][1]));
                objCoord[R] = static_cast<int>(round(circleIMG[i][2]));

                cv::Point center(objCoord[X], objCoord[Y]);

                // draws circle around ball and cross-hair at center
                cv::circle(srcIMG, center, objCoord[R], black, 2);
                cv::line(srcIMG, center, center, black, 2);

                objDist = distFromObj(objCoord[R]);
                alignError = -(objCoord[X] - (IMG_WIDTH_PX / 2.0)) / 225;

                // move the turtlebot
                moveTurtleBot(alignError, objDist);
            }
    }

    //method to have the robot search for the ball and move toward it
    void searchForBall(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cvPtr;
        std::vector<cv::Vec3f> redCircleIMG, circleIMG;
        cv::Mat srcIMG, hsvIMG, redIMG_lower, redIMG_upper, redIMG;

        try
        {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            srcIMG = cvPtr->image;

            cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

            cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), redIMG_lower);
            cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255), redIMG_upper);

            cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);
            cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);

            cv::HoughCircles(redIMG, redCircleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
        }

        catch (cv_bridge::Exception &exception)
        {
            ROS_ERROR("cv_bridge exception: %s", exception.what());
            return;
        }

        trackBall(redCircleIMG, srcIMG);
        cv::imshow(OPENCV_WINDOW, cvPtr->image);
        cv::waitKey(3);
        imagePub_.publish(cvPtr->toImageMsg());
    }

    //method where the robot decides which action to take (try to block goal or search for ball)
    void playSoccer(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!inGame)
            return;

        searchForBall(msg);
    }

    //method to handle game commands
    void gameCommandCallback(const std_msgs::String::ConstPtr &msg)
    {
        inGame = false;

        if (strcmp(msg->data.c_str(), "start") == 0)
            inGame = true;

        else if (strcmp(msg->data.c_str(), "stop") == 0)
        {
            twistMsg.linear.x = 0;
            twistMsg.angular.z = 0;
            velPub.publish(twistMsg);
        }
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "goalie_robot");
    GoalieRobot goalie;
    ros::spin();

    return 0;
}
