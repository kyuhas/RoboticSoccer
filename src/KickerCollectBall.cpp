#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <time.h>
#include <math.h>
#include <limits>

//constants used throughout the program
#define RGB_FOCAL_LEN_MM 138.90625 // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 203.2         // 8" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0     // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0        // in pixels
#define IMG_WIDTH_PX 640.0         // in pixels
#define MAX_BOT_VEL 0.65           // max speed TurtleBot is capable of
#define MIN_BOT_VEL 0.2            // the min speed I want the TurtleBot to go
#define RED 0
#define GREEN 1
#define X 0
#define Y 1
#define R 2
#define MIN_RADIUS 0
#define MAX_RADIUS 0
#define MID_X_LOW 270
#define MID_X_HIGH 380

static const std::string OPENCV_WINDOW = "Image window";

class KickerRobot
{
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;
    ros::Subscriber gameSub_ = nodeHandle_.subscribe("/gameCommands", 10, &KickerRobot::gameCommandCallback, this);
    ros::Publisher velPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    geometry_msgs::Twist twistMsg;

    double alignError = 0.0;

    cv::Scalar black = (0, 255, 5), blue = (200, 200, 250);          // RGB color for circle to be drawn on image
    std::vector<double> objDist = {0.0, 0.0};                        // RED, GREEN in that order
    std::vector<std::vector<int>> objCoord = {{0, 0, 0}, {0, 0, 0}}; // RED, GREEN, in that order
    std::vector<bool> isEmpty = {false, false};                      // RED, GREEN, in that order
    bool inGame, hasRedBall;

  public:
    // constructor
    KickerRobot() : imageTransport_(nodeHandle_)
    {
        imageSub_ = imageTransport_.subscribe("/usb_cam/image_raw", 10, &KickerRobot::playSoccer, this);
        imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

        // initialize booleans because C++ does not do this for us
        hasRedBall = false;
	inGame = false;
    }
    // destructor
    ~KickerRobot() {}

    // method to find the distance the robot is from the ball
    double distFromObj(int objSize)
    {
        double distMeters = (((RGB_FOCAL_LEN_MM * BALL_DIAM_MM * IMG_HEIGHT_PX) / (objSize * CAMERA_HEIGHT_MM)) / 1000);
        return floor((distMeters * 10 + 0.5)) / 10;
    }

    // method to have the robot move to the ball's location or look for the ball
    void moveTurtleBot(bool rotate = false)
    {
        twistMsg.angular.z = -alignError / 225.0; // 225 worked well as a denominator to smooth the alignment

        if (rotate)
        {
            twistMsg.angular.z = 0.5;
            twistMsg.linear.x = 0.0;
        }

        else if (!hasRedBall)
        {
            twistMsg.linear.x = (0.3 * objDist[RED]);
        }

        else if (hasRedBall && isEmpty[GREEN])
        {
            twistMsg.angular.z = 0.2;
            twistMsg.linear.x = 0.0;
        }

        else if (hasRedBall && !isEmpty[GREEN])
        {
            // if you are far away from the goal, move toward it. otherwise, try to kick the ball
            if (objDist[GREEN] > 2.5)
	    {
                twistMsg.linear.x = MAX_BOT_VEL;
	    }

            else
	    {
		twistMsg.linear.x = 0;
		twistMsg.angular.z = 0;
		inGame = false;
	    }
        }

        if (twistMsg.linear.x > MAX_BOT_VEL)
            twistMsg.linear.x = MAX_BOT_VEL;

        velPub.publish(twistMsg);
    }

    // method to track the ball
    void trackBall(std::vector<cv::Vec3f> circleIMG, cv::Mat srcIMG, int color)
    {
        // if the image does not contain any of the color we are looking for, give that color a zero distance
        isEmpty[color] = circleIMG.empty();

        if (isEmpty[color])
        {
            if (color == RED && objDist[RED] <= 0.5 && objDist[RED] != 0.0)
            {
                hasRedBall = true;
                isEmpty[RED] = false;
                return;
            }
            else
            {
                objDist[color] = 0.0;
                if (color == GREEN)
		{
                    moveTurtleBot(true);
		}
            }
        }

        // if we don't have the red ball and the image does not contain red, rotate the TurtleBot until red ball is found
        if (color == RED && isEmpty[RED] && !hasRedBall)
            moveTurtleBot(true);

        else
        {
            for (size_t i = 0; i < circleIMG.size(); i++)
            {
                // x and y coordinates of circle center, and radius
                objCoord[color][X] = static_cast<int>(round(circleIMG[i][0]));
                objCoord[color][Y] = static_cast<int>(round(circleIMG[i][1]));
                objCoord[color][R] = static_cast<int>(round(circleIMG[i][2]));

                cv::Point center(objCoord[color][X], objCoord[color][Y]);

                // draws circle around ball and cross-hair at center
                cv::circle(srcIMG, center, objCoord[color][R], black, 2);
                cv::line(srcIMG, center, center, black, 2);

                objDist[color] = distFromObj(objCoord[color][R]);

                if (color == RED)
                {
                    // we do not have the red ball - go to it
                    alignError = objCoord[RED][X] - (IMG_WIDTH_PX / 2.0);
                    moveTurtleBot();
                }

                else
                {
                    // kicker has the red ball -- now make sure that we are centered on green ball
                    alignError = objCoord[GREEN][X] - (IMG_WIDTH_PX / 2.0);
                    moveTurtleBot();
                }
            }
        }
    }

    // method to have the robot see if it can find either the red or green ball
    void searchForBall(const sensor_msgs::ImageConstPtr &msg)
    {

        cv_bridge::CvImagePtr cvPtr;
        cv_bridge::CvImagePtr cvGrayPtr;
        std::vector<cv::Vec3f> circleIMG, redCircleIMG, greenCircleIMG;
        cv::Mat srcIMG, hsvIMG, redIMG_lower, redIMG_upper, redIMG, greenIMG_lower, greenIMG_upper, greenIMG, greenRange;

        try
        {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            srcIMG = cvPtr->image;

            // converting color to HSV
            cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

            // define color limits, image weighting, blurring, and circle detection
            if (!hasRedBall)
            {
                cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), redIMG_lower);
                cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255), redIMG_upper);
                cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);
                cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);
                cv::HoughCircles(redIMG, redCircleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
            }

            else
            {
		cv::inRange(hsvIMG, cv::Scalar(45, 100, 100), cv::Scalar(75, 255, 255), greenRange);
                cv::addWeighted(greenRange, 1.0, greenRange, 1.0, 0.0, greenIMG);
                cv::GaussianBlur(greenIMG, greenIMG, cv::Size(9, 9), 2, 2);
                cv::HoughCircles(greenIMG, greenCircleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
            }
        }

        catch (cv_bridge::Exception &exception)
        {
            ROS_ERROR("cv_bridge exception: %s", exception.what());
            return;
        }

        if (!hasRedBall)
            trackBall(redCircleIMG, srcIMG, RED);

        else
            trackBall(greenCircleIMG, srcIMG, GREEN);

        // Update GUI Window and publish modified stream
        cv::imshow(OPENCV_WINDOW, cvPtr->image);
        cv::waitKey(3);
        imagePub_.publish(cvPtr->toImageMsg());
    }

    // method where the robot decides which action to take (try to make goal or search for ball)
    void playSoccer(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!inGame)
            return;

        searchForBall(msg);
    }

    // method to handle game commands
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

    ros::init(argc, argv, "kicker_collect");
    KickerRobot kicker;
    ros::spin();

    return 0;
}

