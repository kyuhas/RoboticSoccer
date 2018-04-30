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

//<node name="kicker_collect" pkg="final_project" type="kicker_collect_ball" />

//constants used throughout the program
#define RGB_FOCAL_LEN_MM 138.90625 // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 203.2         // 8" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0     // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0        // in pixels
#define IMG_WIDTH_PX 640.0         // in pixels
#define MAX_BOT_VEL 0.65           // max speed TurtleBot is capable of
#define MIN_BOT_VEL 0.2            // the min speed I want the TurtleBot to go
#define RED 0
#define BLUE 1
#define X 0
#define Y 1
#define R 2
#define MIN_RADIUS 0
#define MAX_RADIUS 0
#define MID_X_LOW 310
#define MID_X_HIGH 330

// constant time for how frequently to check if the ball is still there
const int NUM_SECONDS = 5;

// constants based on the position of the goal
const double goalLowerX = 0.1;
const double goalUpperX = 0.9;
const double goalLowerY = 1.2;
const double goalUpperY = 3.0;
const double goalCenterY = (goalUpperY + goalLowerY) / 2.0;

static const std::string OPENCV_WINDOW = "Image window";

class KickerRobot
{
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;
    ros::Subscriber gameSub_ = nodeHandle_.subscribe("/gameCommands", 10, &KickerRobot::gameCommandCallback, this);
    ros::Subscriber odomSub_ = nodeHandle_.subscribe("/odom", 1000, &KickerRobot::odomCallback, this);
    ros::Subscriber amclSub_ = nodeHandle_.subscribe("/amcl_pose", 10, &KickerRobot::amclPoseCallback, this);
    ros::Subscriber mbcSub = nodeHandle_.subscribe("/move_base_controller_result", 10, &KickerRobot::mbControllerResultCallback, this);
    ros::Publisher velPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    ros::Publisher mbcPub = nodeHandle_.advertise<move_base_msgs::MoveBaseGoal>("/goal_location", 1);

    geometry_msgs::Twist twistMsg;
    geometry_msgs::Pose kickerPos;

    double alignErrorRed = 0.0, alignErrorBlue = 0.0, botVelX = 0.0, botAngZ = 0.0;

    cv::Scalar black = (0, 255, 5), blue = (200, 200, 250);          // RGB color for circle to be drawn on image
    std::vector<double> objDist = {0.0, 0.0};                        // RED, BLUE in that order
    std::vector<std::vector<int>> objCoord = {{0, 0, 0}, {0, 0, 0}}; // RED, BLUE, in that order
    std::vector<bool> isEmpty = {false, false};                      // RED, BLUE, in that order
    bool isKickingBall, inGame, goalSet, hasRedBall, haveTurned, haveMovedBack, haveMovedForward;

    clock_t thisTime, lastTime;
    double timeCounter;

  public:
    // constructor
    KickerRobot() : imageTransport_(nodeHandle_)
    {
        // real camera
        imageSub_ = imageTransport_.subscribe("/usb_cam/image_raw", 10, &KickerRobot::playSoccer, this);

        // camera used for simulation
        //imageSub_ = imageTransport_.subscribe("/camera/rgb/image_raw", 10, &KickerRobot::playSoccer, this);
        imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

        // initialize clock vars
        thisTime = clock();
        lastTime = thisTime;
        timeCounter = 0;

        // for testing purposes only
        inGame = true;
        //inGame = false;

        // initialize booleans because C++ does not do this for us
        isKickingBall = false;

        goalSet = false;
        haveTurned = false;
        haveMovedBack = false;
        haveMovedForward = false;
        hasRedBall = false;
    }
    // destructor
    ~KickerRobot() {}

    // method to find the distance the robot is from the ball
    double distFromObj(int objSize)
    {
        double distMeters = (((RGB_FOCAL_LEN_MM * BALL_DIAM_MM * IMG_HEIGHT_PX) / (objSize * CAMERA_HEIGHT_MM)) / 1000);
        return floor((distMeters * 10 + 0.5)) / 10;
    }

    // if we have reached our goal, goalSet is now false
    void mbControllerResultCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (goalSet && strcmp(msg->data.c_str(), "true") == 0)
            goalSet = false;
    }

    // method to set the pose of the kicker robot. If it gets too close to the goalie, make it go backwards.
    // TODO: maybe also add a check if it gets too close to any walls?
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        kickerPos = msg->pose.pose;

        if (kickerPos.position.x < goalUpperX + 1.0)
        {
            // TODO: uncomment once debugging is done
            //twistMsg.linear.x = -0.5;
            //velPub.publish(twistMsg);
        }
    }

    // method to send the ball to a specific location
    void moveToLocation(move_base_msgs::MoveBaseGoal goal)
    {
        goalSet = true;
        mbcPub.publish(goal);
    }

    // method to have the robot move to the ball's location or look for the ball
    void moveTurtleBot(bool rotate = false, bool alignBlueBall = false)
    {
        if (goalSet)
        {
            ROS_INFO("A goal has been set, returning");
            return;
        }

        twistMsg.angular.z = -alignErrorRed / 225.0; // 225 worked well as a denominator to smooth the alignment

        if (rotate)
        {
            twistMsg.angular.z = 0.5;
            twistMsg.linear.x = 0.0;
            //ROS_INFO("I am rotating");
        }

        else if (alignBlueBall)
        {
            twistMsg.angular.z = -alignErrorBlue / 225.0;
            twistMsg.linear.x = 0.0;
            //ROS_INFO("Rotating so that I am centered on blue ball");
        }

        else if (!hasRedBall)
        {
            twistMsg.linear.x = (0.3 * objDist[RED]);
            ROS_INFO("Going to get the red ball.");
        }

        else if (hasRedBall && isEmpty[BLUE])
        {
            ROS_INFO("Looking for the blue ball");
            twistMsg.angular.z = 0.2;
            twistMsg.linear.x = 0.0;
        }

        else if (hasRedBall && !isEmpty[BLUE])
        {
            ROS_INFO("I have the red ball and can see the goal");
            // if you are far away from the goal, move toward it. otherwise, try to kick the ball
            if (objDist[BLUE] > 1.5)
                twistMsg.linear.x = MAX_BOT_VEL;

            else
            {
                twistMsg.linear.x = 0.0;
                // inGame = false? // to be tested
            }
        }

        if (twistMsg.linear.x > MAX_BOT_VEL)
            twistMsg.linear.x = MAX_BOT_VEL;

        velPub.publish(twistMsg);
    }

    // method to get the current velocity of the robot
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // robot linear and angular velocity rounded to the nearest 100th
        botVelX = floor((msg->twist.twist.linear.x * 100 + 0.5)) / 100;
        botAngZ = floor((msg->twist.twist.angular.z * 100 + 0.5)) / 100;
    }

    // returns true if goalie on kicker's left side of goal
    // returns false if goalie on kicker's right side of goal
    bool goalieOnLeft(double dist)
    {
        ROS_INFO("Checking to see which side the goalie is on");
        // get the robot's orientation in radians
        double theta = 2 * acos(kickerPos.orientation.w);
        double dx = dist * cos(theta);
        double dy = dist * sin(theta);
        double goalieXPos = kickerPos.position.x + dx;
        double goalieYPos = kickerPos.position.y + dy;

        // we will determine if the ball if coming in from the left or right based on the ballYPos
        return goalieYPos < goalCenterY;
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
                ROS_INFO("GOT DAT RED BALL, YO");
                return;
            }
            else
            {
                objDist[color] = 0.0;
                if (color == BLUE)
                    moveTurtleBot(true);
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
                    alignErrorRed = objCoord[RED][X] - (IMG_WIDTH_PX / 2.0);
                    moveTurtleBot();
                }

                else
                {
                    ROS_INFO("i see a blue circle. deciding what to do.");
                    // kicker has the red ball -- now make sure that we are centered on blue ball
                    alignErrorBlue = objCoord[BLUE][X] - (IMG_WIDTH_PX / 2.0);
                    moveTurtleBot(false, true);

                    // if the blue ball is close to the center of the frame, move turtlebot
                    if (objCoord[BLUE][X] <= MID_X_HIGH && objCoord[BLUE][X] >= MID_X_LOW)
                        ROS_INFO("I AM ALIGNED WITH THE BLUE BALL");
                    //moveTurtleBot();
                }

                // for easy debugging in Gazebo, please wait until final code is pushed to delete
                std::stringstream ssRedDist, ssBlueDist, ssBotVelX, ssAlignError, ssBotOrient, ssBotPosX, ssBotPosY;
                ssAlignError << alignErrorRed;
                ssRedDist << objDist[RED];
                ssBlueDist << objDist[BLUE];
                ssBotVelX << botVelX;
                ssBotOrient << kickerPos.orientation.w;
                ssBotPosX << kickerPos.position.x;
                ssBotPosY << kickerPos.position.y;

                std::string alignErrStr = "    ALN_ERR: " + ssAlignError.str() + " px";
                std::string redDistStr = "    RED_DST: " + ssRedDist.str() + " m";
                std::string blueDistStr = "    BLU_DST: " + ssBlueDist.str() + " m";
                std::string botVelStr = "    BOT_VEL: " + ssBotVelX.str() + " m/s";
                std::string botPosXStr = "    POS__X: " + ssBotPosX.str();
                std::string botPosYStr = "    POS__Y: " + ssBotPosY.str();
                std::string botOrientStr = "    POS__W: " + ssBotOrient.str();

                cv::putText(srcIMG, alignErrStr, cv::Point(350, 325), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
                cv::putText(srcIMG, redDistStr, cv::Point(350, 350), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
                cv::putText(srcIMG, blueDistStr, cv::Point(350, 375), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
                cv::putText(srcIMG, botVelStr, cv::Point(350, 400), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
                cv::putText(srcIMG, botPosXStr, cv::Point(350, 450), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
                cv::putText(srcIMG, botPosYStr, cv::Point(350, 475), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
                cv::putText(srcIMG, botOrientStr, cv::Point(350, 425), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1, CV_AA);
            }
        }
    }

    // method to have the robot see if it can find either the red or blue ball
    void searchForBall(const sensor_msgs::ImageConstPtr &msg)
    {

        cv_bridge::CvImagePtr cvPtr;
        cv_bridge::CvImagePtr cvGrayPtr;
        std::vector<cv::Vec3f> circleIMG, redCircleIMG, blueCircleIMG;
        cv::Mat srcIMG, hsvIMG, redIMG_lower, redIMG_upper, redIMG, blueIMG_lower, blueIMG_upper, blueIMG;

        cv::Scalar black = (0, 255, 5);    // RGB color for circle to be drawn on image
        cv::Scalar blue = (200, 200, 250); // RGB color for text displayed on image

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
                cv::inRange(hsvIMG, cv::Scalar(60, 100, 100), cv::Scalar(80, 255, 255), blueIMG_lower);
                cv::inRange(hsvIMG, cv::Scalar(100, 100, 100), cv::Scalar(120, 255, 255), blueIMG_upper);
                cv::addWeighted(blueIMG_lower, 1.0, blueIMG_upper, 1.0, 0.0, blueIMG);
                cv::GaussianBlur(blueIMG, blueIMG, cv::Size(9, 9), 2, 2);
                cv::HoughCircles(blueIMG, blueCircleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
            }
        }

        catch (cv_bridge::Exception &exception)
        {
            ROS_ERROR("cv_bridge exception: %s", exception.what());
            return;
        }

        if (!hasRedBall)
        {
            trackBall(redCircleIMG, srcIMG, RED);
        }

        else
            trackBall(blueCircleIMG, srcIMG, BLUE);

        // Update GUI Window and publish modified stream
        cv::imshow(OPENCV_WINDOW, cvPtr->image);
        cv::waitKey(3);
        imagePub_.publish(cvPtr->toImageMsg());
    }

    // method to get the starting location of the kicker
    move_base_msgs::MoveBaseGoal getStartingLocation()
    {
        move_base_msgs::MoveBaseGoal startingLocation;

        startingLocation.target_pose.header.frame_id = "base_link";
        startingLocation.target_pose.header.stamp = ros::Time::now();
        startingLocation.target_pose.pose.position.x = (goalLowerX + goalUpperX) / 2.0;
        startingLocation.target_pose.pose.position.y = 6.0; //TODO: maybe change this later?
        startingLocation.target_pose.pose.orientation.w = 1;

        return startingLocation;
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
        /*
        inGame = false;

        if (strcmp(msg->data.c_str(), "start") == 0)
            inGame = true;

        else if (strcmp(msg->data.c_str(), "stop") == 0)
        {
            twistMsg.linear.x = 0;
            twistMsg.angular.z = 0;
            velPub.publish(twistMsg);
        }
        else if (strcmp(msg->data.c_str(), "field") == 0)
        {
            // publish command to go to the field
            move_base_msgs::MoveBaseGoal startingLocation = getStartingLocation();
            moveToLocation(startingLocation);
        }
        */
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "kicker_collect");
    KickerRobot kicker;
    ros::spin();

    return 0;
}

