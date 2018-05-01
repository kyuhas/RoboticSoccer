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

// constants based on the position of the goal
const double goalLowerX = 0.1;
const double goalUpperX = 0.9;
const double goalLowerY = 1.2;
const double goalUpperY = 3.0;
const double goalCenterY = (goalUpperY + goalLowerY) / 2.0;

static const std::string OPENCV_WINDOW = "Image window";
static double currObjDist = 0.0, alignmentError = 0.0, botVelX = 0.0;
static int xCoord, yCoord, radius;

cv::Scalar black = (0, 255, 5), blue = (200, 200, 250);                  // RGB color for circle to be drawn on image
static std::vector<double> objDist = {0.0, 0.0};                         // RED, BLUE in that order
static std::vector<std::vector<int>> objCoords = {{0, 0, 0}, {0, 0, 0}}; // RED, BLUE, in that order

//constants used throughout the program
#define RGB_FOCAL_LEN_MM 138.90625 // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 200.0         // 8" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0     // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0        // in pixels
#define IMG_WIDTH_PX 640.0         // in pixels
#define MAX_BOT_VEL 0.65           // max speed TurtleBot is capable of
#define MIN_BOT_VEL 0.2            // the min speed I want the TurtleBot to go
#define RED 0
#define BLUE 1
#define MIN_RADIUS 0
#define MAX_RADIUS 0
#define X 0
#define MID_X_LOW 200
#define MID_X_HIGH 450
#define LOW_LEFT_ANGLE_Z -0.8
#define HIGH_LEFT_ANGLE_Z -0.7
#define LOW_RIGHT_ANGLE_Z 0.7
#define HIGH_RIGHT_ANGLE_Z 0.85
#define LOW_RIGHT_ANGLE_W 0.6
#define HIGH_RIGHT_ANGLE_W 0.7
#define HIGH_STRAIGHT_ANGLE_Z -0.9

bool inGame, goalSet, haveTurnedToSide, haveTurnedBackToCenter, haveMovedForward, isBlocking, isBlockingOnLeft;

class GoalieRobot
{
        ros::NodeHandle nodeHandle_;
        image_transport::ImageTransport imageTransport_;
        image_transport::Subscriber imageSub_;
        image_transport::Publisher imagePub_;
        ros::Subscriber gameSub_ = nodeHandle_.subscribe("/gameCommands", 10, &GoalieRobot::gameCommandCallback, this);
        ros::Subscriber amclSub_ = nodeHandle_.subscribe("/amcl_pose", 10, &GoalieRobot::setPose, this);
        ros::Subscriber odomSub_ = nodeHandle_.subscribe("/odom", 1000, &GoalieRobot::odomCallback, this);
        ros::Publisher velPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
        geometry_msgs::Twist twistMsg;
        geometry_msgs::Pose goaliePos;

        ros::Subscriber mbcSub = nodeHandle_.subscribe("/move_base_controller_result", 10, &GoalieRobot::mbControllerResultCallback, this);
        ros::Publisher mbcPub = nodeHandle_.advertise<move_base_msgs::MoveBaseGoal>("/goal_location", 1);

        int haveMovedForwardCount;
        double expectedLocation;

      public:
            //constructor
            GoalieRobot() : imageTransport_(nodeHandle_)
            {
                // real camera
                imageSub_ = imageTransport_.subscribe("/usb_cam/image_raw", 10, &GoalieRobot::playSoccer, this);

                // simulation camera
                //imageSub_ = imageTransport_.subscribe("/camera/rgb/image_raw", 10, &GoalieRobot::playSoccer, this);
                imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

                //initialize bools
                //inGame = false;
                inGame = true;
                goalSet = false;
                haveTurnedToSide = false;
                haveTurnedBackToCenter = false;
                haveMovedForward = false;
                isBlocking = false;
                isBlockingOnLeft = false;

                haveMovedForwardCount = 0;
            }
            //destructor
            ~GoalieRobot() {}

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

            // method to set the pose of the goalie robot. If out of goal bounds, move
            // to the center of the goal
            void setPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
            {
                goaliePos = msg->pose.pose;
                /*
			        if(goaliePos.position.x < goalLowerX || goaliePos.position.x > goalUpperX
				        || goaliePos.position.y < goalLowerY || goaliePos.position.y > goalUpperY) {
				        moveToLocation(getStartingLocation());
			        }
			        */
            }

            void rotateGoalie(bool rotateLeft = false)
            {
                // if you have not yet turned to the side
                if (!haveTurnedToSide)
                {
                    if (rotateLeft)
                    {

                        if (goaliePos.orientation.z >= LOW_LEFT_ANGLE_Z && goaliePos.orientation.z <= HIGH_LEFT_ANGLE_Z)
                        {
                            twistMsg.angular.z = 0;
                            twistMsg.linear.x = 0;
                            velPub.publish(twistMsg);
                            haveTurnedToSide = true;
                        }

                        else
                        {
                            twistMsg.angular.z = 0.5;
                            twistMsg.linear.x = 0;
                            velPub.publish(twistMsg);
                        }
                    }
                    else
                    {
                        if (goaliePos.orientation.z >= LOW_RIGHT_ANGLE_Z && goaliePos.orientation.z <= HIGH_RIGHT_ANGLE_Z)
                        {
                            twistMsg.angular.z = 0;
                            twistMsg.linear.x = 0;
                            velPub.publish(twistMsg);
                            haveTurnedToSide = true;
                        }
                        else
                        {
                            twistMsg.angular.z = -0.5;
                            twistMsg.linear.x = 0;
                            velPub.publish(twistMsg);
                        }
                    }
                    return;
                }

                // if the turn goal has finished and we have not yet moved forward three times
                if (haveTurnedToSide && !haveMovedForward)
                {
                    if (haveMovedForwardCount == 0)
                    {
                        if (rotateLeft)
                            expectedLocation = goaliePos.position.y - 0.5;
                        else
                            expectedLocation = goaliePos.position.y + 0.5;
                    }

                    if ((rotateLeft && goaliePos.position.y <= expectedLocation) || (!rotateLeft && goaliePos.position.y >= expectedLocation))
                    {
                        twistMsg.linear.x = 0;
                        twistMsg.angular.z = 0;
                        velPub.publish(twistMsg);
                        haveMovedForward = true;
                        //haveTurnedToSide = false;
                    }

                    else
                    {
                        twistMsg.angular.z = 0;
                        twistMsg.linear.x = 0.5;
                        velPub.publish(twistMsg);
                        haveMovedForwardCount++;
                    }

                    return;
                }

                // if we have finished moving forward
                if (haveMovedForward && !haveTurnedBackToCenter)
                {
                    if (goaliePos.orientation.w <= 0.1)
                    {
                        twistMsg.angular.z = 0;
                        twistMsg.linear.x = 0;
                        velPub.publish(twistMsg);
                        haveTurnedBackToCenter = true;
                    }
                    else
                    {
                        twistMsg.linear.x = 0;
                        twistMsg.angular.z = rotateLeft ? -0.5 : 0.5;
                        velPub.publish(twistMsg);
                    }

                    return;
                }

                // if we have finished turning back to center
                if (haveTurnedBackToCenter)
                {
                    ROS_INFO("done rotating");
                    twistMsg.angular.z = 0;
                    twistMsg.linear.x = 0;
                    velPub.publish(twistMsg);

                    isBlocking = false;
                    // reset all of the booleans and count
                    haveTurnedBackToCenter = false;
                    haveTurnedToSide = false;
                    haveMovedForward = false;
                    haveMovedForwardCount = 0;
                    return;
                }
            }

            // method to have the robot move so that it can see the ball
            void moveTurtleBot()
            {
                // tune this value later
                if (objDist[RED] <= 1.5 && objDist[RED] > 0.0) //make sure it isn't counting empty images
                {
                    // see which side the red ball is on
                    isBlocking = true;

                    // if the red ball is approaching from left or right
                    if (objCoords[RED][X] <= MID_X_LOW || objCoords[RED][X] >= MID_X_HIGH)
                    {
                        isBlockingOnLeft = (objCoords[RED][X] <= MID_X_LOW);

                        // if goalie can move left
                        if (isBlockingOnLeft && goaliePos.position.y > 1.8)
                            rotateGoalie(true);

                        // if goalie can move right
                        if (!isBlockingOnLeft && goaliePos.position.y < 2.6)
                            rotateGoalie(false);

                        return;
                    }

                    else
                        isBlocking = false;
                }

                // if the red ball is further than 1.5 meters away, we are no longer blocking
                isBlocking = false;
            }

            //method to get the current velocity of the robot
            void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
            {
                // robot linear and angular velocity rounded to the nearest 100th
                botVelX = floor((msg->twist.twist.linear.x * 100 + 0.5)) / 100;
            }

            //the ball is getting close to the goal
            bool redBallApproaching(int radius)
            {
                return (radius > 30);
            }

            bool redIsEmpty()
            {
                return objDist[RED] == 0.0;
            }

            void trackBall(std::vector<cv::Vec3f> circleIMG, cv::Mat srcIMG)
            {
                if (isBlocking)
                {
                    rotateGoalie(isBlockingOnLeft);
                    return;
                }

                if (circleIMG.empty())
                    objDist[RED] = 0.0;

                else
                    for (size_t i = 0; i < circleIMG.size(); i++)
                    {
                        // center coordinates of circle, and the radius
                        xCoord = static_cast<int>(round(circleIMG[i][0]));
                        yCoord = static_cast<int>(round(circleIMG[i][1]));
                        radius = static_cast<int>(round(circleIMG[i][2]));

                        objCoords[RED][0] = xCoord;
                        objCoords[RED][1] = yCoord;
                        objCoords[RED][2] = radius;

                        cv::Point center(xCoord, yCoord);

                        // draws circle around ball and cross-hair at center
                        cv::circle(srcIMG, center, radius, black, 2);
                        cv::line(srcIMG, center, center, black, 2);

                        objDist[RED] = distFromObj(radius);

                        // move the turtlebot
                        moveTurtleBot();
                    }
            }

            //method to have the robot search for the ball and move toward it
            void searchForBall(const sensor_msgs::ImageConstPtr &msg)
            {

                cv_bridge::CvImagePtr cvPtr;
                std::vector<cv::Vec3f> circleIMG;
                cv::Mat srcIMG, hsvIMG, redIMG_lower, redIMG_upper, redIMG;
                std::vector<cv::Vec3f> redCircleIMG;

                try
                {
                    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                    srcIMG = cvPtr->image;

                    // converting color to HSV
                    cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

                    // defining upper and lower red color range
                    cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), redIMG_lower);
                    cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255), redIMG_upper);

                    // weighting image and performing blur to reduce noise in image
                    cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);
                    cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);

                    // Hough gradient transform to find circles
                    cv::HoughCircles(redIMG, redCircleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
                }

                catch (cv_bridge::Exception &exception)
                {
                    ROS_ERROR("cv_bridge exception: %s", exception.what());
                    return;
                }

                trackBall(redCircleIMG, srcIMG);

                // Update GUI Window
                cv::imshow(OPENCV_WINDOW, cvPtr->image);
                cv::waitKey(3);
                // Output modified video stream1
                imagePub_.publish(cvPtr->toImageMsg());
            }

            //method to send the ball to a specific location
            void moveToLocation(move_base_msgs::MoveBaseGoal goal)
            {
                goalSet = true;
                mbcPub.publish(goal);
            }

            move_base_msgs::MoveBaseGoal getStartingLocation()
            {
                move_base_msgs::MoveBaseGoal startingLocation;
                startingLocation.target_pose.header.frame_id = "base_link";
                startingLocation.target_pose.header.stamp = ros::Time::now();
                startingLocation.target_pose.pose.position.x = (goalLowerX + goalUpperX) / 2.0;
                startingLocation.target_pose.pose.position.y = 1.0;
                startingLocation.target_pose.pose.position.z = goaliePos.position.z;
                startingLocation.target_pose.pose.orientation.z = -1;
                return startingLocation;
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
                //inGame = false;

                ROS_INFO("In GC callback");
                //inGame = true;
                /*

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
                    //publish command to go to the field
                    moveToLocation(getStartingLocation());
                }
                */
            }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "goalie_robot");
    GoalieRobot goalie;
    ros::spin();

    return 0;
}

