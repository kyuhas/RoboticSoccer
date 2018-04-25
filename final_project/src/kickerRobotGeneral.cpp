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
#include <actionlib/client/simple_action_client.h>
#include <time.h>
#include <math.h>

// constant time for how frequently to check if the ball is still there
const int NUM_SECONDS = 5;

// constants based on the position of the goal
const double goalLowerX = 0.1;
const double goalUpperX = 0.9;
const double goalLowerY = 1.2;
const double goalUpperY = 3.0;
const double goalCenterY = (goalUpperY + goalLowerY)/2.0;

//constants used throughout the program
#define RGB_FOCAL_LEN_MM 138.90625 // camera focal length in mm ... 525 pixels
#define BALL_DIAM_MM 200.0 // 8" diameter ball in mm
#define CAMERA_HEIGHT_MM 300.0 // height of camera off ground in mm
#define IMG_HEIGHT_PX 480.0 // in pixels
#define IMG_WIDTH_PX 640.0 // in pixels
#define MAX_BOT_VEL 0.65 // max speed TurtleBot is capable of
#define MIN_BOT_VEL 0.2 // the min speed I want the TurtleBot to go
#define RED 0
#define BLUE 1
#define YELLOW 2
#define MIN_RADIUS 0
#define MAX_RADIUS 0

static const std::string OPENCV_WINDOW = "Image window";
static double currObjDist = 0.0, alignmentError = 0.0, botVelX = 0.0;
static int xCoord, yCoord, radius, sequence;

cv::Scalar black = (0, 255, 5), blue = (200, 200, 250); // RGB color for circle to be drawn on image
static std::vector<double> objDist = {0.0, 0.0, 0.0}; // RED, BLUE, YELLOW in that order
static std::vector<std::vector<int> > objCoords = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // RED, BLUE, (HOLDER FOR AMCL INFO) in that order
static cv::Mat redIMG, blueIMG, yellowIMG;
std::vector<cv::Vec3f> redCircleIMG, blueCircleIMG;

static bool ballIsInFrontOfRobot;
static bool inFrontOfGoal;
static bool isKickingBall;
static bool inGame;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class KickerRobot {
	ros::NodeHandle nodeHandle_;
    	image_transport::ImageTransport imageTransport_;
    	image_transport::Subscriber imageSub_;
    	image_transport::Publisher imagePub_;
	ros::Subscriber gameSub_ = nodeHandle_.subscribe("/gameCommands", 10, &KickerRobot::gameCommandCallback, this);
  	ros::Subscriber odomSub_ = nodeHandle_.subscribe("/odom", 1000, &KickerRobot::odomCallback, this);
  	ros::Subscriber amclSub_ = nodeHandle_.subscribe("/amcl_pose", 10, &KickerRobot::setPose, this);
	ros::Publisher velPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    	geometry_msgs::Twist twistMsg;
    	geometry_msgs::Pose kickerPos;
    	bool isKickingBall;
    	bool ballIsInFrontOfRobot;
    	clock_t this_time;
    	clock_t last_time;
    	double time_counter;

  	public:
		// constructor
		KickerRobot() : imageTransport_(nodeHandle_) {
			imageSub_ = imageTransport_.subscribe("/usb_cam/image_raw", 10, &KickerRobot::playSoccer, this);
			imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);
			
			// initialize clock vars
			this_time = clock();
			last_time = this_time;
			time_counter = 0;
		}
		// destructor
		~KickerRobot() {}

		// method to find the distance the robot is from the ball
        	double distFromObj(int objSize){
		    	double distMeters = (((RGB_FOCAL_LEN_MM * BALL_DIAM_MM * IMG_HEIGHT_PX)/(objSize * CAMERA_HEIGHT_MM))/1000);
		    	return floor((distMeters*10 + 0.5))/10;
		}

		
		// method to set the pose of the kicker robot. If it gets too close to the goalie, make it go backwards. 
		// TODO: maybe also add a check if it gets too close to any walls?
		void setPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {			
			kickerPos = msg->pose.pose;
			if(kickerPos.position.y > goalUpperY + 1.0) {
				twistMsg.linear.x = -0.5;
				velPub.publish(twistMsg);
			}
		}
		
		// method to actually kick the ball into the goal
		// TODO: edit this method further
		void kickBall() {
			// move backward
			twistMsg.linear.x = -0.5;
			velPub.publish(twistMsg);
			//TODO: should we add a time pause here?
			// move forward quickly
			twistMsg.linear.x = 1.0;
			velPub.publish(twistMsg);
		}

		// method to have the robot move to the ball's location or look for the ball
		void moveTurtleBot(bool rotate = false){

            		if (rotate){
                		twistMsg.angular.z = 0.5;
                		twistMsg.linear.x = 0.0;
                		velPub.publish(twistMsg);
                		return;
            		}

			if (isKickingBall) return; //do not move the robot with this method if the ball is being kicked

            		twistMsg.angular.z = -alignmentError/225.0; // 225 worked well as a denominator to smooth the alignment

			if (!hasRedBall()) { 
				twistMsg.linear.x = ((0.3 * objDist[RED]) + MIN_BOT_VEL);
			}
			
			else if(hasRedBall() && inFrontOfGoal) {
				kickBall();
			}
			
            		if (twistMsg.linear.x > MAX_BOT_VEL) twistMsg.linear.x = MAX_BOT_VEL;

            		velPub.publish(twistMsg);
		}

		// method to get the current velocity of the robot
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
			// robot linear and angular velocity rounded to the nearest 100th
			botVelX = floor((msg->twist.twist.linear.x*100 + 0.5))/100;
		}


		// check to see if the ball has been "captured" by the robot
        	bool hasRedBall(){
            		return (objDist[RED] <= 0.4 && (!redCircleIMG.empty() || !redIMG.empty()));
		}


		// method to have the kicker face the goal
       		void lookAtGoal(){
			move_base_msgs::MoveBaseGoal goal; 
			goal.target_pose.header.frame_id = "base_link";
  			goal.target_pose.header.stamp = ros::Time::now();
  			goal.target_pose.pose.orientation.w = 1;
			moveToLocation(goal);
        	}

		// method that gets called to periodically see if the robot still has the ball
		void periodicCheckForBall() {
			// check to see if the ball is still in front of the robot
            		ballIsInFrontOfRobot = hasRedBall();
		}

		// returns true if goalie on kicker's left side of goal
		// returns false if goalie on kicker's right side of goal
		bool determineGoalieLeftOrRight(double dist) {
			// get the robot's orientation in radians
			double theta = 2*acos(kickerPos.orientation.w);
			double dx = dist*cos(theta);
			double dy = dist*sin(theta);
			double goalieXPos = kickerPos.position.x + dx;
			double goalieYPos = kickerPos.position.y + dy;
			// we will determine if the ball if coming in from the left or right based on the ballYPos
			if(goalieYPos < goalCenterY) {
				return false;
			}
			else return true;
		}

		// method to track the ball
	    	void trackBall(std::vector<cv::Vec3f> circleIMG, cv::Mat srcIMG, cv::Mat hsvIMG, int colorFlag){

			// check to see if the ball is still there
            		ballIsInFrontOfRobot = hasRedBall();
            
            		if (colorFlag == BLUE) {
				// if you do not yet see the blue ball, turn to face the goal
				if (circleIMG.empty()) lookAtGoal(); 
				return;
            		}

		    	// colorFlag = 0, 1, or 2. This corresponds to the global static vector "objDist" indices
		    	//if (blueCircleIMG.empty()) objDist[BLUE] = 0.0;

		    	if (redCircleIMG.empty()) moveTurtleBot(true);

		    	else for (size_t i = 0; i < circleIMG.size(); i++){
				// center coordinates of circle, and the radius
			    	xCoord = static_cast<int>(round(circleIMG[i][0]));
				yCoord = static_cast<int>(round(circleIMG[i][1]));
				radius = static_cast<int>(round(circleIMG[i][2]));

				objCoords[colorFlag][0] = xCoord;
				objCoords[colorFlag][1] = yCoord;
				objCoords[colorFlag][2] = radius;

				cv::Point center(xCoord, yCoord);

				// draws circle around ball and cross-hair at center
				cv::circle(srcIMG, center, radius, black, 2);
				cv::line(srcIMG, center, center, black, 2);

				objDist[colorFlag] = distFromObj(radius);

				if (colorFlag == RED && !ballIsInFrontOfRobot){
				    	// we do not have the red ball - go to it
					alignmentError = objCoords[RED][0] - (IMG_WIDTH_PX / 2.0);;
					moveTurtleBot();
				}
				else if (colorFlag == BLUE) {
					// at this point, the blue ball should be centered in the frame
					bool goalieInLeftSideOfGoal = determineGoalieLeftOrRight(objDist[BLUE]);
					if (goalieInLeftSideOfGoal) {
						// TODO: have the kicker aim to the right
					} 
					else {
						// TODO: have the kicker aim to the left
					}
				}
	    		}
	    	}

		// method to have the robot see if it can find either the red or blue ball
		void searchForBall(const sensor_msgs::ImageConstPtr& msg) {

            		cv_bridge::CvImagePtr cvPtr;
            		cv_bridge::CvImagePtr cvGrayPtr;
            		std::vector<cv::Vec3f> circleIMG;
            		cv::Mat srcIMG, hsvIMG, redIMG_lower, redIMG_upper, redIMG, blueIMG_lower, blueIMG_upper, blueIMG;

            		cv::Scalar black = (0, 255, 5); // RGB color for circle to be drawn on image
            		cv::Scalar blue = (200, 200, 250); // RGB color for text displayed on image

            		try {
                		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                		srcIMG = cvPtr->image;

                		// converting color to HSV
                		cv::cvtColor(srcIMG, hsvIMG, CV_BGR2HSV);

                		cv::inRange(hsvIMG, cv::Scalar(60, 100, 100), cv::Scalar(80, 255, 255), blueIMG_lower);
                		cv::inRange(hsvIMG, cv::Scalar(120, 100, 100), cv::Scalar(130, 255, 255), blueIMG_upper);

                		// defining upper and lower red color range
                		cv::inRange(hsvIMG, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), redIMG_lower);
                		cv::inRange(hsvIMG, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255), redIMG_upper);

                		// weighting image and performing blur to reduce noise in image
                		cv::addWeighted(blueIMG_lower, 1.0, blueIMG_upper, 1.0, 0.0, blueIMG);
                		cv::addWeighted(redIMG_lower, 1.0, redIMG_upper, 1.0, 0.0, redIMG);

                		cv::GaussianBlur(blueIMG, blueIMG, cv::Size(9, 9), 2, 2);
                		cv::GaussianBlur(redIMG, redIMG, cv::Size(9, 9), 2, 2);

                		// Hough gradient transform to find circles
                		cv::HoughCircles(blueIMG, blueCircleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
                		cv::HoughCircles(redIMG, circleIMG, CV_HOUGH_GRADIENT, 1, hsvIMG.rows / 8, 100, 20, MIN_RADIUS, MAX_RADIUS);
            		}

            		catch (cv_bridge::Exception &exception) {
                		ROS_ERROR("cv_bridge exception: %s", exception.what());
                		return;
            		}

            		trackBall(blueCircleIMG, srcIMG, hsvIMG, BLUE);
            		trackBall(redCircleIMG, srcIMG, hsvIMG, RED);
        	}

        	// method to send the ball to a specific location
        	bool moveToLocation(move_base_msgs::MoveBaseGoal goal) {
			MoveBaseClient ac("move_base", true);

			// wait for the action server to come up
            		while (!ac.waitForServer(ros::Duration(5.0))) {
                		ROS_INFO("Waiting for the move_base action server to come up");

            			ROS_INFO("Sending goal");

            			ac.sendGoal(goal);
            			ac.waitForResult();

            			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                			ROS_INFO("Hooray, the base moved to the correct location.");
                			return true;
            			}

            			else {
            				ROS_INFO("The base failed to move to the correct location.");
            				return false;
            			}
        		}
		}


		// method to get the starting location of the kicker
		move_base_msgs::MoveBaseGoal getStartingLocation() {
		    	move_base_msgs::MoveBaseGoal startingLocation;

		    	startingLocation.target_pose.header.frame_id = "base_link";
  			startingLocation.target_pose.header.stamp = ros::Time::now();
  			startingLocation.target_pose.pose.position.x = (goalLowerX + goalUpperX)/2.0;
  			startingLocation.target_pose.pose.position.y = 6.0; //TODO: maybe change this later?
  			startingLocation.target_pose.pose.orientation.w = 1;
  			
            		return startingLocation;
		}
		
		// method to get the kick location of the kicker 
		//TODO: we should figure out how many different kick positions it should have
		move_base_msgs::MoveBaseGoal getKickLocation() {
		    	move_base_msgs::MoveBaseGoal kickLoc;
		    	kickLoc.target_pose.header.frame_id = "base_link";
  			kickLoc.target_pose.header.stamp = ros::Time::now();
  			kickLoc.target_pose.pose.position.x = (goalLowerX + goalUpperX)/2.0;
  			kickLoc.target_pose.pose.position.y = 5.0; //TODO: maybe change this later?
  			kickLoc.target_pose.pose.orientation.w = 1;
            		return kickLoc;
		}

		// method where the robot decides which action to take (try to make goal or search for ball)
        	void playSoccer(const sensor_msgs::ImageConstPtr &msg) {
		    	if (!inGame) return;

			// if you have the ball -- either check to make sure it is there or kick it if you are not
			// already doing so 
            		if (ballIsInFrontOfRobot) {
				// add periodic check to make sure the ball is still there
                		time_counter += (double)(this_time - last_time);
                		if (time_counter > (double)(NUM_SECONDS * CLOCKS_PER_SEC)) {
                			// check to see if the ball is still in the robot's grasp
                			periodicCheckForBall();
                			time_counter -= (double)(NUM_SECONDS * CLOCKS_PER_SEC);
                		}
                		else {
                			if (!isKickingBall) {
						// assume we still have ball and go to the goal
						isKickingBall = true;
						bool didReachGoalPos = moveToLocation(getKickLocation());
						if (didReachGoalPos) kickBall();
						isKickingBall = false;
                			}
                		}
            			return;
		    	}

		    	else searchForBall(msg); // The ball is not yet in front of the robot. Go look for it!
        	}

        	// method to handle game commands
        	void gameCommandCallback(const std_msgs::String::ConstPtr &msg) {
		    	inGame = false;

		    	if (strcmp(msg->data.c_str(), "start") == 0) inGame = true;

		    	else if (strcmp(msg->data.c_str(), "stop") == 0) {
		    		twistMsg.linear.x = 0;
		    		twistMsg.angular.z = 0;
		    		velPub.publish(twistMsg);
		    	}
		    	else if (strcmp(msg->data.c_str(), "field") == 0) {
		        	// publish command to go to the field
                		move_base_msgs::MoveBaseGoal startingLocation = getStartingLocation();
                		moveToLocation(startingLocation);
		    	}
		}
	};

int main(int argc, char** argv){
    	ros::init(argc, argv, "kicker_robot");

    	KickerRobot kicker();

    	ros::spin();
   	return 0;
}

