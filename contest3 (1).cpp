#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <inttypes.h>

//BUMPER
#define N_BUMPER (3)

//BUMPER 
//GLOBAL VARIABLE FOR BUMPER 
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//State or nav related 
geometry_msgs::Twist follow_cmd;
int world_state;
int prev_state;
bool state_lockout = false;
bool obj_detected = false;

//Timer related global vars/structs
ros::Timer state_timer;
std::chrono::time_point<std::chrono::system_clock> state_start;
void set_start_time(); //Start time is set on state change
uint64_t get_time_elapsed(); //Returns millis since state start

bool enable_show_img = false;
int key_time = 0;
int minHessian = 800;
const float ratio_thresh = 0.75f;
//Preallocate memory needed for image matching to make things faster
std::vector<KeyPoint> keypoints_ref, keypoints_img;
Mat descriptor_ref, descriptor_img;
Ptr<SURF> detector = SURF::create( minHessian );
Mat img_ref;

bool matchImage(Mat inp_frame);
void excited();


void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
	if (follow_cmd.linear.x == 0 && follow_cmd.angular.z == 0){
		obj_detected=false;
	} else obj_detected=true;
}

//BUMPER
bool bumperPressed=false;

void bumperCB(const geometry_msgs::Twist msg){
    bumper[msg->bumper]=msg->state;// IDK IF THIS IS RIGHT 
    
    //if a bumper is pressed the robot enters a corresponding state and function to react
    if (bumper[0]==kobuki_msgs::BumperEvent::PRESSED) {
        world_state = 6;
        ROS_INFO("Left Bumper Pressed");
        bumperPressed=true;
    } else if (bumper[1]==kobuki_msgs::BumperEvent::PRESSED) {
        world_state = 7;
        ROS_INFO("Middle Bumper Pressed");
        bumperPressed=true;
    } else if (bumper[2]==kobuki_msgs::BumperEvent::PRESSED) {
        world_state = 8;
        ROS_INFO("Right Bumper Pressed");
        bumperPressed=true;
    } else {
        bumperPressed=false;
    }
}

void timerCB(const ros::TimerEvent& event){
	state_timer.stop();
	world_state = 0;
	state_lockout = false;
	ROS_INFO("Releasing lockout");
}

//-------------------------------------------------------------
void imshow_fast(Mat img){
	if (enable_show_img){
		imshow("image", img);
		cv::waitKey(key_time);
		cv::destroyAllWindows();
	}
	return;
}
void initAll(){ //Consider doing everything in grayscale it make it faster
	string path_to_imgref = ros::package::getPath("mie443_contest3") + "/imgs/imgref.jpg";
	img_ref = imread(path_to_imgref);
	detector -> detectAndCompute( img_ref, noArray(), keypoints_ref, descriptor_ref );
	imshow_fast(img_ref);
}

// Show image surprise 
void imshow_surprise(Mat img){
	if (enable_show_img){
		imshow("image", img);
		cv::waitKey(key_time); 
	}
	return;
//

double angular = 0.0;
double linear = 0.0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	state_timer = nh.createTimer(ros::Duration(10.0), timerCB);
	state_timer.stop();
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	//imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	world_state = 0;
	prev_state = 0;
	bool ranOnce=false;
	
	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	initAll();

	//sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();
	Mat img_test=imread(ros::package::getPath("mie443_contest3")+"/imgs/imgtest.jpg");
	imshow_fast(img_test);

	set_start_time();


	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();
		//State evaluation, put the most important states first! timerCB will automatically return to state 0 and reset state_lockout
		ROS_INFO("Time elapsed: %" PRIu64 "\n", get_time_elapsed());
		// if (!state_lockout && prev_state==0 && matchImage(rgbTransport.getImg())) {
		// 	world_state = 4;
		// 	sc.playWave(path_to_sounds + "excited.wav");
		// }  
		
		if (prev_state != world_state && world_state != 0){
			state_timer.start();
			state_lockout=true;
			set_start_time();
			ROS_INFO("Starting lockout");
		}
		prev_state=world_state;

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			ranOnce=false;
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			/*
			...
			...
			*/
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);

		}else if(world_state == 8){ 

			if (!ranOnce) {
				Mat img_test=imread(ros::package::getPath("mie443_contest3")+"/imgs/meow_surprised_yoga.jpg");
				imshow_surprise(img_test);
				ros::Duration(0.5).sleep();
				sc.playWave(path_to_sounds + "sound.wav");
				ranOnce=true;
			}

			rightBumper();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);

		}else if(world_state == 6){

			if (!ranOnce) {
				Mat img_test=imread(ros::package::getPath("mie443_contest3")+"/imgs/meow_surprised_yoga.jpg");
				imshow_surprise(img_test);
				ros::Duration(0.5).sleep();
				sc.playWave(path_to_sounds + "sound.wav");
				ranOnce=true;
			}
			leftBumper();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);
		
		} else if (world_state == 4){
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "sound.wav");
				ranOnce=true;
			}
			excited();
			vel.angular.z = angular;
			vel.linear.x = linear;
			vel_pub.publish(vel);
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}
	return 0;
}
void excited(){
	linear = 0.5;
	angular = 1;
	return;
}
bool matchImage(Mat inp_frame){
	detector -> detectAndCompute( inp_frame, noArray(), keypoints_img, descriptor_img );
	
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptor_ref, descriptor_img, knn_matches, 2 );

	std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    ROS_INFO("Good matches: %lu", good_matches.size());

	Mat img_matches;
    drawMatches( img_ref, keypoints_ref, inp_frame, keypoints_img, good_matches,img_matches, Scalar::all(-1),
    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	imshow_fast(img_matches);

	if (good_matches.size()>50) return true;
	else return false;
}

void set_start_time(){
	state_start = std::chrono::system_clock::now();
	return;
}
uint64_t get_time_elapsed(){
	uint64_t mil_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-state_start).count();
	return mil_elapsed;
}
void rightBumper() {
    //the rightBumper function turns the robot right by 90 degrees if the right bumper is hit
    //make a surprised cat sound 
    //Return to its original position 
    //returns state 0 to move forward
    if(get_time_elapsed()<1000){
        angular=1.0;
        linear=0;
        return;
    } 
    if(get_time_elapsed()>1000 && get_time_elapsed()<2000 ){
        angular=-1.0;
        linear=0;
        return;

    }else {
	cv::destroyAllWindows();
        world_state=0;
        return;
    }
}

//BUMPER SHIT 
void leftBumper() {
    //the rightBumper function turns the robot left by 90 degrees if the right bumper is hit
    //make a surprised cat sound 
    //Return to its original position 
    //returns state 0 to move forward
    if(get_time_elapsed()<1000){
        angular=-1.0;
        linear=0;
        return;
    } 
    if(get_time_elapsed()>1000 && get_time_elapsed()<2000 ){
        angular=1.0;
        linear=0;
        return;

    }else {
	cv::destroyAllWindows();
        world_state=0;
        return; 
    }
}
