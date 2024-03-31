#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

geometry_msgs::Twist follow_cmd;
int world_state;
int prev_state;
bool state_lockout = false;

ros::Timer state_timer;

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
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
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

double angular = 0.0;
double linear = 0.0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	state_timer = nh.createTimer(ros::Duration(5.0), timerCB);
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

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();
		//State evaluation, put the most important states first! timerCB will automatically return to state 0 and reset state_lockout
		
		//if (matchImage(rgbTransport.getImg())) world_state = 4;
		if (!state_lockout && matchImage(img_test) && prev_state!=4) world_state = 4; 
		
		if (prev_state!=world_state && world_state != 0){
			state_timer.start();
			state_lockout=true;
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
		} else if (world_state == 4){
			if (!ranOnce) {
				sc.playWave(path_to_sounds + "excited.wav");
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