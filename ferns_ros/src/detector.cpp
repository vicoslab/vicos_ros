#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ferns_ros/Detection.h>
#include <ferns_ros/DetectedPoint.h>
#include "ferns/mcv.h"
#include "ferns/planar_pattern_detector_builder.h"
#include "ferns/template_matching_based_tracker.h"

using namespace cv;
using namespace std;
using namespace ferns_ros;
namespace enc = sensor_msgs::image_encodings;


const int max_filename = 1000;

enum source_type {webcam_source, sequence_source, video_source};

planar_pattern_detector * detector = NULL;
template_matching_based_tracker * tracker = NULL;

/*
0: Detect when tracking fails or for initialization then track.
1: Track only
2: Detect only (DEFAULT)
3: Detect + track in every frame
*/
int mode = 2;
//Show CV window with ferns detections
bool show_window = true;
bool show_tracked_locations = true;
bool show_keypoints = true;


void draw_quadrangle(Mat& frame, int u0, int v0, int u1, int v1,int u2, int v2, int u3, int v3, Scalar color, int thickness, Detection& detection) {

  //Create msgs to publish detections on a topic
  detection.top_left.x = u0; detection.top_left.y = v0;
	detection.top_right.x = u1; detection.top_right.y = v1;
	detection.bottom_right.x = u2; detection.bottom_right.y = v2;
	detection.bottom_left.x = u3; detection.bottom_left.y = v3;

  cv::line(frame, cv::Point(u0, v0), cv::Point(u1, v1), color, thickness);
  cv::line(frame, cv::Point(u1, v1), cv::Point(u2, v2), color, thickness);
  cv::line(frame, cv::Point(u2, v2), cv::Point(u3, v3), color, thickness);
  cv::line(frame, cv::Point(u3, v3), cv::Point(u0, v0), color, thickness);

}

void draw_detected_position(Mat& frame, planar_pattern_detector* detector, Detection& detection)
{
  draw_quadrangle(frame,
		  detector->detected_u_corner[0], detector->detected_v_corner[0],
		  detector->detected_u_corner[1], detector->detected_v_corner[1],
		  detector->detected_u_corner[2], detector->detected_v_corner[2],
		  detector->detected_u_corner[3], detector->detected_v_corner[3],
		  cvScalar(255), 3, detection);
}

void draw_initial_rectangle(Mat& frame, template_matching_based_tracker* tracker, Detection& detection)
{
  draw_quadrangle(frame,
		  tracker->u0[0], tracker->u0[1],
		  tracker->u0[2], tracker->u0[3],
		  tracker->u0[4], tracker->u0[5],
		  tracker->u0[6], tracker->u0[7],
		  cvScalar(128), 3, detection);
}

void draw_tracked_position(Mat& frame, template_matching_based_tracker* tracker, Detection& detection)
{
  draw_quadrangle(frame,
		  tracker->u[0], tracker->u[1],
		  tracker->u[2], tracker->u[3],
		  tracker->u[4], tracker->u[5],
		  tracker->u[6], tracker->u[7],
		  cvScalar(255), 3, detection);
}

void draw_tracked_locations(Mat& frame, template_matching_based_tracker * tracker)
{
  for(int i = 0; i < tracker->nx * tracker->ny; i++) {
    int x1, y1;
    tracker->f.transform_point(tracker->m[2 * i], tracker->m[2 * i + 1], x1, y1);
    cv::circle(frame, cv::Point(x1, y1), 3, cv::Scalar(255, 255, 255), 1);
  }
}

void draw_detected_keypoints(Mat& frame, planar_pattern_detector * detector)
{
  for(int i = 0; i < detector->number_of_detected_points; i++)
    cv::circle(frame,
	     cv::Point(detector->detected_points[i].fr_u(),
		     detector->detected_points[i].fr_v()),
	     16 * (1 << int(detector->detected_points[i].scale)),
	     cv::Scalar(100), 1);
}

void draw_recognized_keypoints(Mat& frame, planar_pattern_detector * detector)
{
  for(int i = 0; i < detector->number_of_model_points; i++)
    if (detector->model_points[i].class_score > 0)
      cv::circle(frame,
	       cv::Point(detector->model_points[i].potential_correspondent->fr_u(),
		       detector->model_points[i].potential_correspondent->fr_v()),
	       16 * (1 << int(detector->detected_points[i].scale)),
	       cv::Scalar(255, 255, 255), 1);
}


bool detect_and_draw(Mat& frame, Detection& detection) {

	static bool last_frame_ok=false;

  IplImage iplFrame = frame;

	if (mode == 1 || ((mode==0) && last_frame_ok)) {
		bool ok = tracker->track(&iplFrame);
		last_frame_ok=ok;


		if (!ok) {
			if (mode==0) {
        detect_and_draw(frame, detection);
        return true;
      } else {
				draw_initial_rectangle(frame, tracker, detection);
				tracker->initialize();
			}
		} else {

			draw_tracked_position(frame, tracker, detection);
			if (show_tracked_locations) draw_tracked_locations(frame, tracker);

		}

		cv::putText(frame, "template tracking", cv::Point(10, 30), FONT_HERSHEY_PLAIN, 5, cv::Scalar(255, 255, 255));
	} else {
		detector->detect(&iplFrame);

		if (detector->pattern_is_detected) {

			last_frame_ok=true;

			tracker->initialize(detector->detected_u_corner[0], detector->detected_v_corner[0],
					detector->detected_u_corner[1], detector->detected_v_corner[1],
					detector->detected_u_corner[2], detector->detected_v_corner[2],
					detector->detected_u_corner[3], detector->detected_v_corner[3]);

			if (mode == 3 && tracker->track(&iplFrame)) {

				if (show_keypoints) {
					draw_detected_keypoints(frame, detector);
					draw_recognized_keypoints(frame, detector);
				}
			  draw_tracked_position(frame, tracker, detection);

				if (show_tracked_locations) draw_tracked_locations(frame, tracker);

				cv::putText(frame, "detection+template", cv::Point(10, 30), FONT_HERSHEY_PLAIN, 5, cv::Scalar(255, 255, 255));
			} else {
				if (show_keypoints) {
					draw_detected_keypoints(frame, detector);
					draw_recognized_keypoints(frame, detector);
				}
				draw_detected_position(frame, detector, detection);
				cv::putText(frame, "detection", cv::Point(10, 30), FONT_HERSHEY_PLAIN, 5, cv::Scalar(255, 255, 255));
			}
		} else {
			last_frame_ok=false;
			if (show_keypoints) draw_detected_keypoints(frame, detector);

			if (mode == 3)
				cv::putText(frame, "detection+template", cv::Point(10, 30), FONT_HERSHEY_PLAIN, 5, cv::Scalar(255, 255, 255));
			else
				cv::putText(frame, "detection", cv::Point(10, 30), FONT_HERSHEY_PLAIN, 5, cv::Scalar(255, 255, 255));
		}
	}
  
  return last_frame_ok;
}

cv_bridge::CvImagePtr bridge;


bool SHOW_CV_WINDOW;
string IMAGE_TOPIC;

ros::Publisher detection;

/*Callback for color image from kinect*/
void imageReceiver(const sensor_msgs::ImageConstPtr& image) { 
 
  try{
    bridge = cv_bridge::toCvCopy(image, enc::MONO8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }  

  ROS_ERROR("%d %d", bridge->image.cols, bridge->image.rows);

  //frame = new IplImage(bridge->image);
  //cv::imshow("ferns-demo", bridge->image);
	
  //if (bridge->image.origin != IPL_ORIGIN_TL)
  //  cv::flip(bridge->image, bridge->image, 0);

  Detection detectionMsg;

  if(detect_and_draw(bridge->image, detectionMsg)) {
    detectionMsg.header = image->header;	
	  detection.publish(detectionMsg);
  }

  // Show ferns detections in CV window.
  if(SHOW_CV_WINDOW){
		cv::imshow("Ferns", bridge->image);
  	cv::waitKey(1);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ferns_ros");

  string model_image("/home/vicos/ROS/ferns_data/book.jpg");

  string detector_model = model_image + string(".detector_data");

  detector = planar_pattern_detector_builder::just_load(detector_model.c_str());

  if (!detector) {
    ROS_ERROR("Unable to load detector for image '%s'", model_image.c_str());
    return -1;
  }

  detector->set_maximum_number_of_points_to_detect(1000);
  tracker = new template_matching_based_tracker();

  string tracker_model = model_image + string(".tracker_data");

  if (!tracker->load(tracker_model.c_str())) {
    ROS_ERROR("Unable to load tracker for image '%s'", model_image.c_str());
    return -1;    
  }

  //ROS handle has to be after detector initialization. Otherwise we get seg fault because of Ferns library!
  //ros::NodeHandle n;

  ros::NodeHandle n;
  n.param("ferns_detector/model", model_image, string("model.jpg"));

	tracker->initialize();

  n.param("ferns_detector/show_cv_window", SHOW_CV_WINDOW, true);
  n.param("ferns_detector/image_topic", IMAGE_TOPIC, string("/camera/rgb/image_color"));

  if (SHOW_CV_WINDOW) {
	  cv::namedWindow("Ferns", CV_WINDOW_AUTOSIZE); 
  }

  detection = n.advertise<Detection>("detection", 1000);
	ros::Subscriber sub = n.subscribe(IMAGE_TOPIC, 10, imageReceiver);

	ros::spin(); 

	if(!ros::ok())
	{
		clog << endl;
		delete detector;
		delete tracker;

		if (SHOW_CV_WINDOW) cvDestroyWindow("Ferns");
	}


  return 0;
  
}

