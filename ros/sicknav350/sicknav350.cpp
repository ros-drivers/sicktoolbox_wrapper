/*
 * Authors: Nick Hillier and Fred Pauling (CSIRO, 2011)
 * 
 * Based on the sicklms.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 * 
 * Released under BSD license.
 */ 

#include <iostream>
#include <sicktoolbox/SickNAV350.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" 
#include <deque>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace std;
using namespace SickToolbox;

// TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)
void publish_scan(ros::Publisher *pub, double *range_values,
                  uint32_t n_range_values, unsigned int *intensity_values,
                   uint32_t n_intensity_values, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  if (inverted) { // assumes scan window at the bottom
    scan_msg.angle_min = angle_max;
    scan_msg.angle_max = angle_min;
  } else {
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values-1);
  scan_msg.scan_time = 0.125;//scan_time;
  scan_msg.time_increment = scan_msg.scan_time/*scan_time*/ / n_range_values;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 250.;
  scan_msg.ranges.resize(n_range_values);
  scan_msg.header.stamp = start;
  for (size_t i = 0; i < n_range_values; i++) {
    scan_msg.ranges[i] = (float)range_values[i]/1000;
  }
  scan_msg.intensities.resize(n_intensity_values);
  for (size_t i = 0; i < n_intensity_values; i++) {
    scan_msg.intensities[i] = 0;//(float)intensity_values[i];
  }
  pub->publish(scan_msg);
}

// A complimentary filter to get a (much) better time estimate, does not
// calibrate out constant network latency delays, but does get rid of 
// timming jitter to get better timing estimates than the 
// communicated clock resolution (which is only 1ms)
class smoothtime { 
    protected:
        ros::Time smoothtime_prev, smoothed_timestamp;
        double time_smoothing_factor;
        double error_threshold;
    public:
        //! 
        smoothtime(){
            time_smoothing_factor = 0.95; /// slowly skew the clocks into sync
            error_threshold = .50; /// 50% jitter is acceptable for , discard data otherwise.
        }
        //! Between 0 and 1, bigger is smoother
        void set_smoothing_factor(double smoothing_factor){ 
            time_smoothing_factor = smoothing_factor;
        }
        //! Between 0 and 1, threshold on jitter acceptability, higher accepts more jitter before discarding
        void set_error_threshold(double err_threshold){ 
            error_threshold = err_threshold;
        }
        ros::Time smooth_timestamp(ros::Time recv_timestamp, ros::Duration expctd_dur) {
            if (smoothtime_prev.is_zero() == true) {
                smoothed_timestamp = recv_timestamp;
            } else {
                smoothed_timestamp = smoothtime_prev + expctd_dur;
                double err = (recv_timestamp - smoothed_timestamp).toSec();
                double time_error_threshold = expctd_dur.toSec() * error_threshold;
                if ((time_smoothing_factor > 0) && (fabs(err) < time_error_threshold)){
                    ros::Duration correction = ros::Duration(err * (1 - time_smoothing_factor));
                    smoothed_timestamp += correction;
                } else {
                    // error too high, or smoothing disabled - set smoothtime to last timestamp
                    smoothed_timestamp = recv_timestamp;
                }
            }
            smoothtime_prev = smoothed_timestamp;
            return smoothed_timestamp;
        }
};

class averager {
    protected:
        std::deque<double> deq;
        unsigned int max_len;
        double sum;
    public:
        averager(int max_len = 50){
            this->max_len = max_len;
        }
        void add_new(double data) {
            deq.push_back(data);
            sum += data;
            if (deq.size() > max_len) {
                sum -= deq.front();
                deq.pop_front();
            }
        }
        double get_mean() {
            return sum/deq.size();
        }
};
       
void PublishReflectorTransform(double x,double y,double th,tf::TransformBroadcaster odom_broadcaster)
{
    ros::Time current_time;
	current_time=ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "reflector";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
}
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sicknav350");
    int port;
    std::string ipaddress;
    std::string frame_id;
    std::string scan;
    bool inverted;
    int sick_motor_speed = 8;//10; // Hz
    double sick_step_angle = 1.5;//0.5;//0.25; // deg (0.125 = no gaps between spots)
    double active_sector_start_angle = 0;
    double active_sector_stop_angle = 360;//269.75;
    double smoothing_factor, error_threshold;
    ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	nh_ns.param<std::string>("scan", scan, "scan");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 1);
	nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
	nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
	nh_ns.param("inverted", inverted, false);
	nh_ns.param<std::string>("frame_id", frame_id, "laser");
    nh_ns.param("timer_smoothing_factor", smoothing_factor, 0.97);
    nh_ns.param("timer_error_threshold", error_threshold, 0.5);
    nh_ns.param("resolution", sick_step_angle, 1.0); 
    nh_ns.param("start_angle", active_sector_start_angle, 0.); 
    nh_ns.param("stop_angle", active_sector_stop_angle, 360.); 
    nh_ns.param("scan_rate", sick_motor_speed, 5); 
//	ipaddress="127.0.0.1";
    /* Define buffers for return values */
    double range_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
    unsigned int intensity_values[SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
    /* Define buffers to hold sector specific data */
    unsigned int num_measurements = {0};
    unsigned int sector_start_timestamp = {0};
	unsigned int sector_stop_timestamp = {0};
    double sector_step_angle = {0};
    double sector_start_angle = {0};
    double sector_stop_angle = {0};
    /* Instantiate the object */
    SickNav350 sick_nav350(ipaddress.c_str(),port);
double last_time_stamp=0;
    try {
        /* Initialize the device */
        sick_nav350.Initialize();
        // TODO: do some calls to setup the device - e.g. scan rate.
        try {
//		sick_nav350.GetSickIdentity();
		sick_nav350.SetOperatingMode(3);
        } catch (...) {
            ROS_ERROR("Configuration error");
            return -1;
        }
        smoothtime smoothtimer;
        averager avg_fulldur, avg_scandur;
        smoothtimer.set_smoothing_factor(smoothing_factor);
        smoothtimer.set_error_threshold(error_threshold);
        ros::Time last_start_scan_time;
        unsigned int last_sector_stop_timestamp = 0;
        double full_duration;  
	ros::Rate loop_rate(8);
	tf::TransformBroadcaster odom_broadcaster;

        while (ros::ok()) {
            /* Grab the measurements (from all sectors) */
//        	sick_nav350.GetDataLandMark(1,1);
        	sick_nav350.GetDataNavigation(1,1);
            sick_nav350.GetSickMeasurements(range_values,
                                        &num_measurements,
                                        &sector_step_angle,
                                        &sector_start_angle,
                                        &sector_stop_angle,
                                        &sector_start_timestamp,
                                        &sector_stop_timestamp
                                        );
	double x1=(double) sick_nav350.PoseData_.x;
	double y1=(double) sick_nav350.PoseData_.y;
	double phi1=sick_nav350.PoseData_.phi;
	double x2,y2;
	double phi2=phi1-180000-1250-300;
	phi2=phi2/1000/180*3.14159;
	x2=x1-529*cos(phi2);
	y2=y1-529*sin(phi2);
	x2=x2/1000;
	y2=y2/1000;
	std::cout<<sick_nav350.PoseData_.x<<"   "<<x1<<" "<<y1<<"   "<<x2<<" "<<y2<<std::endl;
	PublishReflectorTransform(x2,y2,phi2,odom_broadcaster);
	if (sector_start_timestamp<last_time_stamp) 
	{
		loop_rate.sleep();
		ros::spinOnce();
		continue;
	}
	last_time_stamp=sector_start_timestamp;
            ros::Time end_scan_time = ros::Time::now();
            
            double scan_duration = (sector_stop_timestamp - sector_start_timestamp) * 1e-3;
            avg_scandur.add_new(scan_duration);
            scan_duration = 0.125;//avg_scandur.get_mean();
            
            if (last_sector_stop_timestamp == 0) {
                full_duration = 1./((double)sick_motor_speed);
            } else {
                full_duration = (sector_stop_timestamp - last_sector_stop_timestamp) * 1e-3;
            }
            avg_fulldur.add_new(full_duration);
            full_duration = avg_fulldur.get_mean();
            
            ros::Time smoothed_end_scan_time = smoothtimer.smooth_timestamp(end_scan_time, ros::Duration(full_duration));
            ros::Time start_scan_time = smoothed_end_scan_time - ros::Duration(scan_duration);
            sector_start_angle-=180;
		sector_stop_angle-=180;
            publish_scan(&scan_pub, range_values, num_measurements, intensity_values,
                   num_measurements, start_scan_time, scan_duration, inverted,
                   DEG2RAD((float)sector_start_angle), DEG2RAD((float)sector_stop_angle), frame_id);
/*
            ROS_INFO_STREAM/*DEBUG_STREAM*//*("Num meas: " << num_measurements
                 << " smoothed start T: " << start_scan_time
                 << " smoothed rate: " << 1./(start_scan_time - last_start_scan_time).toSec()
                 << " raw start T: " << sector_start_timestamp
                 << " raw stop T: " << sector_stop_timestamp
                 << " dur: " << full_duration
                 << " step A: " << sector_step_angle
                 << " start A: " << sector_start_angle
                 << " stop A: " << sector_stop_angle);*/
            last_start_scan_time = start_scan_time;
            last_sector_stop_timestamp = sector_stop_timestamp;
		loop_rate.sleep();
			ros::spinOnce();

        }
        /* Uninitialize the device */
       sick_nav350.Uninitialize();
    }
    catch(...) {
        ROS_ERROR("Error");
        return -1;
    }
    return 0;
}
