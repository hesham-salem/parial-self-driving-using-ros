#include "ros/ros.h"
#include "std_msgs/String.h"
#include"sensors/x_state.h"
#include"sensors/z_vector.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "FusionEKF.h"
#include "tools.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//objects
sensors::z_vector z_vector;
sensors::x_state x_state;

//void callback()
void chatterCallback(sensors::z_vector msg)
{
  ROS_INFO("start");
   z_vector.type = msg.type;
   z_vector.px = msg.px;
   z_vector.py = msg.py;
   z_vector.vx = msg.vx;
   z_vector.vy = msg.vy;
   z_vector.timestamp=1111;
}


int main(int argc, char **argv)

{
    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;
    sensors::x_state x_state; //the same name
    unsigned char sensor_type;
       MeasurementPackage meas_package;
       GroundTruthPackage gt_package;
        long long timestamp;

//////////////////////////////////////// intialization

  ros::init(argc, argv, "sensor_fusion");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensors::x_state>("object_position", 1000);
    ros::Subscriber sub = nh.subscribe("raw_measurement", 1000, chatterCallback);

  ros::Rate loop_rate(10);
   ros::spinOnce();

////////////////////////////////////////

   while (ros::ok())
  {


        sensor_type = z_vector.type ;//object
        ROS_INFO("%c",sensor_type);

       if (sensor_type == 'L') {
         // LASER MEASUREMENT
           ROS_INFO("start publisher");

         // read measurements at this timestamp
         meas_package.sensor_type_ = MeasurementPackage::LASER;
         meas_package.raw_measurements_ = VectorXd(2);
         double x;
         double y;
          x=z_vector.px;
         y=z_vector.py;
         meas_package.raw_measurements_ << x, y;
         timestamp = z_vector.timestamp;
         meas_package.timestamp_ = timestamp;
         measurement_pack_list.push_back(meas_package); //the problem here
       } else if (sensor_type == 'R') {
         // RADAR MEASUREMENT

         // read measurements at this timestamp
         meas_package.sensor_type_ = MeasurementPackage::RADAR;
         meas_package.raw_measurements_ = VectorXd(3);
         float ro;
         float phi;
         float ro_dot;

                 ro=z_vector.ro;

                 phi=z_vector.phi;

                 ro_dot=z_vector.ro_dot;
         meas_package.raw_measurements_ << ro, phi, ro_dot;

         timestamp=z_vector.timestamp;
       //  meas_package.timestamp_ = timestamp;
         measurement_pack_list.push_back(meas_package);
       };




    // Create a Fusion EKF instance
     FusionEKF fusion_EKF;

     // used to compute the RMSE later
     vector<VectorXd> estimations;
     vector<VectorXd> ground_truth;

     //Call the EKF-based fusion
     size_t N = measurement_pack_list.size();
     for (size_t k = 0; k < N; ++k) {
       // start filtering from the second frame (the speed is unknown in the first
       // frame)
       fusion_EKF.ProcessMeasurement(measurement_pack_list[k]);


    // output the estimation
      float  px = fusion_EKF.ekf_.x_(0) ;
       float py = fusion_EKF.ekf_.x_(1) ;
       float vx = fusion_EKF.ekf_.x_(2) ;
       float vy = fusion_EKF.ekf_.x_(3);


       x_state.px=px;
       x_state.py=py;
       x_state.vx=vx;
       x_state.vy=vy;



       estimations.push_back(fusion_EKF.ekf_.x_);
       ground_truth.push_back(gt_pack_list[k].gt_values_);


     // compute the accuracy (RMSE)
     Tools tool;
     cout << "Accuracy - RMSE:" << endl << tool.CalculateRMSE(estimations, ground_truth) << endl;








  }
       pub.publish(x_state);

       ros::spinOnce();

       loop_rate.sleep();

}

  return 0;
}
