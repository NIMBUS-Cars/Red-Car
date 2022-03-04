#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <cmath>

class ReactiveMethod {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, car_width, car_length, max_acceleration,car_radius;

    // Publish drive data
    ros::Publisher drive_pub;

    //Listens for Laser Scan
    ros::Subscriber scan_sub;

    //float DISCREPANCY_DISTANCE = .4; 
    float SCAN_WITHIN_ANGLE = ( 90 / 180.0) *0.95 * M_PI; //ADDED .95 for when it now checks for farthest angle based on how far it can go
    double currentSpeed = 0;

public:
    ReactiveMethod() {
        n = ros::NodeHandle("~");
        std::string drive_topic, scan_topic;
        n.getParam("/nav_drive_topic", drive_topic);
        n.getParam("/scan_topic", scan_topic);
        n.getParam("/max_speed", max_speed);
        n.getParam("/max_steering_angle", max_steering_angle);
        n.getParam("/width", car_width);
        n.getParam("/wheelbase", car_length);
        n.getParam("/max_decel",max_acceleration);
        car_radius =sqrt(pow(car_width,2) +pow(car_length,2))/2;
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        scan_sub = n.subscribe(scan_topic, 1, &ReactiveMethod::scan_callback, this);
    }
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
        //things to add
        // speed that corresponds to how much you can travel with the aeb
        std::vector<float> safeScanDistances(static_cast<int>(msg->ranges.size()));
        for(int i = 0; i< static_cast<int>(msg->ranges.size());i++){
            float test = msg->ranges.at(i);
            safeScanDistances.at(i) = msg-> ranges.at(i);
        }
        for(int i = 0; i< static_cast<int>(msg->ranges.size());i++){

                double radiusToReduceValues = angleToAvoid(msg->ranges.at(i));
                int indiciesToConsider = static_cast<int>(radiusToReduceValues / msg->angle_increment);
                for(int j = 0; j< indiciesToConsider;j++){
                    if(i-j>=0 && safeScanDistances.at(i-j)>msg->ranges.at(i)){
                        safeScanDistances.at(i-j) = msg->ranges.at(i);
                         float test = msg->ranges.at(i);
                    }
                    if(i+j<static_cast<int>(msg->ranges.size()) && safeScanDistances.at(i+j)>msg->ranges.at(i)){
                        safeScanDistances.at(i+j)= msg->ranges.at(i);
                         float test = msg->ranges.at(i);
                    }
                }
        }
        int indexOfFarthestValue = static_cast<int>(abs(msg->angle_min/msg->angle_increment));
        float farthestDistance = safeScanDistances.at(indexOfFarthestValue);
        for(int i = 0; i< static_cast<int>(msg->ranges.size());i++){
            if(abs(msg->angle_increment * i + msg->angle_min) < SCAN_WITHIN_ANGLE){
                if(safeScanDistances.at(i) > farthestDistance){
                    // indexOfFarthestValue = i;
                    // farthestDistance = safeScanDistances.at(i);
                    //Checks to see if angle is ok for pure pursuit so it can choose other angles if the best angle isnt great
                    
                    double currentAngle = msg->angle_increment * i + msg->angle_min;
                    double radiusOfCurvature = abs(pow(safeScanDistances.at(i),2) / (2 * safeScanDistances.at(i) * sin(currentAngle)));
                    double steeringAngleForCurrentAngle = getSteeringAngleFromRadius(radiusOfCurvature,currentAngle);
                    if(abs(steeringAngleForCurrentAngle) <max_steering_angle){
                        indexOfFarthestValue = i;
                        farthestDistance = safeScanDistances.at(i);
                    }

                }
            }
        }
        double angleOfFarthestPoint = msg->angle_increment * indexOfFarthestValue + msg->angle_min;
        // Pure Pursuit
        double radiusOfCurvature;
         int indexOfZero = static_cast<int>(abs(msg->angle_min/msg->angle_increment));
         //replacing this for the length of the longest angle helps us avoid walls we might face that are between the 0 and the angle to the distance we are looking for that may make our car crash
         //adjustments were made to base angle to max it the min of the distance in front of it so car can curve properly in long passage ways
         double minimumAngleToConsider = -abs(atan(car_width/car_length));
        if(minimumAngleToConsider<-M_PI){
            minimumAngleToConsider = -M_PI;
        }
        double maximumAngleToConsider = abs(atan(car_width/car_length));
        if(maximumAngleToConsider>M_PI){
            maximumAngleToConsider = M_PI;
        }
        int indexOfMin = (minimumAngleToConsider /msg->angle_increment)+indexOfZero;
        int indexOfMax = (maximumAngleToConsider /msg->angle_increment)+indexOfZero;
        
        double shortestDistanceInCurve = std::numeric_limits<double>::infinity();
            if(angleOfFarthestPoint >0){
                // was i = indexOfZero
                for(int i = indexOfMin;i<indexOfFarthestValue;i++){
                    if(shortestDistanceInCurve>msg->ranges.at(i)){
                        shortestDistanceInCurve = msg->ranges.at(i);
                    }
                }
            }else{
                // was i< indexOfZero
                for(int i = indexOfFarthestValue;i<indexOfMax;i++){
                    if(shortestDistanceInCurve>msg->ranges.at(i)){
                        shortestDistanceInCurve = msg->ranges.at(i);
                    }
                }
            }
        if(abs(angleOfFarthestPoint) < .95 * M_PI){
        radiusOfCurvature = abs(pow(shortestDistanceInCurve,2) / (2 * shortestDistanceInCurve * sin(angleOfFarthestPoint)));
        }else{
            radiusOfCurvature = shortestDistanceInCurve -car_radius * 1.5;
        }
        double steeringAngle = getSteeringAngleFromRadius(radiusOfCurvature,angleOfFarthestPoint);
        
        // CALCULATE SPEED BASED ON TIME TO COLLISION DIRECTLY IN FRONT OF IT
        double smallestTTC = std::numeric_limits<double>::infinity();
        for(int i = indexOfMin;i<indexOfMax;i++){
            if(i>=0 &&i<static_cast<int>(msg->ranges.size())){
           if(abs(i*msg->angle_increment + msg->angle_min)<.1){
              float distance = msg->ranges.at(i) - car_radius; // may need to remove car radius added as test
              float velocity = currentSpeed; // may need to change back to currentSpeed 
              float TTC = distance / velocity;
              if(TTC<smallestTTC){
                 smallestTTC=TTC;
              }
           }else{
              float distance = msg->ranges.at(i)- car_radius; // may need to remove car radius added as test
              if(distance <car_width /2 * tan(90 - abs(i*msg->angle_increment + msg->angle_min))){ 
              float velocity = currentSpeed * cos(abs(i*msg->angle_increment + msg->angle_min)); // may need to change back to currentSpeed  * cos(...)
              float TTC = distance / velocity;
              if(TTC<smallestTTC){
                 smallestTTC=TTC;
              }
              }

           }
            }
        }
        double relativeTTCDistance  = (smallestTTC-0.5) *currentSpeed; // removing .2 can be allowed I added this to give it somewhat of a buffer so it will always have .2 TTC worth of a gap in distance
        if(currentSpeed ==0){
            relativeTTCDistance = msg->ranges.at(indexOfZero)-car_radius;
        }
        if(relativeTTCDistance<0){
            relativeTTCDistance =0;
        }
        //IDEA TO LATER SELF
        //may not be ideal. you may need a lower acceleration because the vehicle wont use its max acceleration if it thinks it only needs to go down a little bit in velocity
        // this assumes it is always using its max acceleartion
        double allowableSpeed = sqrt(2 * max_acceleration * relativeTTCDistance);
        if(allowableSpeed<0){
            allowableSpeed = 0;
        }
        if(abs(steeringAngle) >(20.0/180)*M_PI){
            if(allowableSpeed >2){
                allowableSpeed = 1;
            }
        }
        else if(abs(steeringAngle) >(30.0/180)*M_PI){
            if(allowableSpeed >1){
                allowableSpeed = 0.5;
            }
        }
        //TODOS
        // MAKE ADJUSTMENTS FOR REALLY BIG HALLYWAYS WHERE CURVE GETS TO CLOSE TO THE WALL
        // ADD A SPACER ON THE SIDE OF THE CAR THAT LIKES TO STAY A DISTANCE FROM THE WALL
        // THERE SEEMS TO BE ERROS WHEN IT TRIES TO MAKE SHARP TURNS WHEN IT IS LESS THAN THAT RADIUS OF CURATURE CHECK THAT PART OUT
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = steeringAngle;
        drive_msg.speed = allowableSpeed;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
        // ROS_INFO("Angle of Best Direction %s",std::to_string(angleOfFarthestPoint).c_str());
        // ROS_INFO("Corrected Steering Angle %s",std::to_string(steeringAngle).c_str());
        // ROS_INFO("TTC DISTANCE: %s",std::to_string(relativeTTCDistance).c_str());
        // ROS_INFO("Smallest TTC: %s",std::to_string(smallestTTC).c_str());
        // ROS_INFO("Old Speed: %s",std::to_string(currentSpeed).c_str());
        // ROS_INFO("Speed of Car: %s",std::to_string(allowableSpeed).c_str());
        currentSpeed = allowableSpeed;
        if(currentSpeed<0.5){
            currentSpeed = 0.5;
        }

    }
    double angleToAvoid(float distance){
        float avoidRadius = car_radius * 1.5;
        return atan(avoidRadius/distance);
    }
    double getSteeringAngleFromRadius(double radiusOfCurvature, double angleOfFarthestPoint){
        double steeringAngle;
        if(radiusOfCurvature < car_radius && angleOfFarthestPoint <0){
            steeringAngle = -1 * max_steering_angle;
        }
         else if(radiusOfCurvature < car_radius && angleOfFarthestPoint>0){
            steeringAngle = max_steering_angle;
        }else{
            steeringAngle = atan(1/ ( sqrt( (pow(radiusOfCurvature,2) - pow(car_length/2,2)) / pow(car_length,2) ) ) );
        }
        if(angleOfFarthestPoint<0){
            steeringAngle *= -1;
        }
        return steeringAngle;
    }

}; // end of class definition

int main(int argc, char ** argv) {
    ros::init(argc, argv, "Reactive_Method");
    ReactiveMethod rw;
    ros::spin();
    return 0;
}
