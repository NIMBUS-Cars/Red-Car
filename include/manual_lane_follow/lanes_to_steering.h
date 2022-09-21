#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
using namespace std;
using namespace cv;
double slope(Point first, Point second);

class Control
{
public:
    static double steer(vector<vector<double>> yellowLaneLines, vector<vector<double>> whiteLaneLines, int laneNumber)
    {
        double steeringAngle = std::numeric_limits<double>::quiet_NaN();

        /** -------------------------------------------------**\
        * ------------------Define Constants------------------ *
        \**--------------------------------------------------**/
        // 6-1 FOR CONSTS
        double INNER_TANH_CONSTANT = 0.5;
        double OUTER_TAHN_CONSTANT = 0.4;
        double CENTER_X = 0.5;
        double SLOPE_ADJUSTMENT_CONSTANT =3;
        /** -------------------------------------------------**\
        * -------------YELLOW & WHITE LANES FOUND------------- *
        \**--------------------------------------------------**/
        if (static_cast<int>(yellowLaneLines.size()) >= 1 && static_cast<int>(whiteLaneLines.size()) >= 1)
        {
            //ROS_INFO("TWO LANES FOUND");
            //Get solpes and intercepts from vectors
            double wSlope = whiteLaneLines.at(0).at(1); //Slope of the white line
            double wXCoord = whiteLaneLines.at(0).at(0); //X coordinate of the white line (at the bottom of the image)
            double ySlope = yellowLaneLines.at(0).at(1); //Slope of the yellow line
            double yXCoord = yellowLaneLines.at(0).at(0); //X coordinate of the yellow line (at the bottom of the image)
            double adjustedWSlope = -1 * (wSlope + (wXCoord - CENTER_X) * SLOPE_ADJUSTMENT_CONSTANT);
            double adjustedYSlope = ySlope + (yXCoord - CENTER_X) *  SLOPE_ADJUSTMENT_CONSTANT;
            if(adjustedWSlope == 0){
                adjustedWSlope = 0.01;
            }
            if(adjustedYSlope == 0){
                adjustedYSlope = 0.01;
            }
            steeringAngle = OUTER_TAHN_CONSTANT * tanh(INNER_TANH_CONSTANT * ((adjustedYSlope+adjustedWSlope)/2));
        }
        /** -------------------------------------------------**\
        * ---------------CONTROL FOR INNER LANES-------------- *
        \**--------------------------------------------------**/
        else if (static_cast<int>(whiteLaneLines.size()) == 0 && static_cast<int>(yellowLaneLines.size()) == 1)
        {
            //ROS_INFO("ONLY YELLOW LANE FOUND");
            double ySlope = yellowLaneLines.at(0).at(1);
            double yXCoord = yellowLaneLines.at(0).at(0);
            double adjustedYSlope = ySlope + (yXCoord - CENTER_X) * SLOPE_ADJUSTMENT_CONSTANT;
            steeringAngle = OUTER_TAHN_CONSTANT * tanh(INNER_TANH_CONSTANT * adjustedYSlope);
         }
        /** -------------------------------------------------**\
        * ---------------CONTROL FOR OUTER LANES-------------- *
        \**--------------------------------------------------**/
        else if (static_cast<int>(yellowLaneLines.size()) == 0 && static_cast<int>(whiteLaneLines.size()) == 1)
        {
            //ROS_INFO("ONLY WHITE LANE FOUND");
            ///No yelow lane found but white lane found
            //Most likely making a left turn in the right lane
            double wSlope = whiteLaneLines.at(0).at(1);
            double wXCoord = whiteLaneLines.at(0).at(0);
            double adjustedWSlope = -1 * (wSlope + (wXCoord - CENTER_X) *SLOPE_ADJUSTMENT_CONSTANT);

            steeringAngle = OUTER_TAHN_CONSTANT * tanh(INNER_TANH_CONSTANT * adjustedWSlope);
            
        }
        return steeringAngle;
    }
};