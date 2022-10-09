#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <manual_lane_follow/lane_detector.h>
#include <manual_lane_follow/lanes_to_steering.h>
using namespace std;
using namespace cv;

class ManualSteeringControl{
    public:
    int laneNumber = -1;
    int whmin = 50, whmax = 179, wsmin = 0, wsmax = 73, wvmin = 221, wvmax = 255;
    int yhmin = 25, yhmax = 40, ysmin = 5, ysmax = 255, yvmin = 200, yvmax = 255;
    ManualSteeringControl(){
        // namedWindow("White Trackbar",(640,200));
        // createTrackbar("Hue min","White Trackbar",&whmin,179);
        // createTrackbar("Hue max","White Trackbar",&whmax,179);
        // createTrackbar("Sat min","White Trackbar",&wsmin,255);
        // createTrackbar("Sat max","White Trackbar",&wsmax,255);
        // createTrackbar("Val min","White Trackbar",&wvmin,255);
        // createTrackbar("Val max","White Trackbar",&wvmax,255);
        // namedWindow("Yellow Trackbar",(640,200));
        // createTrackbar("Hue min","Yellow Trackbar",&yhmin,179);
        // createTrackbar("Hue max","Yellow Trackbar",&yhmax,179);
        // createTrackbar("Sat min","Yellow Trackbar",&ysmin,255);
        // createTrackbar("Sat max","Yellow Trackbar",&ysmax,255);
        // createTrackbar("Val min","Yellow Trackbar",&yvmin,255);
        // createTrackbar("Val max","Yellow Trackbar",&yvmax,255);
    }
    //should be 720 x 1280 rbg image
    //do not read implementation. This was mixed from mutiple files and will not make sense.
    double get_steering_angle_from_mat(cv::Mat imageinRGB){
        double steeringAngle = 0;
        double carSpeed = 0.5;
        try
        {
            Rect crop(250, 450, 960, 270);
                Mat croppedImage;
                croppedImage = imageinRGB(crop);
                //White mins and maxes for Hue-Saturation-Value model

                Scalar wLower(whmin, wsmin, wvmin);
                Scalar wUpper(whmax, wsmax, wvmax);
                //Yellow mins and maxes for Hue-Saturation-Value model

                Scalar yLower(yhmin, ysmin, yvmin);
                Scalar yUpper(yhmax, ysmax, yvmax);

                //Create ImageProcessor (From Header file)
                ImageProcessor i;

                //Gets the blurred images for they white and yellow lines
                Mat yBlur = i.getBlur(yLower, yUpper, croppedImage);
                Mat wBlur = i.getBlur(wLower, wUpper, croppedImage);
                vector<vector<double> > yellowLaneLines;
                vector<vector<double> > whiteLaneLines;       

                Mat yErodeMat = i.getErode(yBlur);
                Mat wErodeMat = i.getErode(wBlur);

                //Processes images using Hough Transform and adds all slopes and intercepts along the bottom of image to the vector
                yellowLaneLines = i.processImage(yErodeMat);
                whiteLaneLines = i.processImage(wErodeMat);
                ROS_INFO("Yellow Lines: %s White Lines: %s",std::to_string(yellowLaneLines.size()).c_str(),std::to_string(whiteLaneLines.size()).c_str());

                imshow("Crop",croppedImage);
                imshow("Yellow",yErodeMat);
                imshow("White", wErodeMat);
                waitKey(3);


                //Finds total number of lanes found
                int numLanes = static_cast<int>(whiteLaneLines.size()) + static_cast<int>(yellowLaneLines.size());
                

                /** -------------------------------------------------**\
                * ----------------Steering Calculation----------------- *
                \**--------------------------------------------------**/

                
                //IF BOTH COLORS OF LANES ARE FOUND, REDETERMINE LANE (USING FUNCTION DEFINED BELOW)
                if (static_cast<int>(whiteLaneLines.size()) >= 1 && static_cast<int>(yellowLaneLines.size()) >= 1)
                {
                    laneNumber = laneFinder(whiteLaneLines, yellowLaneLines, laneNumber);
                }

                //CREATE STEERING CONTROLER FOR CONSTANT SPEED
                Control control;

                //Function returns steering angle and speed (always 0.75 in this case) based on line lines and lane number
                double steering_angle_prediction = control.steer(yellowLaneLines, whiteLaneLines, laneNumber);
                ROS_INFO("Steering Prediction %s",std::to_string(steering_angle_prediction).c_str());
                return steering_angle_prediction;      

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return 0;
        }
        return steeringAngle *-1;
        
    }
    int laneFinder(vector<vector<double> > wLines, vector<vector<double> > yLines, int lane)
    {
        //USES THE X COORDINATE OF EACH OF THE LINES TO DETERMINE LANE
        int laneFind = lane;
        if (yLines.size() > 0)
        {
            if (yLines.at(0).at(0) > 480)
            {
                laneFind = 0;
            }
            else
            {
                laneFind = 1;
            }
        }
        else if (wLines.size() > 0)
        {
            if (wLines.at(0).at(0) < 480)
            {
                laneFind = 0;
            }
            else
            {
                laneFind = 1;
            }
        }
        return laneFind;
    }
};