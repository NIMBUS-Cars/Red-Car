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

using namespace std;

class LaneDetector
{
    double line_seperation_distance = 0.3; // 300px / 1280px from testing
    double image_width = 1280.0;
    double min_height_percent_to_consider_line = 240;
public:
    Mat getErode(Mat yBlurredImage)
    {
        Mat yEdgeImage;
        Canny(yBlurredImage, yEdgeImage, 100, 200);
        Mat yDilatedImage;
        Mat ykernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        Mat yExtraBlurMat;
        dilate(yEdgeImage, yDilatedImage, ykernel);
        GaussianBlur(yDilatedImage, yExtraBlurMat, Size(7, 7), 0);
        Mat yErodeMat;
        erode(yExtraBlurMat, yErodeMat, ykernel);

        return yErodeMat;
    }
    Mat getBlur(Scalar yLower, Scalar yUpper, Mat croppedImage)
    {
        Mat imageInHSV;
        cvtColor(croppedImage, imageInHSV, COLOR_BGR2HSV);
        Mat yColorFiltered;
        inRange(imageInHSV, yLower, yUpper, yColorFiltered);
        Mat yBlurredImage;
        GaussianBlur(yColorFiltered, yBlurredImage, Size(5, 5), 0);
        return yBlurredImage;
    }
    vector<vector<double>> processImage(Mat erodeMat)
    {
        vector<int> countOfYellowLinesAddedToEachLane;
        vector<vector<double>> yellowLaneLines;

        if (erodeMat.size().height > 1)
        {
            vector<Vec4i> yellowLines;
            vector<vector<Point2d>> yellowPointsForLines;
            for(int i = 0; i<3;i++){
                vector<Vec4i> linesFromOneRun;
                HoughLinesP(erodeMat, linesFromOneRun, 1, CV_PI / 180, 70, 30, 10);
                for(size_t j = 0; j<linesFromOneRun.size();j++){
                    yellowLines.push_back(linesFromOneRun.at(j));
                }
            }

            for (size_t i = 0; i < yellowLines.size(); i++)
            {   
                if(yellowLines[i][1] > min_height_percent_to_consider_line || yellowLines[i][3] > min_height_percent_to_consider_line){
                    vector<Point2d> newPoint;
                    Point2d beginning(yellowLines[i][0], yellowLines[i][1]);
                    Point2d end(yellowLines[i][2], yellowLines[i][3]);
                    if (end.y > beginning.y)
                    {
                        newPoint.push_back(end/image_width);
                        newPoint.push_back(beginning/image_width);
                    }
                    else
                    {
                        newPoint.push_back(beginning/image_width);
                        newPoint.push_back(end/image_width);
                    }
                    yellowPointsForLines.push_back(newPoint);
                }
            }

            for (int j = 0; j < static_cast<int>(yellowPointsForLines.size()); j++)
            {
                int indexOfCorrespondence = -1;
                for (int i = 0; i < static_cast<int>(yellowLaneLines.size()); i++)
                {
                    //averages the slope and x distance of all yellowLines in the same area
                    if (abs(yellowPointsForLines.at(j).at(0).x - yellowLaneLines.at(i).at(0)) < line_seperation_distance)
                    {
                        double oldX = yellowLaneLines.at(i).at(0);
                        double oldSlope = yellowLaneLines.at(i).at(1);
                        double newSlope = slope(yellowPointsForLines.at(j).at(0), yellowPointsForLines.at(j).at(1));
                        double newX = yellowPointsForLines.at(j).at(0).x;
                        yellowLaneLines.at(i).at(0) = (yellowLaneLines.at(i).at(0) * countOfYellowLinesAddedToEachLane.at(i) + yellowPointsForLines.at(j).at(0).x) / (countOfYellowLinesAddedToEachLane.at(i) + 1);
                        yellowLaneLines.at(i).at(1) = (yellowLaneLines.at(i).at(1) * countOfYellowLinesAddedToEachLane.at(i) + slope(yellowPointsForLines.at(j).at(0), yellowPointsForLines.at(j).at(1))) / (countOfYellowLinesAddedToEachLane.at(i) + 1);

                        double averageX = yellowLaneLines.at(i).at(0);
                        double averageSlope = yellowLaneLines.at(i).at(1);

                        countOfYellowLinesAddedToEachLane.at(i)++;
                        indexOfCorrespondence = j;
                    };
                }
                if (indexOfCorrespondence == -1)
                {
                    // add first line when it doesnt match any of the others that already exist
                    vector<double> xPointAndSlope;
                    xPointAndSlope.push_back(yellowPointsForLines.at(j).at(0).x);
                    xPointAndSlope.push_back(slope(yellowPointsForLines.at(j).at(0), yellowPointsForLines.at(j).at(1)));
                    yellowLaneLines.push_back(xPointAndSlope);
                    countOfYellowLinesAddedToEachLane.push_back(1);
                }
            }
        }
        return yellowLaneLines;
    }
    double slope(Point2d first,Point2d second){
    // slope is taken such that horizontal side of camera is y axis where right side is positive and left side is negative
    // vertical side of camera is x axis where top side would be positve and bottom side is negative
    // so 0,0 to 1,1 should output -1
    // any line leaning left in the camera will have a negative slope
    // any line leaning right in the camera should have a positive slope
    // slopes close to zero are straight lines
    double secondX = second.x;
    double secondY = second.y;
    double firstX  = first.x;
    double firstY = first.y;
        if(second.y - first.y == 0 ){
        return 0;
        }
        return (-1.0 * (second.x - first.x )) / (second.y - first.y);
    }
};