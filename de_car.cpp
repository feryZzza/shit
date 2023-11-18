#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#define PI 3.1415926

using namespace std;
using namespace cv;

int main()
{
    double heights[16];
    int armfind = 0;
    VideoCapture capture("/home/z/cv/v/finalfinaltest.webm");
    Mat frame, binary;
    RotatedRect RA[16], R[16];

    while(1)
    {

        capture>> frame;

        Mat hsv,gray;

        cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        GaussianBlur(hsv,hsv,cv::Size(11,11),2,2); // 高斯滤波过滤噪点
        inRange(hsv,cv::Scalar(0,151,191),cv::Scalar(210,255,255),gray);

        Mat struct1 = cv::getStructuringElement(0,cv::Size(4,4));
        Mat struct2 = cv::getStructuringElement(0,cv::Size(9,9));

        dilate(gray,gray,struct1);// 膨胀
        //dilate(gray,gray,struct2);

        imshow("test",gray);// 预处理测试

        frame.copyTo(binary);


        vector<vector<Point>> contours;
        findContours(gray, contours, RETR_LIST, CHAIN_APPROX_NONE);

        for (size_t i = 0; i < contours.size(); i++){

            vector<Point> points;
            double area = contourArea(contours[i]);
            if (area < 50 || 1e3 < area) continue;
            drawContours(frame, contours, static_cast<int>(i), Scalar(0), 2);

            double high;
            points = contours[i];

            RotatedRect rrect = fitEllipse(points);
            cv::Point2f* vertices = new cv::Point2f[4];
            rrect.points(vertices);

            for (int j = 0; j < 4; j++)
            {
                cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),4); // 画出识别的灯条
            }

             high = rrect.size.height;

             for(size_t j = 1;j < contours.size();j++)
             {

                 vector<Point> pointsA;
                 double area = contourArea(contours[j]);
                 if (area < 20 || 1e3 < area) continue;


                 double highA, distance;
                 double slop ;
                 pointsA = contours[j];

                 RotatedRect rrectA = fitEllipse(pointsA);

                 slop = abs(rrect.angle - rrectA.angle);
                 highA = rrectA.size.height;
                 distance = sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                 (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));


                 double max_height, min_height;
                 if(rrect.size.height > rrectA.size.height){
                     max_height = rrect.size.height;
                     min_height = rrectA.size.height;
                 }
                 else{
                     max_height = rrectA.size.height;
                     min_height = rrect.size.height;
                 }

                double line_x = abs(rrect.center.x-rrectA.center.x);
                double difference = max_height - min_height;
                double aim =   distance/((highA+high)/2);
                double difference3 = abs(rrect.size.width -rrectA.size.width);
                double height = (rrect.size.height+rrectA.size.height)/200;
                double slop_low = abs(rrect.angle + rrectA.angle)/2;


                if(( aim < 3.0 - height && aim > 2.0 - height && slop <= 5 && difference <=8 && difference3 <= 5
                      && (slop_low <= 30 || slop_low >=150) && line_x >0.6*distance))// 匹配两个灯条是装甲板的条件（已删除斜率匹配条件）
                {

                     heights[armfind] = (rrect.size.height+rrectA.size.height)/2;
                     R[armfind] = rrect;
                     RA[armfind] = rrectA;
                     armfind++;
                 }
             }
        }

        double max = 0;
        int mark;
        for(int i = 0;i < armfind;i++)// 如果多个目标存在，选择更近装甲板
        {     
            if(heights[i]  >= max)
            {
                max = heights[i];
                mark = i;
            }
        }

        if(armfind != 0)
        {
            cv::circle(binary,Point((R[mark].center.x+RA[mark].center.x)/2,
                       (R[mark].center.y+RA[mark].center.y)/2),
                       abs((R[mark].size.height+RA[mark].size.height)/4),cv::Scalar(0,0,255),4); // 装甲板中心点

           double center_x = (R[mark].center.x+RA[mark].center.x)/2;// 装甲板中心点x
           double center_y = (R[mark].center.y+RA[mark].center.y)/2;// 装甲板中心点y

           Point2f armor1[4],armor2[4];
           R[mark].points(armor1);
           RA[mark].points(armor2);

           line(binary,armor1[0],armor2[0],cv::Scalar(0, 255, 0),4);
           line(binary,armor1[2],armor2[2],cv::Scalar(0, 255, 0),4); // 画出装甲板
        }

        int KeyCode = waitKey(50);
        if (KeyCode == 32)
            break;
        imshow("okey",binary);

        armfind = 0;
    }
}