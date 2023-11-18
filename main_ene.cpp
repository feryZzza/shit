#include "NetworkManager.h"
//#include "Kalm.h"
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <deque>  


bool reg = false;
bool find;
float a = 35;

class LimitedDeque {  
private:
    std::deque<int> data;  
    int max_size;  
public:  
    LimitedDeque(int size) : max_size(size) {}  
      
    void insert(int value) {  
        if (data.size() >= max_size) {  
            data.pop_front();
            for (int i = 0; i < data.size(); ++i) {
                data[i] = data[i+1];
            }  
        }
        data.push_back(value); 
    }

void display() {    
    int difference;  
    for (int i = 1; i < data.size(); ++i)
     {    
        difference += std::abs(data[i] - data[i-1]);  
     } 

     if (difference <= 4)
     {  
        a = 13.5;  
     }
     else if (difference >= 50 && difference < 270)
     {  
       a = 48;  
     }  
     else if (difference >= 270)
     {
        a = 35;
     }
} 
};



void sigint_handler(int sig) { exit(1); }

int main() {
  signal(SIGINT, sigint_handler);

  // 创建 NetworkManager 对象
  network::NetworkManager net("127.0.0.1", 20205654, "梗小姐赢麻了", 5558,
                              5559);

  // 注册视觉程序
  reg = net.registerUser(0, 100000);



  if (!reg) {
    std::cout << "Register Failed, Exit" << std::endl;
    exit(1);
  }

  std::cout << "Register Success! the client will start at 2 second" << std::endl;

  sleep(2);

  auto time_system_start = std::chrono::system_clock::now();

  auto time_temp = std::chrono::system_clock::now();


   LimitedDeque recordx(3);

  while (true) {

    cv::Mat img;
    auto message = net.getLatestRecvMessage();
    auto time_now = std::chrono::system_clock::now();
    float x_temp[10],y_temp[10];
    int k = 0;

    if (!message.img.empty()) {
      std::cout<<"接受到消息！！！"<<std::endl;
      /*******************************************************************************/
      /*                               YOUR CODE STARTS */
      /*******************************************************************************/
      cv::Mat frame = message.img;
      cv::Mat gray,hsv;

      /*-------------------------------------------------v相机内参v---------------------------------------------------------*/
      cv::Size size_ori = message.img.size();
      double angle = 60 * M_PI / 180.0;
      float fx = size_ori.width / (2 * tan(angle));
      float fy = size_ori.height / (2 * tan(angle));
      float cx = size_ori.width / 2;
      float cy = size_ori.height / 2;

      cv::Mat camera = (cv::Mat_<double>(3,3)<< fx , 0  , cx,
                                              0  , fy , cy,
                                              0  , 0  , 1 );

      /*-----------------------------------------------------v模板匹配v-------------------------------------------------------*/

      cv::Mat temp = cv::imread("/home/z/cv/T-DT_CampusGame_Client_2023-main/module6.png");
      cv::Mat result;
      int matchMethod = cv::TM_CCOEFF_NORMED;
      cv::matchTemplate(frame,temp,result,matchMethod);
      double minVal,maxVal;
      cv::Point minLoc, maxLoc;
      cv::minMaxLoc(result,&minVal,&maxVal,&minLoc,&maxLoc);
      double threshold = 0.7;
      cv::Point2f centerR;

      if(maxVal > threshold)
      {
        centerR.x = (maxLoc.x + 0.5*temp.cols);
        centerR.y = (maxLoc.y + 0.5*temp.rows);
        cv::rectangle(frame,cv::Point(maxLoc.x,maxLoc.y),cv::Point(maxLoc.x+temp.cols,maxLoc.y+temp.rows),cv::Scalar(0,0,255),2);
        std::cout << "R has been found" << std::endl;
      }


      /*--------------------------------------------------v图像预处理v--------------------------------------------------------*/

      cv::Point2f center(maxLoc.x + temp.cols/2 , maxLoc.y + temp.rows/2);// R心

      cv::Mat black = cv::Mat::zeros(message.img.size(), CV_8UC1);
      cv::circle(black, center, 57, cv::Scalar(255), -1); 
      cv::Mat Roi;
      frame.copyTo(Roi, black);
      //cv::circle(frame,center,60,cv::Scalar(255,255,255),2);

      cv::cvtColor(Roi,hsv,cv::COLOR_BGR2HSV);
      cv::GaussianBlur(hsv,hsv,cv::Size(9,9),2,2);
      cv::inRange(hsv,cv::Scalar(87,90,207),cv::Scalar(105,255,255),gray);

      cv::Mat struct1 = cv::getStructuringElement(0,cv::Size(4,4));
      cv::Mat struct2 = cv::getStructuringElement(0,cv::Size(8,8));

      cv::dilate(gray,gray,struct1);

      /*------------------------------------------------------v寻找流水灯v----------------------------------------------------*/

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
      
      cv::Point2f rectcorner;

      for (int i = 0; i < contours.size(); i++)
      {
        double area = cv::contourArea(contours[i]);
        cv::Rect rect = cv::boundingRect(contours[i]);
        if (area > 200 && area < 600)
        {
          cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);
          cv::Point2f vertices[3];
          rotatedRect.points(vertices);
          float dis;
          for (int j = 0; j < 4; j++)
          {
            rectcorner = vertices[j];
            dis = std::sqrt((rectcorner.x - center.x)*(rectcorner.x - center.x) + (rectcorner.y - center.y)*(rectcorner.y - center.y));
            if(dis > 50)
            {
              x_temp[k] = rectcorner.x;
              y_temp[k] = rectcorner.y;
              k++;
            }
          }
          find = true;
          std::cout << "HIT find:" << find <<std::endl;
        }
        else
        {
          find = false;
        }
      }
      contours.clear();


      /*------------------------------------------------------v标定击打点v----------------------------------------------------*/

      cv::Point2f hit_inside(x_temp[1] ,y_temp[1]);

      double S = cv::norm(hit_inside - center);

      cv::Point2f direction = (hit_inside - center) / S;

      cv::Point2f hit_center = center + static_cast<int>(1.5 * S) * direction;
      
      /*------------------------------------------------------v标定预测点v----------------------------------------------------*/
-
      cv::Point2f pixel; 
      pixel = hit_center;

      recordx.insert(hit_center.x);
      recordx.display();
      float new_x;
      float new_y;

      for (int i = 0; i < 1; i++) {
        // 计算相对于中心点的极坐标半径和角度
        float delta_x = pixel.x - center.x;
        float delta_y = pixel.y - center.y;
        float radius = std::sqrt(delta_x * delta_x + delta_y * delta_y);  // 极坐标半径
        float angle = std::atan2(delta_y, delta_x);  // 极坐标角度

        // 在极坐标中进行旋转
        angle += (32.5 * CV_PI / 180.0); // 旋转（弧度）

        // 将极坐标结果转换回笛卡尔坐标
        new_x = center.x + radius * cos(angle);
        new_y = center.y + radius * sin(angle);

        //cv::circle(frame, cv::Point(new_x, new_y), 3, cv::Scalar(0, 0, 255), 2); // 预测点
      }

      /*--------------------------------------------------------v单点解算v----------------------------------------------------*/

      cv::Mat caminv = camera.inv();
      cv::Mat xaingsu = (cv::Mat_<double>(3,1)<< new_x,new_y,1);//解算点的像素xy值
      cv::Mat camerxi =  caminv * xaingsu;

      float X = camerxi.at<double>(0);
      float Y = camerxi.at<double>(1);
      float Z = camerxi.at<double>(2);

      double theta_x = std::atan2(X,Z);
      double theta_y = std::atan2(-Y,std::sqrt(X*X + Z*Z));

      double theta_x_deg = theta_x * (60 / M_PI);//yaw
      double theta_y_deg = theta_y * (60 / M_PI);//pitch

      /*--------------------------------------------------------^单点解算^-----------------------------------------------------*/

      // 发送控制信息
      int fire = 1;

      //cv::imshow("img", frame);
      //cv::waitKey(1);

      float  rotaion_speed = 1;

      auto time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now-time_temp);

      time_temp=time_now;

      int shoot_rate=1;

      std::cout<<"time "<<message.pitch<<std::endl;

      float sendyaw = theta_x_deg + message.yaw + 0.075;
      float sendpitch = theta_y_deg + message.pitch + 0.05;
      if(sendpitch > 360)
      {
        sendpitch = sendpitch - (float)360;
      }

        sendpitch = sendpitch * (1.155);

      if ((new_x - center.x)< 0 && (new_y - center.y)>-20)
      {
        sendpitch = sendpitch + 0.4;
        if (a == 13.5)
        {
          sendpitch = sendpitch + 0.2;
        }
      }

      if((center.y - new_y)>30 && (center.x - new_x) < 0 && a == 13.5)
      {
        sendpitch = sendpitch - 0.15;
        sendyaw = sendyaw + 0.15;

      }
      

      std::cout << sendpitch << "    " << sendyaw << std::endl;

      if (sendpitch <= 60)
      {
        net.sendControlMessage(network::SendStruct(sendyaw,sendpitch, fire,1,1280,720));
      }
      else if(sendpitch > 60){
        net.sendControlMessage(network::SendStruct(message.yaw,5.0, fire,1,1280,720));
      }
      
    } else {
        std::cout << "Get an empty image~" << std::endl;
    }
  }
  return 0;
}
