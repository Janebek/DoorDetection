#include "rs-measure.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include<librealsense2/rsutil.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

//#include "geometry_msgs/PoseStamped.h" //include posestamp head file
#include "geometry_msgs/PoseArray.h"

#include "quaternions.h"  //get quaternions(x, y, z, w)
#include "readPython.h"   //get position(x, y, z)

#include <signal.h>
#include <geometry_msgs/Twist.h>
 
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>

#define  PI 3.1415926

using namespace std;
using namespace cv;






struct Output {
	int id;  //结果类别id
	double confidence;  //结果置信度
	cv::Rect box;  //矩形框
};

struct center{
    int x;
    int y;
};

float * get_coordinate(const rs2::depth_frame& depth,
                            const state& s)
{

    // float *p = new float;
    float *p ;
    pixel from_pixel = {s.detect_point.x,s.detect_point.y};

    p = compute_coordinate(depth, from_pixel);

    return p;

}

float * compute_coordinate(const rs2::depth_frame& frame, pixel u)
{
    float upixel[2]; // From pixel
    float upoint[3]; // From point (in 3D)

    float* point = new float[3];

    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = u.first;
    upixel[1] = u.second;

    float udist = frame.get_distance(upixel[0], upixel[1]);

    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data

    rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

    point[0] = upoint[0]*100;
    point[1] = upoint[1]*100;
    point[2] = upoint[2]*100;

    return point ;
   
}

// void compute_coordinate(const rs2::depth_frame& frame, pixel u)
// {
//     float upixel[2]; // From pixel
//     float upoint[3]; // From point (in 3D)

//     // Copy pixels into the arrays (to match rsutil signatures)
//     upixel[0] = u.first;
//     upixel[1] = u.second;

//     float udist = frame.get_distance(upixel[0], upixel[1]);

//     rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data

//     rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

//     cout<<"坐标为："<<"("<<upoint[0]*100<<","<<upoint[1]*100<<","<<upoint[2]*100<<")"<<endl;;

   
// }

// void get_coordinate(const rs2::depth_frame& depth,
//                             const state& s)
// {

//     pixel from_pixel = {s.detect_point.x,s.detect_point.y};

//     compute_coordinate(depth, from_pixel);

   

// }

class DoorDetect{
public:
    DoorDetect(string modelPath, int classNum=4, double nmsThreshold=0.45, double boxThreshold=0.25, double classThreshold=0.25){
        struct stat s;
        stat(modelPath.c_str(), &s);
        if(stat(modelPath.c_str(), &s) == 0 && (s.st_mode & S_IFREG) && modelPath.substr(modelPath.length() - 4, 4) == "onnx"){
            model = dnn::readNetFromONNX(modelPath);
        }
        else{
            cout << "Model load error!" << endl;
            exit(-1);
        }
        assert(classNum > 0);
        assert(nmsThreshold >= 0 && nmsThreshold <= 1);
        assert(boxThreshold >= 0 && boxThreshold <= 1);
        assert(classThreshold >= 0 && classThreshold <= 1);

        (*this).classNum = classNum;
        (*this).nmsThreshold = nmsThreshold;
        (*this).boxThreshold = boxThreshold;
        (*this).classThreshold = classThreshold;
        (*this).netAnchors = { { 10.0, 13.0, 16.0, 30.0, 33.0, 23.0 }, { 30.0, 61.0, 62.0, 45.0, 59.0, 119.0 }, { 116.0, 90.0, 156.0, 198.0, 373.0, 326.0 } };
        (*this).netStride = {8.0, 16.0, 32.0};
        (*this).imgWidth = 640;
        (*this).imgHeight = 640;
        (*this).ratioHeight = 0.0;
        (*this).ratioWidth = 0.0;

    }

    vector<Output> Detect(Mat img){

        Size newShape = {imgWidth, imgHeight};
        Mat inputImg = ImagePreprocess(img, newShape);

        double r = min((double)newShape.width / img.size().width, (double)newShape.height / img.size().height);
        int topPad = (int)((newShape.height - r * img.size().height) / 2 - 0.1);
        int leftPad = (int)((newShape.width - r * img.size().width) / 2 - 0.1);

        model.setInput(inputImg);
        //cout<<inputImg.rows<<" "<<inputImg.cols<<endl;
        Mat modelOutput;
        model.forward(modelOutput);
        //cout << "Model outupt successfully!" << endl;

        int numberOfOutput = classNum + 5;
        vector<int> classIds;
        vector<float> confidences;
        vector<Rect> boxes;
        float *pData = (float*)modelOutput.data;
        for(int stride = 0; stride < netStride.size(); ++stride){
            //cout << "current stride = " << stride << endl;
            int gridX = (int)(imgWidth / (*this).netStride[stride]);
            int gridY = (int)(imgHeight / (*this).netStride[stride]);

            for(int anchor = 0; anchor < netAnchors.size(); ++anchor){
                int anchorWidth = (*this).netAnchors[stride][anchor * 2];
                int anchorHeight = (*this).netAnchors[stride][anchor * 2 + 1];

                for(int i = 0; i < gridY; ++i){
                    for(int j = 0; j < gridX; ++j){
                        float boxScore = pData[4];
                        if(boxScore > boxThreshold){
                            Mat scores(1, classNum, CV_32FC1, pData + 5);
                            Point classIdPoint;
                            double maxClassScore;
                            minMaxLoc(scores, 0, &maxClassScore, 0, &classIdPoint);

                            maxClassScore = (float)maxClassScore;

                            //0: Door, 1: Handle, 2: cabinet door, 3: refrigerator door
                            if(maxClassScore > classThreshold && classIdPoint.x == 0){
                                float x = (pData[0] * 2.0 - 0.5 + j) * netStride[stride];
                                float y = (pData[1] * 2.0 - 0.5 + i) * netStride[stride];
                                float w = powf(pData[2] * 2.0, 2.0) * anchorWidth;
                                float h = powf(pData[3] * 2.0, 2.0) * anchorHeight;

                                int left = (x - 0.5 * w - leftPad) * (*this).ratioWidth;
                                int top = (y - 0.5 * h - topPad) * (*this).ratioHeight;

                                classIds.push_back(classIdPoint.x);
                                confidences.push_back(maxClassScore * boxScore);
                                boxes.push_back(Rect(left, top, int(w * (*this).ratioWidth), int (h * (*this).ratioHeight)));
                            }
                        }

                        pData += numberOfOutput;
                    }
                }
            }
        }

        vector<int> nms_result;
        vector<Output> output;

	    dnn::NMSBoxes(boxes, confidences, classThreshold, nmsThreshold, nms_result);
	    for (int i = 0; i < nms_result.size(); i++) {
		    int idx = nms_result[i];
		    Output result;
		    result.id = classIds[idx];
		    result.confidence = confidences[idx];
		    result.box = boxes[idx];
		    output.push_back(result);
	    }
        return output;
    }

    Mat ImagePreprocess(Mat Inimg, Size newShape=Size(640, 640)){
        Mat img = LetterBox(Inimg, newShape);
        //Mat img = Inimg;
        //cvtColor(img, img, COLOR_BGR2RGB);   OpenCV4
        cvtColor(img, img, CV_BGR2RGB); //OpenCV3
        img.convertTo(img, CV_32FC3, 1 / 255.0);
        img = dnn::blobFromImage(img);

        return img;
    }

    Mat LetterBox(Mat img, Size newShape={640, 640}, Scalar color={114, 114, 114}, bool minimumRectangle=false, bool scaleFill=false, bool scaleUp=true, int stride=32){
        Mat newImg = img.clone();
        Size shape = img.size();

        double r = min((double)newShape.width / shape.width, (double)newShape.height / shape.height);
        if(!scaleUp){
            r = min(1.0, r);
        }

        Size newUnpad = Size(int(round(shape.width * r)), int(round(shape.height * r)));
        int dw = newShape.width - newUnpad.width, dh = newShape.height - newUnpad.height;

        (*this).ratioHeight = (double)shape.height / newUnpad.height;
        (*this).ratioWidth = (double)shape.width / newUnpad.width;

        if(minimumRectangle){
            dw = dw % stride;
            dh = dh % stride;
        }
        else if(scaleFill){
            dw = 0; dh = 0;
            newUnpad = Size(newShape.width, newShape.height);
            double ratioWidth = (double)newUnpad.width / shape.width;
            double ratioHeight = (double)newUnpad.height / shape.height;
        }

        dw /= 2;
        dh /= 2;

        if(shape != newUnpad){
            //resize(img, newImg, newUnpad, 0, 0, INTER_LINEAR);   //OpenCV4
            resize(img, newImg, newUnpad, 0, 0, CV_INTER_LINEAR); //OpenCV3
        }

        int top = int(round(dh - 0.1)), bottom = int(round(dh + 0.1));
        int left = int(round(dw - 0.1)), right = int(round(dw + 0.1));
        copyMakeBorder(newImg, newImg, top, bottom, left, right, BORDER_CONSTANT, color);
        //cout << newImg.rows << " " << newImg.cols << endl;
        // imshow("New Image", newImg);
        // waitKey();
        // destroyWindow("New Image");

        return newImg;
    }

    void DrawPred(Mat &img, const vector<Output>& result, vector<Scalar> color) {
	    for (int i = 0; i < result.size(); i++) {
		    int left, top;
		    left = result[i].box.x;
		    top = result[i].box.y;
		    int color_num = i;
		    rectangle(img, result[i].box, color[result[i].id], 2, 8);

		    string label = to_string(result[i].id) + ": " + to_string(result[i].confidence);
							 
		    int baseLine;
		    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.1, 0.5, &baseLine);
		    top = max(top, labelSize.height);
		    //rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
		    putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 1, color[result[i].id], 2);
	    }
	    imshow("Detection Result", img);
	    //imwrite("out.bmp", img);
	    waitKey(1);
	    //destroyAllWindows();
    }

    double Sigmoid(double x){
        return 1 / (1 + exp(-x));
    }

private:
    dnn::Net model;
    vector<vector<double>> netAnchors;
    vector<double> netStride;
    int classNum;
    double nmsThreshold;
    double boxThreshold;
    double classThreshold;
    int imgWidth;
    int imgHeight;
    double ratioWidth;
    double ratioHeight;
};

void ShowTwoImages(Mat &img1, Mat &img2){
    Mat twoImages = Mat::zeros(Size(img1.cols * 2, img1.rows), img1.type());
    Rect roi(0, 0, img1.cols, img1.rows);
    img1.copyTo(twoImages(roi));
    roi.x = img1.cols + 10;
    roi.height = img2.rows;
    roi.width = img2.cols;
    img2.copyTo(twoImages(roi));
    imshow("Two Images", twoImages);
    waitKey(0);
} 

//ROS相关方法
double getTimeNow()
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch());
    std::chrono::nanoseconds nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time.time_since_epoch());

    //std::cout<<"now (s):"<<sec.count()<<std::endl;
    // std::cout<<"now (ns):"<<nsec.count()<<std::endl;
    return nsec.count()*10e-10;
}

ros::Publisher cmdVelPub;
void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());//使机器人停止运动
  ROS_INFO("move_turtle_goforward ended!");
  ros::shutdown();
}

int cloud_flag = 0;

void chatterCallback(const geometry_msgs::PoseArray msg)
{
    for(int i=0;i<msg.poses.size();i++){
        geometry_msgs::Pose pose = msg.poses[i];
        ROS_INFO("I heard the pose from the robot"+i); 
        ROS_INFO("the position(x,y,z) is %lf , %lf, %lf", pose.position.x, pose.position.y, pose.position.z);
        ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
    }
    //ROS_INFO("I heard the pose from the robot"); 
    //ROS_INFO("the position(x,y,z) is %lf , %lf, %lf", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg[0]->pose.orientation.w);
    //ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

    cloud_flag = 1;
    system("rosnode kill /target_pose");

 
    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
}


 // Include RealSense Cross Platform API
          // Include short list of convenience functions for rendering

int main(int argc,char * argv[])
{
    int a,b,c,d,e,f,g,h,l,m;
    float *x1,*x2,*x3,*x4,*x5,*z;
    mutex my_mutex;
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_COLOR,RS2_STREAM_DEPTH }, serial))
        return EXIT_SUCCESS;

    // OpenGL textures for the color and depth frames
    texture depth_image, color_image;

    // cout << "coordinate"<<endl ;

    // Colorizer is used to visualize depth data
    rs2::colorizer color_map;
    // Use black to white color map
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    // Decimation filter reduces the amount of data (while preserving best samples)
    rs2::decimation_filter dec;
    // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // but you can also increase the following parameter to decimate depth more (reducing quality)
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // Define transformations from and to Disparity domain
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    // Define spatial filter (edge-preserving)
    rs2::spatial_filter spat;
    // Enable hole-filling
    // Hole filling is an agressive heuristic and it gets the depth wrong many times
    // However, this demo is not built to handle holes
    // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
    spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    // Define temporal filter
    rs2::temporal_filter temp;
    // Spatially align all streams to depth viewport
    // We do this because:
    //   a. Usually depth has wider FOV, and we only really need depth for this demo
    //   b. We don't want to introduce new holes
    rs2::align align_to(RS2_STREAM_DEPTH);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    // cfg.enable_stream(RS2_STREAM_DEPTH);
    // cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
    auto profile = pipe.start(cfg);

    auto sensor = profile.get_device().first<rs2::depth_sensor>();

    // Set the device to High Accuracy preset of the D400 stereoscopic cameras
    if (sensor && sensor.is<rs2::depth_stereo_sensor>())
    {
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    }

    // Define application state and position the ruler buttons
    state app_state1,app_state2,app_state3,app_state4,app_state5; //Pixel 类型
    center center;

    DoorDetect doorDetect = DoorDetect("./model/yolov5nDoorDetectShapeTest.onnx");

    float a_x,a_y,a_z,b_x,b_y,b_z,c_x,c_y,c_z,D,dis,t1,t2,x_1,y_1,z_1,x_2,y_2,z_2,temp1,temp2,temp_x_1,temp_y_1,temp_z_1,angle_1,angle_1_dir,angle_2,angle_2_dir,juli,temp_x_2,temp_y_2,temp_z_2;
    float door_width;
    std::thread coordinate_processing_thread([&]() {
        while (true)
        {

            rs2::frameset data;
            pipe.poll_for_frames(&data);

            //Get each frame
            rs2::frame color_frame = data.get_color_frame();

            // Creating OpenCV Matrix from a color image
            Mat colori(Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            // imshow("before",colori);
            // waitKey(1);
            //1.将图像分辨率降低至６４０＊４８０，输入ｄｅｔｅｃｔ函数
            Mat colori_resize;
            Size dsize = Size(640,480);
            resize(colori,colori_resize,dsize,0,0,INTER_AREA);
            // imshow("after",colori_resize);
            // waitKey(1);

            vector<Output> res = doorDetect.Detect(colori_resize);
            //vector<Output> res = doorDetect.Detect(colori);

            //2.将６４０＊４８０中的坐标位置放大，乘一个缩放系数，纵坐标x乘1.5，横坐标y坐标乘２;
            if(!res.empty()){
                center.x = int(((res[0].box.tl().x + res[0].box.br().x)/2)*1.5) ;
                center.y = int(((res[0].box.tl().y + res[0].box.br().y)/2)*2) ;
                // center.x = (res[0].box.tl().x + res[0].box.br().x)/2 ;
                // center.y = (res[0].box.tl().y + res[0].box.br().y)/2 ;
                a=center.x;
                b=center.y;
                c=center.x - 7;
                d=center.y - 7;
                e=center.x + 10;
                f=center.y + 10;
                g= res[0].box.tl().x * 2;   //创造变量计算门宽
                h= res[0].box.tl().y * 1.5;
                l= res[0].box.br().x * 2;
                m= res[0].box.tl().y * 1.5;
                cout<<"  "<<endl;
                app_state1.detect_point={a,b};  
                app_state2.detect_point={c,d};
                app_state3.detect_point={e,f};          
                app_state4.detect_point={g,h};
                app_state5.detect_point={l,m};
            }
            
            // cout<<"tl.x = "<<g<< "  tl.y ="<<h<<endl;
            // cout<<"tr.x = "<<l<< "  tr.m ="<<h<<endl;

            doorDetect.DrawPred(colori_resize,res,{255,0,0});
                     
            data = data.apply_filter(align_to);
            auto depth1 = data.get_depth_frame();  
            // cout<<"before "<<depth1.get_width()<<endl;
            // cout<<"before "<<depth1.get_height()<<endl;
            //data = data.apply_filter(dec);


            // To make sure far-away objects are filtered proportionally
            // we try to switch to disparity domain
            data = data.apply_filter(depth2disparity);

            // Apply spatial filtering
            data = data.apply_filter(spat);

            // Apply temporal filtering
            data = data.apply_filter(temp);

            // If we are in disparity domain, switch back to depth
            data = data.apply_filter(disparity2depth);

            //// Apply color map for visualization of depth
            //data = data.apply_filter(color_map);

            // Send resulting frames for visualization in the main thread

            auto depth = data.get_depth_frame();
            // cout<<"after"<<depth.get_width()<<endl;
            // cout<<"after"<<depth.get_height()<<endl;
            // auto color = data.get_color_frame();


            // get coordinate
            if(!res.empty()){
            x1 = get_coordinate(depth, app_state1);//获取到门中心点３D坐标
            x2 = get_coordinate(depth, app_state2);//获取到门中心左侧点３Ｄ坐标　
            x3 = get_coordinate(depth, app_state3);//获取到门中心点右侧３Ｄ坐标
            x4 = get_coordinate(depth, app_state4);
            x5 = get_coordinate(depth, app_state5);
            // cout<<"坐标1为("<<*x1<<","<<*(x1+1)<<","<<*(x1+2)<<")"<<endl ;                
            // cout<<"坐标2为("<<*x2<<","<<*(x2+1)<<","<<*(x2+2)<<")"<<endl ; 
            // cout<<"坐标3为("<<*x3<<","<<*(x3+1)<<","<<*(x3+2)<<")"<<endl ; 
            door_width = sqrt(pow(*x4 - *x5, 2.f) +
                             pow(*(x4+1) - *(x5+1), 2.f) +
                             pow(*(x4+2) - *(x5+2), 2.f)) ;
            cout<<"门宽为"<<door_width<<"cm"<<endl;     //1.门的宽度测量还行，误差大概15%，在真值附近15%跳动，还需增加滤波操作，使其更加稳定
                                                       //2.还有一个问题是框住门的框不稳定，会来回晃动，这也导致了门宽度测量不稳定
            // a_x = *x3 - *x1;
            // a_y = *(x3+1) - *(x1+1);
            // a_z = *(x3+2) - *(x1+2);
            // b_x = *x2 - *x1;
            // b_y = *(x2+1) - *(x1+1);
            // b_z = *(x2+2) - *(x1+2);//计算两个向量
            // c_x = a_y*b_z - a_z*b_y;
            // c_y = a_z*b_x - a_x*b_z;
            // c_z = a_x*b_y - a_y*b_x;
            // D = -c_x*(*x1)-c_y*(*(x1+1))-c_z*(*(x1+2));//两向量叉积得法向量
            // dis = 100*sqrt(c_x*c_x+c_y*c_y+c_z*c_z) ; //100代表100cm指门前一米，具体可参见点到平面距离公式
            // t1 = (dis-c_x*(*x1)-c_y*(*(x1+1))-c_z*(*(x1+2))-D)/(c_x*c_x+c_y*c_y+c_z*c_z) ;
            // t2 = (-dis-c_x*(*x1)-c_y*(*(x1+1))-c_z*(*(x1+2))-D)/(c_x*c_x+c_y*c_y+c_z*c_z) ;
            // x_1 = *x1 + c_x*t1;
            // y_1 = *(x1+1) + c_y*t1;
            // z_1 = *(x1+2) + c_z*t1;
            // x_2 = *x1 + c_x*t2;
            // y_2 = *(x1+1) + c_y*t2;
            // z_2 = *(x1+2) + c_z*t2;
            // temp1 = (x_1-*x1)*c_x + (y_1-*(x1+1))*c_y + (z_1-*(x1+2))*c_z; 
            // // temp2 = (x_2-*x1)*c_x + (y_2-*(x1+1))*c_y + (z_2-*(x1+2))*c_z; 
            // if(temp1>0){
            //     temp_x_1 = x_1;
            //     temp_y_1 = y_1;
            //     temp_z_1 = z_1;
            // }
            // else
            // {
            //     temp_x_1 = x_2;
            //     temp_y_1 = y_2;
            //     temp_z_1 = z_2;
            // }

            // //cout<<"門前一米处坐标为 ("<<temp_x_1<<","<<0<<","<<temp_z_1<<")"<<endl;
            // angle_1 = (acos(temp_z_1/sqrt(temp_x_1*temp_x_1 + temp_z_1*temp_z_1)))/(2*PI) * 360;
            // if(temp_x_1<0){
            //     //cout<<"小车第一次需要逆时针旋转"<<angle_1<<"度"<<endl;
            //     angle_1_dir=angle_1;
            // }
            // else{
            //     //cout<<"小车第一次需要顺时针旋转"<<angle_1<<"度"<<endl;
            //     angle_1_dir=-angle_1;
            // }
            // juli = sqrt(temp_x_1*temp_x_1 + temp_z_1*temp_z_1);
            // //cout<<"小车需要行进的距离为"<<juli<<"cm"<<endl;

            // temp_x_2 = *x1 - temp_x_1;
            // temp_y_2 = *(x1+1) - temp_y_1;
            // temp_z_2 = *(x1+2) - temp_z_1;
            // angle_2 = (acos((temp_x_2*temp_x_1+temp_z_2*temp_z_1)/(sqrt(temp_x_1*temp_x_1 + temp_z_1*temp_z_1)+sqrt(temp_x_2*temp_x_2+temp_y_2*temp_y_2+temp_z_2*temp_z_2))))/(2*PI) * 360;

            // if(temp_x_1*temp_z_2 - temp_z_1*temp_x_2>0){
            //     //cout<<"小车第二次需要逆时针旋转"<<angle_2<<"度"<<endl;
            //     angle_2_dir=angle_2;
            // }
            // else{
            //     //cout<<"小车第二次需要顺时针旋转"<<angle_2<<"度"<<endl;
            //     angle_2_dir=-angle_2;
            // }
            
            delete [] x1;
            delete [] x2;
            delete [] x3;
            delete [] x4;
            delete [] x5;

            }
    }
    });

    // std::thread out([&]() {
    // while(true){
    //     my_mutex.lock();
    // // cout<<"目标点的坐标为"<<*x<<endl ;
    // for(int i=0;i<50;i++){
    // if(i=49){
    //     z=x1;
    //     i=0;
    // }
    // }
    // my_mutex.unlock();
    // }
    // });

// std::thread publish([&]() {
//   std::this_thread::sleep_for (std::chrono::seconds(30));

//   ros::init(argc, argv, "publisher");
//   ros::NodeHandle n;
//   //ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("target_pose", 10); //initialize chatter
//   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseArray>("target_pose", 10); //initialize chatter
//   ros::Rate loop_rate(10);

//   //角度转四元数
//   float setangle_1 = angle_1_dir;  //贾尼计算得到的角度　假设为45度
// //   cout<<"its in !!!!!!!!!!!!!!!!!"<<setangle<<endl;
//   struct xyzw qz_1 = angletrans(setangle_1);
//   ROS_INFO("the quaternions_1(x,y,z,w) is %lf , %lf, %lf, %lf", qz_1.x, qz_1.y, qz_1.z, qz_1.w);
//   float setangle_2 = angle_2_dir;  //贾尼计算得到的角度　假设为45度
// //   cout<<"its in !!!!!!!!!!!!!!!!!"<<setangle<<endl;
//   struct xyzw qz_2 = angletrans(setangle_2);
//   ROS_INFO("the quaternions_1(x,y,z,w) is %lf , %lf, %lf, %lf", qz_2.x, qz_2.y, qz_2.z, qz_2.w);
  
//   //坐标转换
//   //从云端获取坐标(a1, a2, a3)
//   float a1 = 1.0;
//   float a2 = 2.0;
//   float a3 = 3.0;
//   struct position p_pub = getPosition(a1, a2, a3, temp_x_1, 0.0, temp_z_1);
//   ROS_INFO("the position(x,y,z) is %f , %f, %f", p_pub.x, p_pub.y, p_pub.z);
 

//   while (ros::ok())
//   {
//     //generate pose by ourselves.
//     float positionX, positionY, positionZ;
//     double orientationX_1, orientationY_1, orientationZ_1, orientationW_1, orientationX_2, orientationY_2, orientationZ_2, orientationW_2;
//     //We just make the robot has fixed orientation. Normally quaternion needs to be normalized, which means x^2 + y^2 + z^2 +w^2 = 1
//     double fixedOrientation = 0.1;
//     orientationX_1 = qz_1.x;
//     orientationY_1 = qz_1.y;
//     orientationZ_1 = qz_1.z;
//     orientationW_1 = qz_1.w; 

//     orientationX_2 = qz_2.x;
//     orientationY_2 = qz_2.y;
//     orientationZ_2 = qz_2.z;
//     orientationW_2 = qz_2.w; 

//     //We just make the position x,y,z all the same. The X,Y,Z increase linearly
//     positionX = p_pub.x;
//     positionY = p_pub.y;
//     positionZ = p_pub.z;
 
//     //geometry_msgs::PoseStamped msg;
//     geometry_msgs::PoseArray msg; 

//     geometry_msgs::Pose init_pose;
//     init_pose.position.x = a1;
//     init_pose.position.y = a2;
//     init_pose.position.z = a3;

//     init_pose.orientation.x = orientationX_1;
//     init_pose.orientation.y = orientationY_1;
//     init_pose.orientation.z = orientationZ_1;
//     init_pose.orientation.w = orientationW_1;
//     msg.poses.push_back(init_pose);

//     geometry_msgs::Pose init_pose2;
//     init_pose2.position.x = positionX;
//     init_pose2.position.y = positionY;
//     init_pose2.position.z = positionZ;

//     init_pose2.orientation.x = orientationX_2;
//     init_pose2.orientation.y = orientationY_2;
//     init_pose2.orientation.z = orientationZ_2;
//     init_pose2.orientation.w = orientationW_2;
//     msg.poses.push_back(init_pose2);
 
//     cout<<"first angle"<<angle_1<<endl;
//     cout<<"門前一米处坐标为 ("<<temp_x_1<<","<<0<<","<<temp_z_1<<")"<<endl;
//     cout<<"小车需要行进的距离为"<<juli<<"cm"<<endl;
//     cout<<"second angle"<<angle_2<<endl;
//     ROS_INFO("we publish the robot's position and orientaion!!!1111111111"); 
//     ROS_INFO("the position(x,y,z) is %f , %f, %f", init_pose.position.x, init_pose.position.y, init_pose.position.z);
//     ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z, init_pose.orientation.w);

//     ROS_INFO("we publish the robot's position and orientaion!!!2222222222"); 
//     ROS_INFO("the position(x,y,z) is %f , %f, %f", init_pose2.position.x, init_pose2.position.y, init_pose2.position.z);
//     ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", init_pose2.orientation.x, init_pose2.orientation.y, init_pose2.orientation.z, init_pose2.orientation.w);
//     //ROS_INFO("the time we get the pose is %f",  msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec);
 
//     std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
 
//     chatter_pub.publish(msg);
//     ros::spinOnce();
//     loop_rate.sleep();
 
//   }

        
//     });


// std::thread turtlebot([&]() {
//     std::this_thread::sleep_for (std::chrono::seconds(15));
// //while(true){
//    cout<<"this is turtlebot thread!"<<endl;

//   ros::init(argc, argv, "move_turtle_goforward");//初始化ROS,它允许ROS通过命令行进行名称重映射
//   ros::NodeHandle node;//为这个进程的节点创建一个句柄
  
//   cmdVelPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);//在/mobile_base/commands/velocity topic上发布一个geometry_msgs/Twist的消息
//   ros::Rate loopRate(10);//ros::Rate对象可以允许你指定自循环的频率
//   signal(SIGINT, shutdown);
  
//   ROS_INFO("move_turtle_goforward cpp start...");
//   geometry_msgs::Twist speed; // 控制信号载体 Twist message
//   double time_1 = getTimeNow();
//   double time_2 = getTimeNow();

//   float rad_1 = angle_1_dir * PI / 180.0;
//   float rad_2 = angle_2_dir * PI / 180.0;
//   float dist = juli;    //单位是cm

//   float interval_1 = rad_1 / 0.5;
//   float interval_2 = dist * 0.01 / 0.1 ;
//   float interval_3 = rad_2 / 0.5;

//   cout << "interval_1:" << interval_1 << endl;
//   cout << "interval_2:" << interval_2 << endl;
//   cout << "interval_3:" << interval_3 << endl;


//   int flag_1 = 10;
//   int flag_3 = 10;
//   if(interval_1 < 0){
//       flag_1 = 0;
//       interval_1 = abs(interval_1);
//   }

//   if(interval_3 < 0){
//       flag_3 = 0;
//       interval_3 = abs(interval_3);
//   }
  
//   while (ros::ok() && time_2-time_1 < interval_1)
//   {
//     speed.linear.x = 0; // 设置线速度为0.1m/s，正为前进，负为后退
//     if(flag_1 == 0) speed.angular.z = -0.5; // 设置角速度为0rad/s，正为左转，负为右转
//     else speed.angular.z = 0.5; // 设置角速度为0rad/s，正为左转，负为右转
//     cmdVelPub.publish(speed); // 将刚才设置的指令发送给机器人
//     loopRate.sleep();//休眠直到一个频率周期的时间
//     time_2 = getTimeNow();
//   }
//   //shutdown(1);
//   time_1 = getTimeNow();
//   while (ros::ok() && time_2-time_1 < interval_2)
//   {
//     speed.linear.x = 0.1; // 设置线速度为0.1m/s，正为前进，负为后退
//     speed.angular.z = 0; // 设置角速度为0rad/s，正为左转，负为右转
//     cmdVelPub.publish(speed); // 将刚才设置的指令发送给机器人
//     loopRate.sleep();//休眠直到一个频率周期的时间
//     time_2 = getTimeNow();
//   }
//   //shutdown(1);
//   time_1 = getTimeNow();
//   while (ros::ok() && time_2-time_1 < interval_3)
//   {
//     speed.linear.x = 0; // 设置线速度为0.1m/s，正为前进，负为后退
//     if(flag_3 == 0) speed.angular.z = -0.5; // 设置角速度为0rad/s，正为左转，负为右转
//     else speed.angular.z = 0.5; // 设置角速度为0rad/s，正为左转，负为右转
//     cmdVelPub.publish(speed); // 将刚才设置的指令发送给机器人
//     loopRate.sleep();//休眠直到一个频率周期的时间
//     time_2 = getTimeNow();
//   }
//   shutdown(1);

// //}
//     });



//通信测试
// std::thread turtlebot([&]() {
//     std::cout << "This is ROS test:" << std::endl;
//     ros::init(argc, argv, "subscriber"); 
//     ros::NodeHandle n; 
//     ros::Subscriber sub = n.subscribe("target_pose", 10, chatterCallback); 
//     ros::spin();

//     //cloud_flag = 1;

//     });


// std::thread turtlebot2([&]() {
//     //std::this_thread::sleep_for (std::chrono::seconds(15));
//     if(cloud_flag == 1){
//         while(true){
//             std::cout << "ROS test success!" << std::endl;
//         }
//     }
        

//     });








    while(true){
        for(int i=0;i<1000;i++){
            if(i=999){
    // cout<<"目标点的坐标("<<*z<<","<<*(z+1)<<","<<*(z+2)<<")"<<endl ;
        i=0;
            }
        }
    }

    //video_processing_thread.join();

    return EXIT_SUCCESS;

    // return 0;
} 






