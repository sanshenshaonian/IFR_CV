#include <iostream>
using namespace std;
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include<librealsense2/rs.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;

std::vector<std::vector<cv::Point>> arrows_Contours;
std::vector<cv::Point> sef_Point;
int hill_points[100][2];
double angle;
Mat image_arrow_draw;
Mat frame;
RotatedRect arrow_rect;
Mat mask;
Mat bgr;
Mat hsv;
Point2f pt[4];

cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
cv::Mat element1 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
cv::Mat element3 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1));



int H_min = 0;
int S_min = 0;
int V_min = 0;
int H_max = 255;
int S_max = 255;
int V_max = 60;




////////////////////////////////识别函数////////////////////////////////////


bool colorred(Mat a)        //查找图像内的有无红色像素点,有就返回1，无就返回0
{
	IplImage *image = &IplImage(a);
	CvScalar scalar1;
	vector<int> sca;
	bool flag = false;
	int count = 0;
	int r = 0, g = 0, b = 0;
	//CvScalara scalar1;
	for (int i = 0;i <= image->height - 1;++i)
	{
		for (int j = 0;j <= image->width - 1;++j)
		{
			scalar1 = cvGet2D(image, i, j);
			/*r = scalar1.val[2];
			g = scalar1.val[1];
			b = scalar1.val[0];
			if (r > 150 && g > 120 && b > 100)
			{
			count++;
			}*/
			if (scalar1.val[2] > 150 && scalar1.val[1] > 110 && scalar1.val[0] > 100)
			{
				/*int a = (i*image->width) + j;
				sca.push_back(a);*/
				count++;
				//flag = true;
			}
		}
	}
	if (count>100)
		flag = true;
	return flag;
}

void colorred2(Mat a)           //筛选轮廓中的红色
{
	IplImage *image = &IplImage(a);
	CvScalar scalar2;
	Mat img_color = Mat::zeros(a.size(), CV_8SC3);
	Mat labeles, stats, centroids;
	int nccomps = connectedComponentsWithStats(a, labeles, stats, centroids);
	for (int i = 0;i < nccomps;i++)
	{

	}
	vector<Vec3b>colors(nccomps + 1);
	colors[0] = Vec3b(0, 0, 0);

}

vector<vector<Point>> NEW_contours(vector<vector<Point>> contours)   //把一些比较小的轮廓删掉
{
	vector<vector<Point>> contours1;
	for (int i = 0;i < contours.size();++i)
	{
		if (contours[i].size() > 30)
		{
			contours1.push_back(contours[i]);
		}
	}
	return contours1;
}









struct float3 {
	float x, y, z;
	float3 operator*(float t)
	{
		return{ x * t, y * t, z * t };
	}

	float3 operator-(float t)
	{
		return{ x - t, y - t, z - t };
	}

	void operator*=(float t)
	{
		x = x * t;
		y = y * t;
		z = z * t;
	}

	void operator=(float3 other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
	}

	void add(float t1, float t2, float t3)
	{
		x += t1;
		y += t2;
		z += t3;
	}
};
struct float2 { float x, y; };

struct rect
{
	float x, y;
	float w, h;

	// Create new rect within original boundaries with give aspect ration
	rect adjust_ratio(float2 size) const
	{
		auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
		if (W > w)
		{
			auto scale = w / W;
			W *= scale;
			H *= scale;
		}

		return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
	}
};

void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
	const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

	int width = other_frame.get_width();
	int height = other_frame.get_height();
	int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
	for (int y = 0; y < height; y++)
	{
		auto depth_pixel_index = y * width;
		for (int x = 0; x < width; x++, ++depth_pixel_index)
		{
			// Get the depth value of the current pixel
			auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

			// Check if the depth value is invalid (<=0) or greater than the threashold
			if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
			{
				// Calculate the offset in other frame's buffer to current pixel
				auto offset = depth_pixel_index * other_bpp;

				// Set pixel to "background" color (0x000000) black
				std::memset(&p_other_frame[offset], 0x000000, other_bpp);
			}
		}
	}
}


//获取深度像素对应长度单位转换
float get_depth_scale(rs2::device dev);

//检查摄像头数据管道设置是否改变
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

int main(int argc, char * argv[]) try
{
	

	
	// Create and initialize GUI related objects
	//创建gui窗口
	//window app(1280, 720, "CPP - Align Example"); // Simple window handling
	//ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
	const char* depth_win = "depth_Image";
	namedWindow(depth_win, WINDOW_AUTOSIZE);
	const char* color_win = "color_Image";
	namedWindow(color_win, WINDOW_AUTOSIZE);

	//深度图像颜色map
	rs2::colorizer c;                          // Helper to colorize depth images
											   //helper用于渲染图片
											   //texture renderer;                     // Helper for renderig images

											   // Create a pipeline to easily configure and start the camera
											   //创建数据管道
	rs2::pipeline pipe;
	rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	//Calling pipeline's start() without any additional parameters will start the first device
	//直接start()，不添加配置参数，则默认打开第一个设备
	// with its default streams.
	//以及以默认的配置进行流输出
	//The start function returns the pipeline profile which the pipeline used to start the device
	//start()函数返回数据管道的profile
	rs2::pipeline_profile profile = pipe.start(pipe_config);

	// Each depth camera might have different units for depth pixels, so we get it here
	//每个深度摄像头有不同单元的像素，我们这里获取
	// Using the pipeline's profile, we can retrieve the device that the pipeline uses
	//使用数据管道的profile获取深度图像像素对应于长度单位（米）的转换比例
	float depth_scale = get_depth_scale(profile.get_device());

	//Pipeline could choose a device that does not have a color stream
	//数据管道可以选择一个没有彩色图像数据流的设备
	//If there is no color stream, choose to align depth to another stream
	//选择彩色图像数据流来作为对齐对象
	rs2_stream align_to = RS2_STREAM_COLOR;//find_stream_to_align(profile.get_stream());

										   /*
										   @这里的对齐是改变深度图，而不改变color图
										   */
										   // Create a rs2::align object.
										   //创建一个rs2::align的对象
										   // rs2::align allows us to perform alignment of depth frames to others frames
										   //rs2::align 允许我们去实现深度图像对齐其他图像
										   //The "align_to" is the stream type to which we plan to align depth frames.
										   // "align_to"是我们打算用深度图像对齐的图像流
	rs2::align align(align_to);

	// Define a variable for controlling the distance to clip
	//定义一个变量去转换深度到距离
	float depth_clipping_distance = 1.f;

	//识别变量定义
	Mat frame_0;
	Mat background, adframe;
	Mat gray;
	Mat frame_hsv;
	Mat hsv[3];
	Mat mid, adframe_bw;
	int framenum=1;
	
	//保存视频路径
	cv::VideoWriter video_writer("test(20).avi",cv::VideoWriter::fourcc('x','v','i','d'),30,Size(640,480));
	while (cvGetWindowHandle(depth_win) && cvGetWindowHandle(color_win)) // Application still alive?
	{
		// Using the align object, we block the application until a frameset is available
		//堵塞程序直到新的一帧捕获
		rs2::frameset frameset = pipe.wait_for_frames();

		// rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
		// Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
		//因为rs2::align 正在对齐深度图像到其他图像流，我们要确保对齐的图像流不发生改变
		//  after the call to wait_for_frames();
		if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
		{
			//If the profile was changed, update the align object, and also get the new device's depth scale
			//如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
			profile = pipe.get_active_profile();
			align = rs2::align(align_to);
			depth_scale = get_depth_scale(profile.get_device());
		}

		//Get processed aligned frame
		//获取对齐后的帧
		auto processed = align.process(frameset);

		// Trying to get both other and aligned depth frames
		//尝试获取对齐后的深度图像帧和其他帧
		rs2::frame aligned_color_frame = processed.get_color_frame();//processed.first(align_to);
		rs2::frame aligned_depth_frame = processed.get_depth_frame().apply_filter(c);
		rs2::depth_frame depth = processed.get_depth_frame();

	   //定义接收图像
		rs2::video_frame result_frame = processed.first(align_to);
		//此处为实现渲染关键步骤
		rs2::depth_frame ligned_depth_frame = processed.get_depth_frame();
		remove_background(result_frame, ligned_depth_frame, depth_scale, 3.5);

		//float w = aligned_depth_frame.as<rs2::video_frame>().get_width();
		//float h = aligned_depth_frame.as<rs2::video_frame>().get_height();
		//rect altered_other_frame_rect{ 0, 0, w, h };
		//altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(result_frame.get_width()),static_cast<float>(result_frame.get_height()) });
		//rs2::depth_frame r_d = result_frame;
		
		/*渲染并获取图像结束*/
		
		//获取对齐之前的color图像
		rs2::frame before_depth_frame = frameset.get_depth_frame().apply_filter(c);
		//获取宽高
		const int depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
		const int depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
		const int color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
		const int color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
		const int b_color_w = before_depth_frame.as<rs2::video_frame>().get_width();
		const int b_color_h = before_depth_frame.as<rs2::video_frame>().get_height();

		std::cout << "w is:" << color_w << "h is:" << color_h << std::endl;

		//If one of them is unavailable, continue iteration
		if (!aligned_depth_frame || !aligned_color_frame)
		{
			continue;
		}
		//创建OPENCV类型 并传入数据
		Mat aligned_depth_image(Size(depth_w, depth_h), CV_8UC3, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
		Mat aligned_color_image(Size(color_w, color_h), CV_8UC3, (void*)aligned_color_frame.get_data(), Mat::AUTO_STEP);
		Mat before_color_image(Size(b_color_w, b_color_h), CV_8UC3, (void*)before_depth_frame.get_data(), Mat::AUTO_STEP);
		//Mat ri(Size(depth_w, depth_h), CV_8UC3, (void*)r_d.get_data(), Mat::AUTO_STEP);





		//识别部分






		cvtColor(aligned_color_image, frame_hsv, CV_RGB2GRAY);
		if (framenum == 1)
		{
			background = frame_hsv.clone();
			frame_0 = background;
		}
		else
		{
			background = frame_0;
		}
		absdiff(frame_hsv, background, adframe);
		threshold(adframe, adframe_bw, 80, 255, 0);
		medianBlur(adframe_bw, mid, 5);
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		erode(mid, mid, element);
		imshow("mid", mid);
		
		vector<vector<Point>> contours;
		vector<vector<Point>> contours_red;
		vector<Vec4i> hierarchy;
		findContours(mid, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);   //参数选择
		contours_red = NEW_contours(contours);
		Rect r;
		vector<Rect> boundRect(contours_red.size());
			//if (a==true)
		std::cout <<"redsize"<< contours_red.size() << std::endl;
		std::cout << "Csize" << contours.size() << std::endl;
		/*	for (int i = 0;i < contours_red.size();i++)
			{
				boundRect[i] = boundingRect(contours_red[i]);
				rectangle(aligned_color_image, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2);

			}*/
			contours_red.clear();
			contours.clear();
		frame_0 = frame_hsv.clone();
		framenum++;
		





		//录制视频
		video_writer.write(aligned_color_image);
		//
		
		
		//显示	
		imshow(depth_win, aligned_depth_image);
		imshow(color_win, aligned_color_image);
	  //imshow("before aligned", before_color_image);
		//imshow("121", ri);
		waitKey(1);
	}
	video_writer.release();
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}
