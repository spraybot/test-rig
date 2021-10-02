#include <iostream>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
// Sample includes
#include <time.h>

#include "disparity_gpu/common.h"

using namespace std;

//void free_gpu_mem();
//cv::Mat compute_disparity(cv::Mat *left_img, cv::Mat *right_img, float *cost_time);
//void cuda_init(SGM_PARAMS *params);
//void zy_remap(cv::Mat &img1, cv::Mat &img2);


int main(int argc, char **argv) {

	if(argc != 4){
		std::cout<<"argc wrong\nuseage: ./a.out image_dir start_number end_number";
		return -1;
	}
	char *prefix = argv[1];
	int start_num = atoi(argv[2]);
	int end_num = atoi(argv[3]);
	char left_img_name[128] = {0};
	char right_img_name[128] = {0};

	char key = ' ';
	double frame_start = 0, frame_end = 0;
	bool time_print = false;
	int i = start_num;	

  SGM_PARAMS params;
	params.preFilterCap = 11;
	params.BlockSize = 3;
	params.P1 = 8 * params.BlockSize * params.BlockSize;
	params.P2 = 32 * params.BlockSize * params.BlockSize;
	params.uniquenessRatio = 10;
	params.disp12MaxDiff = 50;
  //cuda_init(&params);

	cv::Mat resizeImg_left, resizeImg_right;

	float img_resize = 0.8;   /// max 0.94 =  94% of max resolution

	double start = 0.0;
	double end = 0.0;
	double duration = 0.0;
    	
	while (key != 27) {
		
		//start = cv::getTickCount();
		sprintf(left_img_name, "%s/left%d.png", prefix, i);
		sprintf(right_img_name, "%s/right%d.png", prefix, i);
    //resizeImg_left = cv::imread(left_img_name, cv::IMREAD_GRAYSCALE);
		
		if(resizeImg_left.empty()){
		    std::cout<<"read "<<left_img_name<<" fail\n";
		    return 1;
		}
		resizeImg_right = cv::imread(right_img_name, cv::IMREAD_GRAYSCALE);
		
		if(resizeImg_right.empty()){
		    std::cout<<"read "<<right_img_name<<" fail\n";
		    return 1;
		}
			
		i++;

		//cv::imshow("left_zed", resizeImg_left);
		//cv::imshow("right_zed", resizeImg_right);
		//cv::waitKey(0);

		cv::resize(resizeImg_left,resizeImg_left,cv::Size(),  img_resize, img_resize);
		cv::resize(resizeImg_right,resizeImg_right,cv::Size(),img_resize, img_resize);
	
		//std::cout<<"image size, left: "<<resizeImg_left.rows<<" x "<<resizeImg_left.cols<<std::endl;
		//std::cout<<"image size, right: "<<resizeImg_right.rows<<" x "<<resizeImg_right.cols<<std::endl;

		//end = cv::getTickCount();
		//printf("Pre Cost:%lf ms.\n", (end - start)* 1000/cv::getTickFrequency());
	
		start = cv::getTickCount();
    //zy_remap(resizeImg_left, resizeImg_right);

		end = cv::getTickCount();

		duration = (end - start)* 1000/cv::getTickFrequency();// in ms

		cout<<"Total time: "<<duration << "ms \t"<<1/(duration/1000)<<" Hz."<<endl;
		cout<<"--------------------------"<<endl;

		//key = cv::waitKey(0);
    }

  //free_gpu_mem();
	return 0;
}


inline void zy_remap(cv::Mat &img1, cv::Mat &img2)
{
  //cv::Mat img = compute_disparity(&img1, &img2, NULL);
	//cv::imshow("disp", img);
	//cv::waitKey(1);
}
