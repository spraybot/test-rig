#ifndef _COMMON
#define _COMMON

#include <cuda_runtime.h>
#include <iostream>
#include <stdio.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/hal/interface.h>
#include <assert.h>
#include <algorithm>

#define WARP_SIZE 32

#define MAX_DISPARITY 160   //160 192 224 256 288 320 352 384 416   all multiples of 32
#define INVALID_DISP_SCALED 0   //-16
#define DISP_SCALE 16
#define DISP_SHIFT 1   //4

using namespace std;

typedef unsigned char PixType;
typedef short CostType;
typedef short DispType;

typedef struct _SGM_PARAMS
{
	int P1;
	int P2;
	int preFilterCap;
	int BlockSize;
	int uniquenessRatio;
	int disp12MaxDiff;
}SGM_PARAMS;

#endif

/*NOTE: good disparity, MAX_DISPARITY vs rescale size. These are values for good disparity

  Rescale factor	Max Disp	Time    Hz
        0.5           192  	50ms    20

        0.75       	  192  	100ms   10

        0.8           256		130ms   7.5	~ 8Hz w/o display
        0.8           352		220ms   4.5

        0.94       	  352   300ms   3.3
*/
