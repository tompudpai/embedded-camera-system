
/*****************************************************************************
 *
 * Freescale Confidential Proprietary
 *
 * Copyright (c) 2016 Freescale Semiconductor;
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


// step 5: add apex libraries
#include <icp_data.h>
using namespace icp;
#include "apex.h"
#include <oal.h>
#include <apexcv_base.h>
#include "apexcv_pro_hough.h"
#include <apexcv_pro_canny.h>
#include <apexcv_pro_util.h>
#include "../../../../libs/apexcv_pro/canny/include/apexcv_canny_ref.h"


#ifndef __STANDALONE__

#include <signal.h>
#endif // #ifdef __STANDALONE__
#include <string.h>

#include "config.h"

// choose graphical output and parameters
#ifdef __STANDALONE__
  #include "frame_output_dcu.h"
  #define CHNL_CNT io::IO_DATA_CH3
#else // #ifdef __STANDALONE__
  #include "frame_output_v234fb.h"
  #define CHNL_CNT io::IO_DATA_CH3
#endif // else from #ifdef __STANDALONE__

#include "sdi.hpp"
#include "mipi_simple_c.h"
// graph properties



#include "vdb_log.h"
#include "isp_gen.h"
#include "isp_user_define.h"


/* Step 2: Include files from matlab codegen main.c*/
extern "C" {
#include "rt_nonfinite.h"
#include "detectLanesOnly.h"
#include "main.h"
#include "detectLanesOnly_terminate.h"
#include "detectLanesOnly_emxAPI.h"
#include "detectLanesOnly_initialize.h"
}

// Step 3: include definitions for profiling
#include <sys/time.h>
#define GETTIME(time)                           \
  {                                             \
  struct timeval lTime; gettimeofday(&lTime,0); \
  *time=(lTime.tv_sec*1000000+lTime.tv_usec);   \
  }

// step 8 morph skel opencv
#include <opencv2/opencv.hpp>

// step 10 video output
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "avcodec.h"
//#include <libavcodec/avcodec.h>
//#include <libavformat/avformat.h>
//#include <libswscale/swscale.h>
//#include <libavutil/avutil.h>

/* Step 2: Function Declarations from matlab codegen main.c */
static emxArray_real32_T *argInit_d720xd1280x3_real32_T(void);
static float argInit_real32_T(void);
static void main_detectLanesOnly(void);

static bool cameraOn = true;

#ifndef __STANDALONE__ 
  int32_t SigintSetup(void); 
#endif

// DCU input resolution (must be supported by the DCU)
// define IPU engine responsible for exposure computations

//***************************************************************************

/************************************************************************/
/** Statically configures frame exposure parameters.
  * 
  ************************************************************************/
#ifndef __STANDALONE__ 
  int32_t SigintSetup(void); 
#endif 
#ifndef __STANDALONE__ 
  int32_t SigintSetup(void); 
static bool sStop = false; ///< to signal Ctrl+c from command line  
 
#endif 

/* Step 2: Function Definitions from matlab codegen main.c */

/*
 * Arguments    : void
 * Return Type  : emxArray_real32_T *
 */
static emxArray_real32_T *argInit_d720xd1280x3_real32_T(unsigned char* buf) //change from void
{
  emxArray_real32_T *result;
  static int iv1[3] = { 720, 1280, 3 }; //edited from { 2, 2, 3 }

  int idx0;
  int idx1;
  int idx2;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreateND_real32_T(3, iv1);

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
      for (idx2 = 0; idx2 < 3; idx2++) {
        /* Set the value of the array element.
           Change this value to the value that the application requires. */
        result->data[(idx0 + result->size[0] * idx1) + result->size[0] *
          result->size[1] * idx2] = ((float)buf[idx2 + idx1*3 + idx0*1280*3]/255.0f);
      }// change from argInit_real32_T()
    }
  }

  return result;
}

/*
 * Arguments    : void
 * Return Type  : float
 */
static float argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_detectLanesOnly(unsigned char* buf) //change from void
{
  emxArray_real32_T *frameOut;
  emxArray_real32_T *frame;
  emxInitArray_real32_T(&frameOut, 3);

  /* Initialize function 'detectLanesOnly' input arguments. */
  /* Initialize function input argument 'frame'. */
  frame = argInit_d720xd1280x3_real32_T(buf);

  /* Call the entry-point 'detectLanesOnly'. */
  detectLanesOnly(frame, frameOut);

  // add initialization code here
  int pixel;
    for (int idx0 = 0; idx0 < frameOut->size[0U]; idx0++) {
      for (int idx1 = 0; idx1 < frameOut->size[1U]; idx1++) {
        for (int idx2 = 0; idx2 < 3; idx2++) {
          /* Set the value of the array element.
             Change this value to the value that the application requires. */
      	  pixel = (unsigned int)(frameOut->data[(idx0 + frameOut->size[0] * idx1) + frameOut->size[0] *
  								 frameOut->size[1] * idx2]*255);
      	  buf[idx2 + (3 * idx1) + (3* 1280 * idx0)] = pixel>255?255:pixel;
        }
      }
    }

  emxDestroyArray_real32_T(frameOut);
  emxDestroyArray_real32_T(frame);
}

// Step 4: neon optimization using NEON intrinsics to convert RGB to gray
void neon_convert (uint8_t * __restrict dest, uint8_t * __restrict src, int n)
{
  int i;
  uint8x8_t rfac = vdup_n_u8 (77);
  uint8x8_t gfac = vdup_n_u8 (151);
  uint8x8_t bfac = vdup_n_u8 (28);
  n/=8;

  for (i=0; i<n; i++)
  {
    uint16x8_t  temp;
    uint8x8x3_t rgb  = vld3_u8 (src);
    uint8x8_t result;

    temp = vmull_u8 (rgb.val[0],      rfac);
    temp = vmlal_u8 (temp,rgb.val[1], gfac);
    temp = vmlal_u8 (temp,rgb.val[2], bfac);

    result = vshrn_n_u16 (temp, 8);
    vst1_u8 (dest, result);

    src  += 8*3;
    dest += 8;
  }

}

void mask(uint8_t * __restrict gray)
{
	int x, y;
	for(x = 0; x < 1280; x++)
		for(y = 0; y < 720; y++){
			bool cond1 = y > 360;
			//bool cond2 = y > 0.5050*x+320; // 521 - 320 = m(398 - 0) (882, 119) -> y = 0.5050 + 320
			bool cond2 = 0;//y > 0.6050*x+180;
			//bool cond3 = y > -.609*x+1026; // 551-247=m(780-1279) -> y = -.609x+1026
			bool cond3 = 0;//y > -.7*x+956;
			if (cond1 || cond2 || cond3)
				gray[x + (1280*y)] = 0;
		}
}

//step 6 otsu
int otsu_method(uint32_t* histogram, int imagesize)
{
int sum = 0;
for (int t=0 ; t<256 ; t++) sum += t * histogram[t];

float sumB = 0;
int wB = 0;
int wF = 0;

float varMax = 0;
int threshold = 0;

for (int t=0 ; t<256 ; t++) {
   wB += histogram[t];               // Weight Background
   if (wB == 0) continue;

   wF = imagesize - wB;                 // Weight Foreground
   if (wF == 0) break;

   sumB += (float) (t * histogram[t]);

   float mB = sumB / wB;            // Mean Background
   float mF = (sum - sumB) / wF;    // Mean Foreground

   // Calculate Between Class Variance
   float varBetween = (float)wB * (float)wF * (mB - mF) * (mB - mF);

   // Check if new maximum found
   if (varBetween > varMax) {
      varMax = varBetween;
      threshold = t;
   }
}

return threshold;
}

// step 8

/*void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}*/

void bwmorph(cv::Mat img)
{

	void *bufferSkel = OAL_MemoryAllocFlag(1280*720, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS | OAL_MEMORY_FLAG_ZERO);
	memset(bufferSkel, 0, 1280*720);
	cv::Mat skel(img.size(), CV_8UC1, bufferSkel);

	void *bufferTemp = OAL_MemoryAllocFlag(1280*720, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS | OAL_MEMORY_FLAG_ZERO);
	memset(bufferTemp, 0, 1280*720);
	cv::Mat temp(img.size(), CV_8UC1);
	//cv::Mat eroded;

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	bool done;
	do
	{
		cv::morphologyEx(img, temp, cv::MORPH_OPEN, element);
		cv::bitwise_not(temp, temp);
		cv::bitwise_and(img, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		cv::erode(img, img, element);

	    double max;
	    cv::minMaxLoc(img, 0, &max);
	    done = (max == 0);

	    /*cv::erode(img, eroded, element);
	    cv::dilate(eroded, temp, element); // temp = open(img)
	    cv::subtract(img, temp, temp);
	    cv::bitwise_or(skel, temp, skel);
	    eroded.copyTo(img);

	    done = (cv::countNonZero(img) == 0);*/

	} while (!done);
	skel.copyTo(img);

	OAL_MemoryFree(bufferSkel);
	OAL_MemoryFree(bufferTemp);
	//img.data = (skel.data);
	//img.data = skel.data; //(skel.data);
}


//step 7 hough transform support functions to draw lines
void setPixel(int x, int y, unsigned char *buffer)
{

	// line thickness of 5 pixels
if(x<1275 && x>=5 && y<720 && y>=0)
{
buffer[0 + (3*x) + (3*1280*y)] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y)] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y)] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) + 3] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) + 3] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) + 3] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) - 3] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) - 3] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) - 3] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) + 6] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) + 6] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) + 6] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) - 6] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) - 6] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) - 6] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) + 9] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) + 9] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) + 9] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) - 9] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) - 9] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) - 9] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) + 12] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) + 12] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) + 12] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) - 12] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) - 12] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) - 12] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) + 15] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) + 15] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) + 15] = 0x00;//R
buffer[0 + (3*x) + (3*1280*y) - 15] = 0xff;//B
buffer[1 + (3*x) + (3*1280*y) - 15] = 0x00;//G
buffer[2 + (3*x) + (3*1280*y) - 15] = 0x00;//R
}
}

/*void line(int x0, int y0, int x1, int y1, unsigned char *buffer) {

  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    setPixel(x0,y0, buffer);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}*/


//void line(int x0, int y0, int x1, int y1, unsigned char *buffer) {
void line(int* pts, int idx, unsigned char *buffer) {

  int x0 = pts[4*idx + 0];
  int y0 = pts[4*idx + 1];
  int x1 = pts[4*idx + 2];
  int y1 = pts[4*idx + 3];
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    setPixel(x0,y0, buffer);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }

}

//void line_polar(float rho, float theta, int lenght, int* pt, int idx)
void line_polar(float rho, float theta, int lenght, int *pt, int idx)
{ // example: theta = 145, rho = 295, accum = 20

	if(idx < 3)
		 printf("step 0: %d: rho: %.2f theta:%.2f\n", idx, rho, theta);

	double a = cos(((double)theta)), b = sin(((double)theta)); //a = 0.334, b = .4677
	double x = a*rho, y = b*rho; // x = 260.740, y = 137.985

	//if(idx > 15)
			 //printf("step 0: %d: a:%.2f b:%.2f x:%.2f y:%f\n", idx, a, b, x, y);

	double x0 = 640 +(x + lenght*(-b)); //x0 = 640+260.740-.4677*length = 891.386
	double y0 = 360 +(y + lenght*(a)); //y0 = 360+137.985-.334*length = 491.305
	double x1 = 640 +(x - lenght*(-b)); //x1 = 640+(260.740+.4677*length) = 910.094
	double y1 = 360 +(y - lenght*(a)); //y1 = 360+137.985+.334*length = 504.665

	if(idx < 3)
		printf("step 1: %d: (%.2f, %.2f) to (%.2f, %.2f)\n", idx, x0, y0, x1, y1);

	//int refLine1[4] = {600,600, 100, 100};
	//int refLine2[4] = {680, 600, 1180, 100};

	// rearrange points so (x1,y1) is higher than (x0, y0)
	double temp1, temp2;
	if(y0 > y1){
		temp1 = x0;
		temp2 = y0;
		x0 = x1;
		y0 = y1;
		x1 = temp1;
		y1 = temp2;
	}

	// truncate lines that are past center of frame y1 = m(640-x0) + y0
	double slope = (y1-y0)/(x1-x0);
	if (slope > 0 && x1 > 600 && x0 < 600){
		x1 = 600;
		y1 = slope*(600-x0) + y0;
	}
	if (slope < 0 && x1 < 680 && x0 > 680){
		x1 = 680;
		y1 = slope*(680-x0) + y0;
	}

	// truncate lines that are too high and extend lines that are too short//(540-y0)/m+x0=x1
	// essentially: set y1 to 360
	x1 = (360-y0)/slope + x0;
	y1 = 360;


	//extend line to bottom of frame

	// working line extension -(y1 - 0)/m + x1 = x0
	double xBottom = x1 - y1/slope; // y1-y0=m(x1-xbottom)
	if (xBottom < 0)
	{
		y0 = y1 - slope * x1;
		x0 = 0;
	}
	else if (xBottom > 1279){
		y0 = y1 - slope*(x1 - 1279);
		x0 = 1279;
	}
	else
	{
		y0 = 0;
		x0 = xBottom;
	}
	/*int m = (x1-x0)/(y1-y0);
	if(y1 > y0){ // point 1 is higher than point 0
		x0 = x0 - m*(y0);
		y0 = 0;
	}
	else{ // point 0 is higher than point 1
		x1 = x1 - m*(y1);
		y1 = 0;
	}*/

	/*if(y1 > y0){ // point 1 is higher than point 0 (y1 - y0) = m (x1 - x0) -> y1 - 0 = (y1-y0)/(x1-x0)*(x1-X) ->y1/m = x1-X -> X=x1-y1/m
			float m = (y1-y0)/(x1-x0);
			x0 = x1 - m*(y1);
			y0 = 0;
		}
		else{ // point 0 is higher than point 1 (y1 - y0) = m (x1 - x0) -> 0 - y0 = m*(X - x0) -> -y0/m+x0 = X
			float m = (y1-y0)/(x1-x0);
			x1 = x0 - m*(y0);
			y1 = 0;
		}*/

	/*pt[idx + 0] = x0;
		pt[idx + 1] = y0;
		pt[idx + 2] = x1;
		pt[idx + 3] = y1;*/

	//line(x0,y0,x1,y1,buffer);
	pt[4*idx + 0] = (int)x0;
	pt[4*idx + 1] = (int)y0;
	pt[4*idx + 2] = (int)x1;
	pt[4*idx + 3] = (int)y1;
}
/*

void line_polar(float rho, float theta, int lenght, int* pt, int idx)
{ // example: theta = 145, rho = 295, accum = 20
	double a = cos(((double)theta)), b = sin(((double)theta)); //a = 0.334, b = .4677
	double x = a*rho, y = b*rho; // x = 260.740, y = 137.985
	int x0 = 640 +(x + lenght*(-b)); //x0 = 640+260.740-.4677*length = 891.386
	int y0 = 360 +(y + lenght*(a)); //y0 = 360+137.985-.334*length = 491.305
	int x1 = 640 +(x - lenght*(-b)); //x1 = 640+(260.740+.4677*length) = 910.094
	int y1 = 360 +(y - lenght*(a)); //y1 = 360+137.985+.334*length = 504.665

	//extend line to bottom of frame
	int m = (x1-x0)/(y1-y0);
	if(y1 > y0){ // point 1 is higher than point 0
		x0 = x0 - m*(y0);
		y0 = 0;
	}
	else{ // point 0 is higher than point 1
		x1 = x1 - m*(y1);
		y1 = 0;
	}

	pt[idx + 0] = x0;
	pt[idx + 1] = y0;
	pt[idx + 2] = x1;
	pt[idx + 3] = y1;

	//line(x0,y0,x1,y1,buffer);
}
*/
// step 9
void findCurrentLane(float* rep_ref, int* count_ref, int maxLaneNum,
		int frameFound, int numLines, int* cartPt, int* leftPts, int* rightPts){

	// find 2 lines with top points closest to center
	//line_polar(rho, theta, accum, buffer_out);
	int dis_inf = 1280; // width of image
	int halfWidth = dis_inf / 2;
	uint16_t leftDis = 0xFFFF;
	uint16_t rightDis = 0xFFFF;
	uint16_t leftHeight = 720;
	uint16_t rightHeight = 720;

	for(int i = 0; i < maxLaneNum; i++){

		uint16_t colNum, centerDis, rowNum;

		// find higher point
		colNum = cartPt[4*i+2]; // colNum = x1
		rowNum = cartPt[4*i+3]; // rowNum = y1
		/*if(cartPt[4*i+1] >= cartPt[4*i+3]){ // if y0 < y1 // if y0 > y1
			colNum = cartPt[4*i]; // colNum = x0
			rowNum = cartPt[4*i+1];
		}
		else{
			colNum = cartPt[4*i+2]; // else colNum = x1
			rowNum = cartPt[4*i+3];
		}*/

		// find horizontal distance from center to top of line
		if(count_ref[i] >= frameFound){
			centerDis = abs(halfWidth - colNum);
		}
		else
			centerDis = dis_inf;

		//find slope m = (y1-y0)/(x1-x0)
		double slope = (double)(cartPt[4*i+3] - cartPt[4*i+1])/(double)(cartPt[4*i+2] - cartPt[4*i]);

		//printf("i = %d: halfwidth - col = %d, slope: %.2f\n", i, halfWidth-colNum);
		if((halfWidth - colNum) >= 0){ // if leftLane
			bool cond1 = centerDis < leftDis;
			bool cond2 = centerDis == leftDis && rowNum < leftHeight;
			bool cond3 = slope > 0.4;
			if ((cond1 || cond2) && cond3){
				leftDis = centerDis;
				leftHeight = rowNum;
				leftPts[0] = cartPt[4*i + 0];
				leftPts[1] = cartPt[4*i + 1];
				leftPts[2] = cartPt[4*i + 2];
				leftPts[3] = cartPt[4*i + 3];
			}
		}
		else{ // if right lane
			bool cond1 = centerDis < rightDis;
			bool cond2 = centerDis == rightDis && rowNum < rightHeight;
			bool cond3 = slope < -0.4;
			if ((cond1 || cond2) && cond3){
				rightDis = centerDis;
				rightHeight = rowNum;
				rightPts[0] = cartPt[4*i + 0];
				rightPts[1] = cartPt[4*i + 1];
				rightPts[2] = cartPt[4*i + 2];
				rightPts[3] = cartPt[4*i + 3];
			}
		}
	}

	//printf("left dis: %d, right dis: %d\n", leftDis, rightDis);
	// line points = 0 if invalid
	if (leftDis >= dis_inf){
		leftPts[0] = 0;
		leftPts[1] = 0;
		leftPts[2] = 0;
		leftPts[3] = 0;
	}
	if (rightDis >= dis_inf){
		rightPts[0] = 0;
		rightPts[1] = 0;
		rightPts[2] = 0;
		rightPts[3] = 0;
	}

	printf("leftPts: (%d, %d) to (%d, %d)\n", leftPts[0], leftPts[1], leftPts[2], leftPts[3]);
	printf("rightPts: (%d, %d) to (%d, %d)\n", rightPts[0], rightPts[1], rightPts[2], rightPts[3]);
}

void lane_matching(apexcv::HoughLineDetector hough, float* rep_ref, int* count_ref,
		int maxLaneNum, uint32_t *pLine, int trackThreshold, int frameFound, int frameLost,
		int numLines){

	//uint16_t list = OAL_MemoryAllocFlag(maxLaneNum * numLines, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
	// generate 30 x numLines matrix of max values
	/*uint16_t * list = (uint16_t*)malloc(maxLaneNum * numLines * sizeof(uint16_t));

	// calculate distances between lines found in current frame and in repository
	for(int i = 0; i < maxLaneNum; i++){
		for(int j = 0; j < numLines; j++){

			list[i * numLines + j] = 0xFFFF;

			int rho = hough.line(pLine[j]).rho;
			float theta = hough.line(pLine[j]).theta;
			//int accum = apexcv::HoughLineDetector::accumulator(pLine[j]);

			if(count_ref[i] > 0){
				list[i * numLines + j] = abs(rho - (int)rep_ref[i]) + 100*abs((int)theta - (int)rep_ref[maxLaneNum + i]);
			}

		}
	}

	// find best matches between current lines and those in the repository
	uint16_t* match_dist = (uint16_t*)malloc(numLines * sizeof(uint16_t));
	uint16_t* match_list = (uint16_t*)malloc(2 * numLines * sizeof(uint16_t));
	match_dist[0] = 0xFFFF; match_dist[1] = 0xFFFF;
	match_list[0] = 0; match_list[1] = 0; match_list[2] = 0; match_list[3] = 0;

	uint16_t minValue;
	uint16_t minRowIdx, minColIdx;
	// find minimum value and position in list matrix
	for(int k = 0; k < numLines; k++){
		for(int i = 0; i < maxLaneNum; i++){
			for(int j = 0; j < numLines; j++)
			{
				minValue = 0xFFFF;
				// reset row and col in list matrix of previously found minimum
				if (k > 0){
					for(int x = 0; x < numLines; x++)
						list[minRowIdx * numLines + x] = 0xFFFF;
					for(int y = 0; y < maxLaneNum; y++)
						list[y * numLines + minColIdx] = 0xFFFF;
				}
				else if (list[i * numLines + j] < minValue){
					minRowIdx = i;
					minColIdx = j;
					minValue = list[i * numLines + j];
				}
			}

		}
		match_dist[k] = minValue;
		match_list[k] = minRowIdx; // index of repository line
		match_list[k + numLines] = minColIdx; // index of currently detected line
	}
	//match_list[]

	// update repository values
	for(int i = 0; i < maxLaneNum; i++){
		count_ref[i]--; // reduce all counts by 1
	}
	for(int j = 0; j < numLines; j++){

		int newIdx = -1;
		// if matched line is different enough, insert in unused space of repository
		if(match_dist[j] > trackThreshold){
			// find index of first count_ref element with negative count
			for(int k = 0; k < maxLaneNum; k++){
				if(count_ref[k] < 0 && newIdx < 0)
					newIdx = k;
			}
			rep_ref[newIdx] = hough.line(pLine[match_list[j+numLines]]).rho;
			rep_ref[newIdx + maxLaneNum] = hough.line(pLine[match_list[j+numLines]]).theta;
			rep_ref[newIdx + 2*maxLaneNum] = apexcv::HoughLineDetector::accumulator(pLine[match_list[j+numLines]]);
			count_ref[newIdx] += 2;
		}
		else // if matched line is close enough, update repository lines with new values
		{
			rep_ref[match_list[j]] = hough.line(pLine[match_list[j+numLines]]).rho;
			rep_ref[match_list[j] + maxLaneNum] = hough.line(pLine[match_list[j+numLines]]).theta;
			rep_ref[match_list[j] + 2*maxLaneNum] = apexcv::HoughLineDetector::accumulator(pLine[match_list[j+numLines]]);
			count_ref[match_list[j]] += 2;
		}
	}

	uint16_t countUpperThresh = frameFound + frameLost;
	for(int k = 0; k < maxLaneNum; k++){
		if(count_ref[k] < 0)
			count_ref[k] = 0; // reset negative counts to 0
		if(count_ref[k] > countUpperThresh)
			count_ref[k] = countUpperThresh; // set high counts to upper threshold
	}

	// free allocated memory
	free(match_list);
	free(match_dist);
	free(list);*/

	//OAL_MemoryFree((void*)list);
}

void ISP_CALL()
{
  //*** Init DCU Output ***
	//step 5 initialize
	ACF_Init();
#ifdef __STANDALONE__

	  io::FrameOutputDCU  lDcuOutput(WIDTH_DDR,
	               					HEIGHT_DDR,
	               					io::IO_DATA_DEPTH_08,
	               					CHNL_CNT
#ifdef __DCU_BPP				   
	               					,DCU_BPP
#endif
	               					); 
#else // #ifdef __STANDALONE__

  // setup Ctrl+C handler for Linux
  if(SigintSetup() != SEQ_LIB_SUCCESS)
  {
    VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
    return ;
  }

printf("Press Ctrl+C to terminate the demo.\n");

	io::FrameOutputV234Fb lDcuOutput(WIDTH_DDR,
	               					HEIGHT_DDR,
	               					io::IO_DATA_DEPTH_08,
	               					CHNL_CNT
#ifdef __DCU_BPP				   
	               					,DCU_BPP
#endif
	               					); 
				        
#endif // else from #ifdef __STANDALONE__


  //
  // *** Initialize SDI ***
  //
  sdi::Initialize(0);
  // *** create grabber ***
  sdi_grabber *lpGrabber = new(sdi_grabber);

  // fetched frame buffer storage
    SDI_Frame gFrameIsp;

  //if (cameraOn){
  lpGrabber->ProcessSet(gpGraph, &gGraphMetadata);

   // *** prepare IOs ***

  io_config(lpGrabber);

  // *** prestart grabber ***
  lpGrabber->PreStart();


  // *** configure camera parameters ***
  camera_config();

  //// fetched frame buffer storage
  //SDI_Frame gFrameIsp;


  lpGrabber->Start();
//}


  uint32_t lLoop;

  // Step 2: from matlab codegen main.c
  /* Initialize the application.
       You do not need to do this more than one time. */
  detectLanesOnly_initialize();

  // step 5 filter apex function
  apexcv::convolveFilter f;
  apexcv::Canny c;
  // step 6 histogram + th apex functions
  apexcv::histogram hg;
  apexcv::threshold a;
  apexcv::HoughLineDetector hough;

  // step 5 filter data structures
  void *lp_buffer = OAL_MemoryAllocFlag(1280*720,
  		  OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);

  DataDescriptor srcFilter1;
  srcFilter1.InitManual(1280, 720, lp_buffer, OAL_MemoryReturnAddress(lp_buffer, ACCESS_PHY), DATATYPE_08U);
  DataDescriptor dstFilter1(1280, 720, DATATYPE_08U);
  signed char *test_filter_3x3_oal = (signed char *)OAL_MemoryAllocFlag(9, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)|OAL_MEMORY_FLAG_CONTIGUOUS);
  test_filter_3x3_oal[0] = 0; test_filter_3x3_oal[1] = 0; test_filter_3x3_oal[2] = 0;
  test_filter_3x3_oal[3] = -1; test_filter_3x3_oal[4] = 0; test_filter_3x3_oal[5] = 1;
  test_filter_3x3_oal[6] = 0; test_filter_3x3_oal[7] = 0; test_filter_3x3_oal[8] = 0;

  // step 6 histogram data structures
    DataDescriptor hist_in;
    hist_in.InitManual(1280, 720, (void *)dstFilter1.GetDataPtr(), (void *)OAL_MemoryReturnAddress((void *)dstFilter1.GetDataPtr(), ACCESS_PHY), DATATYPE_08U);
    //void *hist_out_ptr = OAL_MemoryAllocFlag(256*4, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
    DataDescriptor hist_out(256, 1, DATATYPE_32U);
    memset(hist_out.GetDataPtr(), 0, 256 * 4);

    //threshold data structures
     DataDescriptor thres_in;
     thres_in.InitManual(1280, 720, (void *)dstFilter1.GetDataPtr(), (void *)OAL_MemoryReturnAddress((void *)dstFilter1.GetDataPtr(), ACCESS_PHY), DATATYPE_08U);
     DataDescriptor thres_out(1280, 720, DATATYPE_08U);

     //step 8 morph skel data structures
     //emxArray_boolean_T *BM;

     // step 7 hough line detector structures
     DataDescriptor srcHough;
     srcHough.InitManual(1280, 720, (void *)thres_out.GetDataPtr(), (void *)OAL_MemoryReturnAddress((void *)thres_out.GetDataPtr(), ACCESS_PHY), DATATYPE_08U);
     //srcHough.InitManual(1280, 720, (void *)dstFilter1.GetDataPtr(), (void *)OAL_MemoryReturnAddress((void *)dstFilter1.GetDataPtr(), ACCESS_PHY), DATATYPE_08U);

  // step 5 filter initialize
  f.initialize(srcFilter1, dstFilter1, (signed char* )test_filter_3x3_oal);
     //c.initialize(srcFilter1, dstFilter1, 1280, 720, 30, 100, 0);
  hg.initialize(hist_in, hist_out);

  // step 8 morph skel initialize
  //emxInit_boolean_T(&BM, 2);

  // step 7 hough initialize
  hough.initialize(srcHough);
    //hough.setTheta(180);
  hough.setTheta(156, 13*apexcv::HoughLineDetector::deg2rad, 1*apexcv::HoughLineDetector::deg2rad);
    hough.setAccumThreshold(100);

    // step 9 repository initialize
    int maxLaneNum = 23;
    float* rep_ref = (float *)malloc(3 * maxLaneNum * sizeof(float));
    int* count_ref = (int *)malloc(maxLaneNum * sizeof(int));
    int trackThreshold = 14;//75;
    int frameFound = 2;//5;
    int frameLost = 8;//20;
    //int numNormalDriving = 0;
    int filterThresh = 13;

    // initialize first two lines
    rep_ref[0] = 275;
        rep_ref[maxLaneNum] = (180-45)*apexcv::HoughLineDetector::deg2rad;
        rep_ref[2*maxLaneNum] = 20;
        rep_ref[1] = 275;
    	rep_ref[maxLaneNum+1] = 45*apexcv::HoughLineDetector::deg2rad;
    	rep_ref[2*maxLaneNum+1] = 20;
    	count_ref[0] = 10;
    	count_ref[1] = 10;

    // initialize repositories to value 0
    for(int z = 0; z < maxLaneNum; z++){
    	rep_ref[z] = 0;
    	rep_ref[z + maxLaneNum] = 0;
    	rep_ref[z + 2*maxLaneNum] = 0;
    	count_ref[z] = 0;
    }
    //float thetaDiff = 100*(theta - rep_ref[maxLaneNum + i]);
    //list[i * max + j] = abs(rho - (int)rep_ref[i]) - (int)(thetaDiff);

    int* cartPt = (int*)malloc(4 * maxLaneNum * sizeof(int));
	int* leftPts = (int*)malloc(4 * sizeof(int));
	int* rightPts = (int*)malloc(4 * sizeof(int));

	// generate binary mask
	//cv::Point a = (0,0);

	// Video Reader initialization
	cv::VideoCapture cap('./mvi_0050.avi');
	//if(!cap.isOpened())  // check if we succeeded

	//cap.open('mvi_0050.avi');


	// Video Writer initialization
	//cv::VideoWriter vw = cv::VideoWriter("/root/output.mp4", CV_FOURCC('M','P','4','2'), 30.0, cv::Size(1280,720));
	cv::VideoWriter vw;
	//int codec = CV_FOURCC('D','I','V','X');
	//int codec = CV_FOURCC('M','P','4','2');
	int codec = CV_FOURCC('X','V','I','D');
	double fps = 5.0;
	vw.open( "./CSULA_ADAS_S32_Output.avi", codec, fps, cv::Size(1280,720), true);


  while (1)
  {

    for(lLoop=0; lLoop<LOOP_NUM; lLoop++)
    {
    	//if (cameraOn){
		  gFrameIsp = lpGrabber->FramePop();
		  if(gFrameIsp.mImage.mData == NULL)
		  {
			break;
		  } // if pop failed
    	//}
		  {

    	  //if(cameraOn){
    	  cv::Mat frame = cv::Mat(720, 1280, CV_8UC3, (unsigned char *)gFrameIsp.mImage.mData);

    	  //}
		 /*if(!cameraOn)
		  {
			 // grab input data from stored image for testing
			 // void *lp_buffer0 = OAL_MemoryAllocFlag(640*480*3, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
			// memset(lp_buffer0, 0, 1280*720*3);

			 //void *lp_buffer1 = OAL_MemoryAllocFlag(640*480*3, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
			 //memset(lp_buffer1, 0, 640*480*3);

		  }*/
			 //frame.release();
		 	 // if camera on
			 /*cv::Mat in = cv::imread("/root/roadImage2.png", CV_LOAD_IMAGE_COLOR);
			 cv::Mat frame = cv::Mat(720, 1280, CV_8UC3, lp_buffer0);
			 in.copyTo(frame);*/

			 //cv::Mat zeros = cv::Mat::zeros(720,1280)
			 //cv::Mat clip = cv::Mat(640, 480, CV_8UC3, lp_buffer1);  //Create Black Image Matrix for image (y,x)
			 //cv::Mat clip = cv::Mat::ones(480, 640, CV_8UC3);
			 //clip = cv::Scalar(255,255,255);

			 //cv::Mat clip;
			 //cap.read(clip);
			 //clip = cap.read();
			 //cap >> clip;
			 //clip << cap;
			 //clip.copyTo(frame(cv::Rect(10,10,clip.cols, clip.rows))); //(x,y) top left orgin

			 //cv::flip(frame, frame, -1);

			 //DataDescriptor lInput0(in.cols, in.rows, DATATYPE_08U);
			 //memcpy(lInput0.GetDataPtr(), in.data, in.cols * in.rows);


			 //cv::Mat out(720, 1280, CV_8UC3);
			 //in.copyTo(out(cv::Rect(0, 232, 256, 256)));

    	// Step 3: Profiling
    	unsigned long start, end;
    	GETTIME(&start);
    	// Step 2: from matlab codegen main.c
		/*main_detectLanesOnly((unsigned char*)(uintptr_t)gFrameIsp.mImage.mData);
		GETTIME(&end);
		printf("main_detectLanesOnly elapsed:%5.3fms \n",(end-start)/1000.0f);*/
    	// Step 4: neon optimization
		GETTIME(&start);
    	//neon_convert ((uint8_t *)lp_buffer,(uint8_t *)(uintptr_t)gFrameIsp.mImage.mData, 1280*720);
		neon_convert ((uint8_t *)lp_buffer,(uint8_t *)(uintptr_t)frame.data, 1280*720);
		//neon_convert ((uint8_t *)lp_buffer,(uint8_t *)(uintptr_t)lInput0.GetDataPtr(), 1280*720);
		GETTIME(&end);
		printf("neon_convert elapsed:%5.3fms \n",(end-start)/1000.0f);

		// step 10  mask
		mask ((uint8_t *)lp_buffer);

		// step 5 filter process
		GETTIME(&start);
		f.process();
		//c.process();
		GETTIME(&end);
		printf("filter elapsed:%5.3fms \n",(end-start)/1000.0f);

		GETTIME(&start);
		hg.process();
		uint32_t *pDst = (uint32_t*) hist_out.GetDataPtr();
		int th = otsu_method(pDst, 1280*720);
		printf("threshold = %d\n",th);
		th = filterThresh;
		 //th = 75;

		a.initialize(thres_in, thres_out, (unsigned int)th);
		a.process();
		GETTIME(&end);
		printf("xhistogram+otsu+th elapsed:%5.3fms \n",(end-start)/1000.0f);

		// step 8 morphological skeleton
		GETTIME(&start);
		/*(void *)thres_out.GetDataPtr();
	    if (!((BM->size[0] == 0) || (BM->size[1] == 0))) {
		  algbwmorph(BM);
	    }*/
		//bwmorph(thres_out.GetDataPtr());
		void *buffer_thres = OAL_MemoryAllocFlag(1280*720, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
		memset(buffer_thres, 0, 1280*720);
		cv::Mat morph = cv::Mat(720, 1280, CV_8UC1, buffer_thres);

		cv::Mat thres = cv::Mat(720, 1280, CV_8UC1, (unsigned char *)thres_out.GetDataPtr());
		thres.copyTo(morph);
		//bwmorph(morph);
	    GETTIME(&end);
	    printf("morph skel elapsed:%5.3fms \n",(end-start)/1000.0f);

		 // step 7 Hough transform
		GETTIME(&start);
		hough.process();
		GETTIME(&end);
		printf("hough elapsed:%5.3fms \n",(end-start)/1000.0f);
		printf("lines detected:%d\n",hough.lineCount());

		// origin is bottom right corner
		// x drawn from right to left
		// y drawn bottom to top
		/*int refLine1[4] = {600,600, 100, 100};
		int refLine2[4] = {680, 600, 1180, 100};
		line(refLine1, 0, frame.data);
		line(refLine2, 0, frame.data);*/

		 int max = hough.lineCount();
			if(max> maxLaneNum) max = maxLaneNum; //change from 30

		GETTIME(&start);

		// step 9
		uint32_t *pLine = hough.packedLineData();
		//unsigned char *buffer_out = (unsigned char *)gFrameIsp.mImage.mData;

		// match lines in current frame to lines in repository

		//uint16_t list = OAL_MemoryAllocFlag(maxLaneNum * numLines, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
			// generate 30 x numLines matrix of max values
			uint16_t * list = (uint16_t*)malloc(maxLaneNum * max * sizeof(uint16_t));
			uint16_t countUpperThresh = frameFound + frameLost;

			/*if(count_ref[0] == 5){
				rep_ref[0] = 275;
				rep_ref[maxLaneNum] = (180-45)*apexcv::HoughLineDetector::deg2rad;
				rep_ref[2*maxLaneNum] = 20;
				count_ref[0] = countUpperThresh;
			}
			if(count_ref[1] == 5){
				rep_ref[1] = 275;
				rep_ref[maxLaneNum+1] = 45*apexcv::HoughLineDetector::deg2rad;
				rep_ref[2*maxLaneNum+1] = 20;
				count_ref[1] = countUpperThresh;
			}*/


			// calculate distances between lines found in current frame and in repository
			for(int i = 0; i < maxLaneNum; i++){
				for(int j = 0; j < max; j++){

					list[i * max + j] = 0xFFFF;

					int rho = hough.line(pLine[j]).rho;
					float theta = hough.line(pLine[j]).theta;
					//int accum = apexcv::HoughLineDetector::accumulator(pLine[j]);

					if(count_ref[i] > 0){
						float thetaDiff = 100*(theta - rep_ref[maxLaneNum + i]);
						if (thetaDiff < 0)
							list[i * max + j] = abs(rho - (int)rep_ref[i]) - (int)(thetaDiff);
						else
							list[i * max + j] = abs(rho - (int)rep_ref[i]) + (int)(thetaDiff);
					}

				}
			}

			// find best matches between current lines and those in the repository

			//initialize values
			uint16_t* match_dist = (uint16_t*)malloc(max * sizeof(uint16_t));
			uint16_t* match_list = (uint16_t*)malloc(2 * max * sizeof(uint16_t));
			//match_dist[0] = 0xFFFF; match_dist[1] = 0xFFFF;
			for(int i = 0; i < max; i++){
				match_dist[i] = 0xFFFF;
				match_list[i] = 0;
				match_list[i+max] = 0;
			}
			//match_list[0] = 0; match_list[1] = 0; match_list[2] = 0; match_list[3] = 0;

			uint16_t minValue;
			uint16_t minRepIdx, minLineIdx;
			// find minimum value and position in list matrix
			for(int k = 0; k < max; k++){
				minValue = 0xFFFF;
				for(int i = 0; i < maxLaneNum; i++){

					// reset row and col in list matrix of previously found minimum
					if (k > 0){
						for(int x = 0; x < max; x++)
							list[minRepIdx * max + x] = 0xFFFF;
						for(int y = 0; y < maxLaneNum; y++)
							list[y * max + minLineIdx] = 0xFFFF;
					}
					if (list[i * max + k] < minValue){
						minRepIdx = i;
						minLineIdx = k;
						minValue = list[i * max + k];
					}

				}
				match_dist[k] = minValue;
				match_list[k] = minRepIdx; // index of repository line
				match_list[k + max] = minLineIdx; // index of currently detected line
				//printf("k: %d,minValue: %d, repIdx: %d, currentLineIdx: %d\n", k, minValue, minRepIdx, minLineIdx);
			}
			//match_list[]

			// update repository values
			for(int z = 0; z < maxLaneNum; z++){
				count_ref[z]--; // reduce all counts by 1
			}
			for(int j = 0; j < max; j++){

				int newIdx = -1;
				// if matched line is different enough, insert in unused space of repository
				if(match_dist[j] > trackThreshold){
					// find index of first count_ref element with negative count
					for(int k = 0; k < maxLaneNum; k++){
						if(count_ref[k] < 0 && newIdx < 0)
							newIdx = k;
					}
					rep_ref[newIdx] = hough.line(pLine[match_list[j+max]]).rho;
					rep_ref[newIdx + maxLaneNum] = hough.line(pLine[match_list[j+max]]).theta;
					rep_ref[newIdx + 2*maxLaneNum] = apexcv::HoughLineDetector::accumulator(pLine[match_list[j+max]]);
					count_ref[newIdx] += 2;
				}
				else // if matched line is close enough, update repository lines with new values
				{
					rep_ref[match_list[j]] = hough.line(pLine[match_list[j+max]]).rho;
					rep_ref[match_list[j] + maxLaneNum] = hough.line(pLine[match_list[j+max]]).theta;
					rep_ref[match_list[j] + 2*maxLaneNum] = apexcv::HoughLineDetector::accumulator(pLine[match_list[j+max]]);
					count_ref[match_list[j]] += 2;
				}

			}

			for(int k = 0; k < maxLaneNum; k++){
				if(count_ref[k] < 0)
					count_ref[k] = 0; // reset negative counts to 0
				if(count_ref[k] > countUpperThresh)
					count_ref[k] = countUpperThresh; // set high counts to upper threshold
				//printf("count(%d) = %d \n", k, count_ref[k]);
			}

			// free allocated memory
			free(match_list);
			free(match_dist);
			free(list);

			float totalRhoLeft = 0;
			float totalThetaLeft = 0;
			float totalAccumLeft = 0;
			float totalRhoRight = 0;
			float totalThetaRight = 0;
			float totalAccumRight = 0;
			int countLeft = 0;
			int countRight = 0;

			for(int k = 0; k < maxLaneNum; k++){
				float rho = rep_ref[k];
				float theta = rep_ref[k + maxLaneNum];
				int accum = rep_ref[k + 2*maxLaneNum];
				line_polar(rho, theta, accum, cartPt, k);

				if(count_ref[k] > 0){
					double slope = (double)(cartPt[4*k+3] - cartPt[4*k+1])/(double)(cartPt[4*k+2] - cartPt[4*k]);
					if(cartPt[4*k+2] < 640){

						if (slope > .2 && slope < 3){ // leftLane
							totalRhoLeft = totalRhoLeft + rho;
							totalThetaLeft = totalThetaLeft + theta;
							totalAccumLeft = totalAccumLeft + accum;
							countLeft++;
						}
					}
					else // right lane
						if(slope < -.2 && slope > -3){
							totalRhoRight = totalRhoRight + rho;
							totalThetaRight = totalThetaRight + theta;
							totalAccumRight = totalAccumRight + accum;
							countRight++;
						}
				}


				//line(cartPt, k, frame.data);
			}
			float avgRhoLeft = totalRhoLeft/countLeft;
			float avgThetaLeft = totalThetaLeft/countLeft;
			float avgAccumLeft = totalAccumLeft/countLeft;
			float avgRhoRight = totalRhoRight/countRight;
			float avgThetaRight = totalThetaRight/countRight;
			float avgAccumRight = totalAccumRight/countRight;

			line_polar(avgRhoLeft, avgThetaLeft, avgAccumLeft, leftPts, 0);
			line_polar(avgRhoRight, avgThetaRight, avgAccumRight, rightPts, 0);

			// convert points from polar to cartesian in [x0, x1, y0, y1] format

			// choose two lines representing current lane
			//findCurrentLane(rep_ref, count_ref, maxLaneNum, frameFound, max, cartPt, leftPts, rightPts);

			// draw lines
			line(leftPts, 0, frame.data);
			line(rightPts, 0, frame.data);


		/*lane_matching(hough, rep_ref, count_ref, maxLaneNum, pLine,
				trackThreshold, frameFound, frameLost, max);

		// convert points from polar to cartesian in [x0, x1, y0, y1] format
		for(int k = 0; k < maxLaneNum; k++){
			float rho = rep_ref[k];
			float theta = rep_ref[k + maxLaneNum];
			int accum = rep_ref[k + 2*maxLaneNum];
			line_polar(rho, theta, accum, cartPt, 4*k);
			line(cartPt, k, frame.data);
		}

		// choose two lines representing current lane
		//findCurrentLane(rep_ref, count_ref, maxLaneNum, frameFound, max, cartPt, leftPts, rightPts);

		// draw lines
		line(leftPts, 0, frame.data);
		line(rightPts, 0, frame.data);

		GETTIME(&end);
		printf("lane tracking time elapsed:%5.3fms \n",(end-start)/1000.0f);


		/*int prev_rho = 0;
		float prev_theta = 0;
		for(int i=0; i<max; i++)
		{
			printf("%d. accumulator:%d\n",i,apexcv::HoughLineDetector::accumulator(pLine[i]));
			printf("%d. rhoID:%d\n",i,apexcv::HoughLineDetector::rhoID(pLine[i]));
			printf("%d. thetaID:%d\n",i,apexcv::HoughLineDetector::thetaID(pLine[i]));
			printf("%d. rho:%d\n",i,hough.line(pLine[i]).rho);
			printf("%d. theta:%f\n",i,hough.line(pLine[i]).theta);

			//unsigned char *buffer_out = (unsigned char *)gFrameIsp.mImage.mData;
			//unsigned char *buffer_out = (unsigned char *)frame.data;
			//unsigned char *buffer_out = (unsigned char *)lInput0.GetDataPtr();
			//line(0,0,1200,700,buffer_out);

			// only print lines if they are different enough from each other
			int current_rho = hough.line(pLine[i]).rho;
			float current_theta = hough.line(pLine[i]).theta;

			int difference = abs((current_rho - prev_rho) + (int)(100.0f *(current_theta - prev_theta)));

			int* pt = (int*)malloc(4 * sizeof(int));

			//if(difference > 100)
				line_polar(current_rho, current_theta,
						apexcv::HoughLineDetector::accumulator(pLine[i]), cartPt, i);
				line(cartPt, i, frame.data);
				free(pt);

			prev_rho = current_rho;
			prev_theta = current_theta;
		}*/
		//unsigned char *buffer_out = (unsigned char *)gFrameIsp.mImage.mData;
		//line_polar(rhoL, thetaL, accumL, buffer_out);
		//line_polar(rhoR, thetaR, accumR, buffer_out);

		//bool preProcess = false;
		//if(preProcess){
			//convert gray to color (output must be color format)
			/*GETTIME(&start);
			{
				unsigned char *buffer_in = (uint8_t*)srcFilter1.GetDataPtr();
				//unsigned char *buffer_in = (uint8_t *)thres_out.GetDataPtr();
				//unsigned char *buffer_in = (uint8_t *)dstFilter1.GetDataPtr();
				//unsigned char *buffer_out = (unsigned char *)(uintptr_t)gFrameIsp.mImage.mData;
				unsigned char *buffer_out = (unsigned char *)(uintptr_t)frame.data;
				//unsigned char *buffer_out = (unsigned char*)(uintptr_t)lInput0.GetDataPtr();
				for(int i =0; i<1280*720; i++)
				{
					buffer_out[i*3 + 0] =  buffer_in[i];
					buffer_out[i*3 + 1] =  buffer_in[i];
					buffer_out[i*3 + 2] =  buffer_in[i];
				}
			}

			GETTIME(&end);
			printf("Gray2RGB elapsed:%5.3fms \n",(end-start)/1000.0f);*/
		//}

		// Write CSULA to the bottom left corner of the display
		char str[200];
		sprintf(str,"CSULA");

		//cv::Mat frame = cv::Mat(720, 1280, CV_8UC3, (unsigned char *)gFrameIsp.mImage.mData);
		//cv::Mat frame = cv::Mat(720, 1280, CV_8UC3, (unsigned char *)input_mat.data);
		//cv::Mat frame = cv::Mat(720, 1280, CV_8UC3, (unsigned char *)lInput0.GetDataPtr());

		cv::Mat textImg = cv::Mat::zeros(90, 225, frame.type());  //Create Black Image Matrix for image (y,x)
		cv::putText(textImg, str, cv::Point(10, 65), cv::FONT_HERSHEY_SIMPLEX, 2.0,cv::Scalar(255,255,255),8); //Put CSULA (x,y)
		cv::flip(textImg,textImg,-1); // Flip 180
		textImg.copyTo(frame(cv::Rect(1020,10,textImg.cols, textImg.rows))); //(x,y) top left orgin

		//cv::putText(frame, str, cv::Point(100, 100), 1, 1, cv::Scalar(200, 100, 50), 2);
		//unsigned char *input = (unsigned char*)(frame.data);

		// Put frame to output buffer

		//lDcuOutput.PutFrame((void*)thres_out.GetDataPtr());
        //lDcuOutput.PutFrame((void*)(uintptr_t)gFrameIsp.mImage.mData);
        //lDcuOutput.PutFrame(input);

		//void *lp_buffer2 = OAL_MemoryAllocFlag(1280*720*3, OAL_MEMORY_FLAG_ALIGN(ALIGN2_CACHELINE)| OAL_MEMORY_FLAG_CONTIGUOUS);
		//memset(lp_buffer2, 0, 1280*720*3);

		//cv::Mat output_mat = cv::Mat(720, 1280, CV_8UC3, lp_buffer2);
		//input_mat.copyTo(output_mat(cv::Rect(0, 232, 256, 256)));
		//frame.copyTo(output_mat);
		lDcuOutput.PutFrame(frame.data);

		// Write frame to video
		cv::flip(frame, frame, -1);
		vw.write(frame);
		//OAL_MemoryFree(lp_buffer2);

		//OAL_MemoryFree(lp_buffer1);
		//OAL_MemoryFree(lp_buffer0);
		OAL_MemoryFree(buffer_thres);
		//cap.release();

		//if(cameraOn)
			if(lpGrabber->FramePush(gFrameIsp) != LIB_SUCCESS)
			{
			  break;
			} // if push failed

        // step 1
        if(sStop) break;

      } // no error
    } // for LOOP_NUM

    // step 1
    if(sStop) break;

    //if(cameraOn)
		if(NULL == gFrameIsp.mImage.mData)
		{
		  break;
		} // if gpFramout == NULL

    // *** log output ****
  } // while(1)

  //printf("\nSequencer stopped working in frame %d\n",gFrmCnt);

  // Step 2: from matlab codegen main.c
  /* Terminate the application.
       You do not need to do this more than one time. */
  detectLanesOnly_terminate();

  //if(cameraOn)
	  if(lpGrabber)
	  {
		//*** Stop ISP processing ***
		lpGrabber->Stop();

		lpGrabber->Release();

		delete lpGrabber;
	  } // if grabber exists

#ifdef __STANDALONE__
  for(;;);  // don't return on standalone
#endif // #ifdef __STANDALONE__

  // step 9
  free(rep_ref);
  free(count_ref);
  	free(cartPt);
  	free(leftPts);
  	free(rightPts);

}



#ifndef __STANDALONE__
void SigintHandler(int aSigNo)
{
  sStop = true;
} // SigintHandler()

//***************************************************************************

int32_t SigintSetup()
{
  int32_t lRet = SEQ_LIB_SUCCESS;

  // prepare internal signal handler
  struct sigaction lSa;
  memset(&lSa, 0, sizeof(lSa));
  lSa.sa_handler = SigintHandler;

  if( sigaction(SIGINT, &lSa, NULL) != 0)
  {
    VDB_LOG_ERROR("Failed to register signal handler.\n");
    lRet = SEQ_LIB_FAILURE;
  } // if signal not registered

  return lRet;
} // SigintSetup()

//***************************************************************************
#endif // #ifndef __STANDALONE__
