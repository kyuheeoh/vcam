/*
* SteCam_Stereo_Calibration.cpp
*
* Copyright 2016 Dr.Q <kyuheeoh@hotmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
* MA 02110-1301, USA.
*
*
*/

/*
* V4L2 video capture and openCV display for StereoCam with 16bpp
* RAW input( Left 8bpp+Right 8bpp).
* Demosaicing with opencv GPU process, Not guaranted working on every 
* machine.
*
* This program can be used and distributed without restrictions.
**
* see
* http://linuxtv.org/docs.php
* http://docs.opencv.org/3.1.0 for more information
*
* 										Nov.9.2016 Kyu H Oh a.k.a Dr.Q.
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

#include "camera_functions.hpp"

using namespace cv;
using namespace std;


Size imageSize(cam_width,cam_height);

///STEREO CALIBRATION

int main()

{
		/// Variables
	double focal=1.0;
	Point2d pp (0.0,0.0);
	
	
	Mat L_intrinsic = Mat(3, 3, CV_64FC1);
	Mat R_intrinsic = Mat(3, 3, CV_64FC1);
		
	Mat L_distCoeffs = Mat::zeros(8, 1, CV_64F);
	Mat R_distCoeffs = Mat::zeros(8, 1, CV_64F);
	
	Mat L_rvecs,R_rvecs;
	Mat L_tvecs,R_tvecs;
	
	//vector<Point3f> objectPoints;
	vector<Point2f> L_image_points;
	vector<Point2f> R_image_points;	
	
	
	/// Import 2D Calibration DATA from "calib2d.json"
		
	FileStorage fs;
	fs.open ("calib2d.json", FileStorage::READ);

	fs ["L_Intrinsic"] >> L_intrinsic;
	fs ["R_Intrinsic"] >> R_intrinsic;
	fs ["L_DistCoeffs"] >> L_distCoeffs;
	fs ["R_DistCoeffs"] >> R_distCoeffs;
	
	/*	
	fs ["L_rvecs"] >> L_rvecs;
	fs ["R_rvecs"] >> R_rvecs;
	fs ["L_tvecs"] >> L_tvecs;
	fs ["R_tvecs"] >> R_tvecs;
	*/
	

	fs ["L_image_points"] >> L_image_points;
	fs ["R_image_points"] >> R_image_points;
	
	
	
	///  ChessBoard Parameters
	//int numCornersHor (9);
    //int numCornersVer (6);
    //float square_size (0.0254f);
    //Size board_sz = Size(numCornersHor, numCornersVer);
	
	///  Get vector of ChessBoard Point-Cordination in 2D-plane. 
	//ChessBoardSize (board_sz, square_size, objectPoints);
	
	Mat R_rotation, T_transition, EssMtx, mask;
	
	EssMtx = findEssentialMat(L_image_points,
							  R_image_points,
							  focal,
							  pp,
							  RANSAC,
							  0.999,
							  1.0,
							  mask);
	//cout << objectPoints.at(3);						  
	recoverPose(EssMtx, L_image_points, R_image_points, R_rotation, T_transition, focal, pp, mask);

	Mat R1, R2, P1, P2, Q, L_rmap1, L_rmap2, R_rmap1, R_rmap2;
    Rect L_validRoi, R_validRoi;
	cout <<L_validRoi << R_validRoi<<endl;
    stereoRectify(L_intrinsic, L_distCoeffs,
                  R_intrinsic, R_distCoeffs,
                  imageSize, R_rotation, T_transition, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, -1, imageSize, &L_validRoi, &R_validRoi);
    cout <<L_validRoi << R_validRoi<<endl;              
    initUndistortRectifyMap(L_intrinsic, L_distCoeffs, R1, P1, imageSize, CV_16SC2, L_rmap1, L_rmap2);
    initUndistortRectifyMap(R_intrinsic, R_distCoeffs, R2, P2, imageSize, CV_16SC2, R_rmap1, R_rmap2);              
    
  
    /// ---------------------> Calculation Done!
    
	/// CAMERA SETUP!  /////////////////////////////////////////////////
    camera_t* camera;
   	camera = camera_open("/dev/video0", cam_width, cam_height);
	camera_init(camera);
	camera_start(camera);
    /// //////////////////////////////////////////////////////////////// 
      
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	cout << endl << "YAHOO!"<< endl;
		/* skip 5 frames for booting a cam */
	for (int i = 0; i < 5; i++)
	{
		camera_frame(camera, timeout);
	}
    
    
    
    
    namedWindow("Left", CV_WINDOW_NORMAL);
	namedWindow("Right", CV_WINDOW_NORMAL);
	
	
	resizeWindow("Left", 640, 480);
	resizeWindow("Right", 640, 480);
	
	
	moveWindow("Left", 200, 50);
	moveWindow("Right",900, 50);
    
    Mat lframe,rframe;
    UMat luframe,ruframe,luframergb,ruframergb,luframergb2,ruframergb2;
    
    lframe = Mat(cam_height, cam_width, CV_8U);
	rframe = Mat(cam_height, cam_width, CV_8U);
	luframe = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	ruframe = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	luframergb = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
	ruframergb = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
    luframergb2 = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
	ruframergb2 = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
    
    bool showundistorted (0), windowfor_undistorted (0);
    char key;
    //Mat L_canvas, R_canvas;
	//double sf;
	//int w1, h1;
	//sf = 900./MAX(cam_width, cam_height);
	//w1 = cvRound(cam_width*sf);
	//h1 = cvRound(cam_height*sf);
			
    
   
    
    
    while (1)
    {
		camera_frame(camera, timeout);
		
		uint8_t* left_buf = (uint8_t*) calloc((camera->width) * (camera->height), sizeof (uint8_t));
		uint8_t* right_buf = (uint8_t*) calloc((camera->width) * (camera->height), sizeof (uint8_t));
		
				
		GetMatLeftEye(camera->head.start,
							  camera->width,
							  camera->height,
							  left_buf);
		
		
		GetMatRightEye(camera->head.start,
							  camera->width,
							  camera->height,
							  right_buf);
		
					 
		lframe = Mat(cam_height,cam_width,CV_8UC1,left_buf);
		rframe = Mat(cam_height,cam_width,CV_8UC1,right_buf);
		
		lframe. copyTo(luframe);
		rframe. copyTo(ruframe);
		
		cvtColor(luframe,luframergb,CV_BayerGR2RGB);
		cvtColor(ruframe,ruframergb,CV_BayerGR2RGB);
		
		imshow("Left", luframergb);
		imshow("Right", ruframergb);
		
		if (showundistorted)
		{
			if (!windowfor_undistorted)
			{
				namedWindow("Stereo-L", CV_WINDOW_NORMAL);
				namedWindow("Stereo-R", CV_WINDOW_NORMAL);
				resizeWindow("Stereo-L", 640, 480);
				resizeWindow("Stereo-R", 640, 480);
				moveWindow("Stereo-L", 200, 550);
				moveWindow("Stereo-R",900, 550);
				windowfor_undistorted= true;
			}
			
			remap(luframergb, luframergb2, L_rmap1, L_rmap2, INTER_LINEAR);
			remap(ruframergb, ruframergb2, L_rmap1, L_rmap2, INTER_LINEAR);

			

			
		
			
			imshow("Stereo-L",luframergb2);
			imshow("Stereo-R",ruframergb2);
		}
		
		key = waitKey(20);
		cout <<(int) key <<endl;
		if (key==27) break;
		
		if ((key=='T')|(key=='t')) 
		{
			showundistorted = (!showundistorted);
			destroyWindow("Stereo-L");
			destroyWindow("Stereo-R");
			windowfor_undistorted = false;
		}	
			 
		key = -1;		
		
		
		free(left_buf);
		free(right_buf);
		
		if (showundistorted)
		{
			luframergb2.release();
			ruframergb2.release();
		}
				
		luframergb.release();
		ruframergb.release();
		luframe.release();
		ruframe.release();
		lframe.release();
		rframe.release();
	}
	
    ShutdownCamera (camera);
                  
	return 0;
}





















