/*
* stecam_calibrationLR.cpp
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
* RAW Rinput( Left 8bpp+Right 8bpp).
* Demosaicing with opencv GPU process, Not guaranted working on every 
* machine.
*
* This program can be used and distributed without restrictions.
**
* see
* http://linuxtv.org/docs.php
* http://docs.opencv.org/3.1.0 for more information
*
* 										Nov.2.2016 Kyu H Oh a.k.a Dr.Q.
*/

#define TRUE 1
#define FALSE 0

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

enum _AppMode 
{
	Start,
	Reference_View,
	Calib_Image,
	Calib_Read,
	Calib_Process,
	Calib_Undistortion,
	Final_Review,
	ERROR,
	USER_STOP
};
enum _AppMode AppMode;
	

using namespace cv;
using namespace std;

/// ///////////////////////
void quit(const char * msg)
/// ///////////////////////
{
	fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

/// //////////////////////////////////////
int xioctl(int fd, int request, void* arg)
/// //////////////////////////////////////
{
	for (int i = 0; i < 100; i++)
	{
		int r = ioctl(fd, request, arg);
		if (r != -1 || errno != EINTR) return r;
	}
	
	return -1;
}

typedef struct
{
	uint16_t* start;
	size_t length;
}buffer_t;

typedef struct
{
	int fd;
	uint32_t width;
	uint32_t height;
	size_t buffer_count;
	buffer_t* buffers;
	buffer_t head;
}camera_t;

static const int cam_width (1280);
static const int cam_height (960);

/// ///////////////////////////////////////////////////////////////////////
camera_t* camera_open(const char * device, uint32_t width, uint32_t height)
/// ///////////////////////////////////////////////////////////////////////
{
	int fd = open(device, O_RDWR | O_NONBLOCK, 0);
	
	if (fd == -1) quit("Opening Video Capture Device Failed");
	
	camera_t* camera =(camera_t*) malloc(sizeof (camera_t));
	camera->fd = fd;
	camera->width = width;
	camera->height = height;
	camera->buffer_count = 0;
	camera->buffers = NULL;
	camera->head.length = 0;
	camera->head.start = NULL;
	
	return camera;
}

/// //////////////////////////////
void camera_init(camera_t* camera)
/// //////////////////////////////
{
	struct v4l2_capability cap;
	
	if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1)
		quit("ERROR CODE: VIDIOC_QUERYCAP");
	
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
		quit("This Video Device Cannot Capture");

	if (!(cap.capabilities & V4L2_CAP_STREAMING))
		quit("This Video Device Cannot Streaming");

	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0)
	{
		struct v4l2_crop crop;
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;
		
		if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1)
			fprintf (stdout, "Cropping Not Supported");
		
	
	}	

	struct v4l2_format format;
	memset(&format, 0, sizeof format);

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = camera->width;
	format.fmt.pix.height = camera->height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	format.fmt.pix.field = V4L2_FIELD_NONE;

	if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1)
		quit("ERROR CODE: VIDIOC_S_FMT");

	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1)
		quit("ERROR CODE: VIDIOC_REQBUFS");
	
	camera->buffer_count = req.count;
	camera->buffers = (buffer_t*) calloc(req.count, sizeof (buffer_t));

	size_t buf_max = 0;

	for (size_t i = 0; i < camera->buffer_count; i++)
	{
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof buf);
	
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
	
		if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1)
			quit("ERROR CODE: VIDIOC_QUERYBUF");
		
		if (buf.length > buf_max)
			buf_max = buf.length;
		
		camera->buffers[i].length = buf.length;
		camera->buffers[i].start = (uint16_t*) mmap(NULL,
												buf.length,
												PROT_READ | PROT_WRITE,
												MAP_SHARED,
												camera->fd,
												buf.m.offset);
												
		if (camera->buffers[i].start == MAP_FAILED)
			quit("ERROR CODE: MMAP()");	
	}
	camera->head.start = (uint16_t*) malloc(buf_max);
}

/// ///////////////////////////////
void camera_start(camera_t* camera)
/// ///////////////////////////////
{
	for (size_t i = 0; i < camera->buffer_count; i++)
	{
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof buf);
		
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		
	if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1)
		quit("ERROR CODE: VIDIOC_QBUF");
}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1)
		quit("ERROR CODE: VIDIOC_STREAMON");
}

/// //////////////////////////////
void camera_stop(camera_t* camera)
/// //////////////////////////////
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1)
		quit("ERROR CODE: VIDIOC_STREAMOFF");
}

/// ////////////////////////////////
void camera_finish(camera_t* camera)
/// ////////////////////////////////
{
	for (size_t i = 0; i < camera->buffer_count; i++)
	{
		munmap(camera->buffers[i].start, camera->buffers[i].length);
	}
	
	free(camera->buffers);
	camera->buffer_count = 0;
	camera->buffers = NULL;
	free(camera->head.start);
	camera->head.length = 0;
	camera->head.start = NULL;
}

/// ///////////////////////////////
void camera_close(camera_t* camera)
/// ///////////////////////////////
{

	if (close(camera->fd) == -1)
		quit("Video Device Closing Failed" );
		
	free(camera);
}

/// ////////////////////////////////
int camera_capture(camera_t* camera)
/// ////////////////////////////////
{
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	
	if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1)
		return FALSE;
		
	memcpy(camera->head.start,
		   camera->buffers[buf.index].start,
		   buf.bytesused);
		   
	camera->head.length = buf.bytesused;
	
	if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1)
		return FALSE;
	
	return TRUE;
}

/// //////////////////////////////////////////////////////
int camera_frame(camera_t* camera, struct timeval timeout)
/// //////////////////////////////////////////////////////
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(camera->fd, &fds);
	
	int r = select(camera->fd + 1, &fds, 0, 0, &timeout);
	
	if (r == -1) 
		quit ("Frame Selection Time out.");
		
	if (r == 0)
		return FALSE;
		
	return camera_capture(camera);
	
}

/// ///////////////////////
int cutoff_0_FF(int value)
/// ///////////////////////
{
return (value < 0) ? 0 : (0xFF < value) ? 0xFF : value;
}


/// /////////////////////////////////////////////////////////////////////////////////
void GetMatLeftEye(uint16_t* yuyv, uint32_t width, uint32_t height,uint8_t* left_buf)
/// /////////////////////////////////////////////////////////////////////////////////
{
		
	unsigned int L;
										
	for (size_t y = 0; y < height; y +=2)
		for (size_t x = 0; x < width; x += 2)
		{
			size_t index = y * width + x;	
			
			L = yuyv[index]%256;
			left_buf[index] = cutoff_0_FF(L);								
			
			L = yuyv[index + 1]%256;
			left_buf[index + 1] = cutoff_0_FF(L);
			
			L = yuyv[index+width]%256;
			left_buf[index + width] = cutoff_0_FF(L);
		
			L = yuyv[index+width+1]%256;
			left_buf[index + width+1] = cutoff_0_FF(L);
		}
	return;
}


/// ///////////////////////////////////////////////////////////////////////////////////
void GetMatRightEye(uint16_t* yuyv, uint32_t width, uint32_t height,uint8_t* right_buf)
/// ///////////////////////////////////////////////////////////////////////////////////
{
		
	unsigned int R;
										
	for (size_t y = 0; y < height; y +=2)
		for (size_t x = 0; x < width; x += 2)
		{
			size_t index = y * width + x;	
			
			R = yuyv[index]>>8;
			right_buf[index] = cutoff_0_FF(R);								
			
			R = yuyv[index + 1]>>8;
			right_buf[index + 1] = cutoff_0_FF(R);
			
			R = yuyv[index+width]>>8;
			right_buf[index + width] = cutoff_0_FF(R);
		
			R = yuyv[index+width+1]>>8;
			right_buf[index + width +1] = cutoff_0_FF(R);
		
		}
		
	return;

}


/// ////////////////////////////////////////////////////////////////////////
void ChessBoardSize (Size board_sz, float square_size, vector<Point3f>& obj)
/// ////////////////////////////////////////////////////////////////////////
{
	for( int i = 0; i < board_sz.height; i++ )
	{
		for( int j = 0; j < board_sz.width; j++ )
		{
			obj.push_back (Point3f(float(j*square_size),float(i*square_size), 0.0f));
		}
	}
}                                


/// ////////////////////////////////////////////////////////////////////
void ReferenceView (camera_t* camera)
/// ////////////////////////////////////////////////////////////////////
{
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	char key (-1);
	
	// Caption Location.
	Point CaptionLocation;
	CaptionLocation.x = 10;
	CaptionLocation.y = 40;
	
	//Font
	int FontFace (0);
	double FontScale (1);
	
	cv::ocl::setUseOpenCL(true);
	
	Mat lframe;
	Mat rframe;
	
	UMat luframe,ruframe, luframergb, ruframergb;
	
	lframe = Mat(cam_height, cam_width, CV_8U);
	rframe = Mat(cam_height, cam_width, CV_8U);
	luframe = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	ruframe = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	luframergb = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
	ruframergb = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
	
		/* skip 5 frames for booting a cam */
	for (int i = 0; i < 5; i++)
	{
		camera_frame(camera, timeout);
	}

	namedWindow("Left", CV_WINDOW_NORMAL);
	namedWindow("Right", CV_WINDOW_NORMAL);
	
	resizeWindow("Left", 640, 480);
	resizeWindow("Right", 640, 480);
	
	moveWindow("Left", 200, 300);
	moveWindow("Right",900, 300);
	
	while(1)
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
		
		/*
		Here shoud be artificial object insertion
		 */
				
					 
		lframe = Mat(cam_height,cam_width,CV_8UC1,left_buf);
		rframe = Mat(cam_height,cam_width,CV_8UC1,right_buf);
		
		lframe. copyTo(luframe);
		rframe. copyTo(ruframe);
		
		cvtColor(luframe,luframergb,CV_BayerGR2RGB);
		cvtColor(ruframe,ruframergb,CV_BayerGR2RGB);
					
		putText(luframergb,"For Calibration Start, Please Press \"C\" Key...", CaptionLocation, FontFace, FontScale, Scalar(255,0,0));
		putText(ruframergb,"For Calibration Start, Please Press \"C\" Key...", CaptionLocation, FontFace, FontScale, Scalar(255,0,0));				 
	
		imshow("Left", luframergb);
		imshow("Right", ruframergb);
		
		
		key = waitKey(20);
		
	
		printf ("%d\n",key);
		
		if (key == 27) //'ESC'key == (int)27
		{
			AppMode = USER_STOP;
			break;
		}
		if ((key == 99)|(key== 67)) 
		{
			AppMode = Calib_Image;
			break;
		}	// 'c' key == (int) 99
		if ((key == 82)|(key == 114)) 
		{
			AppMode = Calib_Read;
			break;
		}
			 
		key = -1;		
		free(left_buf);
		free(right_buf);
		
		
		luframergb.release();
		ruframergb.release();
		luframe.release();
		ruframe.release();
			
	}

	destroyAllWindows();
		
	return ;
}

/// ////////////////////////////////////////////////////////////////////
void Calibrate(camera_t* camera)
/// ////////////////////////////////////////////////////////////////////
{
	char key;
	int skipframe (0);
	FileStorage fs;
	vector <string> file_name;
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
		
	// ChessBoard 
	int numCornersHor (9);
    int numCornersVer (7);
    float square_size (0.025f);
 
    
    Size board_sz = Size(numCornersHor, numCornersVer);
	
	// Variables for for calibration
	 
	vector < vector <Point3f> > object_points;
    vector < vector <Point2f> > L_image_points;
    vector < vector <Point2f> > R_image_points;
	
	vector < Point2f > L_corners;
	vector < Point2f > R_corners;
	
	int stored_image_pairs=0;
     
    vector<Point3f> obj;
    
    ChessBoardSize (board_sz, square_size, obj);
    
	// Caption Location.
	Point CaptionLocation;
	CaptionLocation.x = 10;
	CaptionLocation.y = 40;
	
	//Font
	int FontFace (0);
	double FontScale (1);
	
	
	
		
	cv::ocl::setUseOpenCL(true);
	
	Mat lframe;
	Mat rframe;
	
	UMat luframe,ruframe, luframegray, ruframegray, luframegrayrsz, ruframegrayrsz, luframergb, ruframergb;
	
	lframe = Mat(cam_height, cam_width, CV_8U);
	rframe = Mat(cam_height, cam_width, CV_8U);
	
	
	luframe = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	luframegray = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	luframegrayrsz= UMat(cam_height*0.5, cam_width*0.5, CV_8U,USAGE_DEFAULT);
	
	ruframe = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	ruframegray = UMat(cam_height, cam_width, CV_8U, USAGE_DEFAULT);
	luframegrayrsz= UMat(cam_height*0.5, cam_width*0.5, CV_8U,USAGE_DEFAULT);
	
	luframergb = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
	ruframergb = UMat(cam_height, cam_width, CV_8UC3, USAGE_DEFAULT);
	
	
	
	//	/* skip 5 frames for booting a cam */
	
	for (int i = 0; i < 5; i++)
	{
		camera_frame(camera, timeout);
	}
	
	
	namedWindow("Left", CV_WINDOW_NORMAL);
	namedWindow("Right",CV_WINDOW_NORMAL);
	resizeWindow("Left", 640, 480);
	resizeWindow("Right", 640, 480);
	moveWindow("Left", 400, 300);
	moveWindow("Right",1100, 300);

	while(1)
	{
		key=-1;
		
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
		
		cvtColor(luframe,luframegray,CV_BayerGR2GRAY);
		cvtColor(ruframe,ruframegray,CV_BayerGR2GRAY);
		
		resize(luframegray,luframegrayrsz,Size(),0.5,0.5,CV_INTER_AREA);
		resize(ruframegray,ruframegrayrsz,Size(),0.5,0.5,CV_INTER_AREA);
		
			
		bool Lfound = findChessboardCorners(luframegrayrsz,
											board_sz,
											L_corners,
											CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);	
		
		
        if (Lfound)
        {
            cornerSubPix(luframegrayrsz,
						 L_corners,
						 Size(11, 11),
						 Size(-1, -1),
						 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
		}
		
		bool Rfound = findChessboardCorners(ruframegrayrsz,
											board_sz,
											R_corners,
											CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
			
        if(Rfound)
        {
            cornerSubPix(ruframegrayrsz,
						 R_corners,
						 Size(11, 11),
						 Size(-1, -1),
						 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        }	
       						
		
		imshow("Left", luframegrayrsz);
		imshow("Right", ruframegrayrsz);							
		
		if (Lfound !=0 && Rfound != 0 && skipframe == 0) 
		{
		
			putText(luframegrayrsz, "Press <SPACE> for Store.", Point2f(10,400), FontFace, FontScale, Scalar(255,255,255));
			putText(ruframegrayrsz, "Press Other keys for Skip.", Point2f(10,400), FontFace, FontScale, Scalar(255,255,255));

			imshow("Left", luframegrayrsz);
			imshow("Right", ruframegrayrsz);		
			
			while (key == -1)
			{
				key=waitKey(1); 
			}
			skipframe = 30;
		} 
		
			
		printf ("%d\n",key);
		
		if (key == 27) //'ESC'key == (int)27
			{
				if (stored_image_pairs < 10)
				{
					AppMode = ERROR;
				}
				else 
				{
					AppMode = Calib_Process;
					break;
				}
			}
			
		if (key == 32) 
			{
				stored_image_pairs ++;
				
				cvtColor(luframe,luframergb,CV_BayerGR2RGB);
				cvtColor(ruframe,ruframergb,CV_BayerGR2RGB);
				
				findChessboardCorners(luframergb,
											board_sz,
											L_corners,
											CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
				
				cvtColor(luframergb,luframegray,CV_BGR2GRAY);
				
				cornerSubPix(luframegray,
						 L_corners,
						 Size(11, 11),
						 Size(-1, -1),
						 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				
				findChessboardCorners(ruframergb,
											board_sz,
											R_corners,
											CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
				
				cvtColor(ruframergb,ruframegray,CV_BGR2GRAY);
				
				cornerSubPix(ruframegray,
						 R_corners,
						 Size(11, 11),
						 Size(-1, -1),
						 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				
				L_image_points.push_back(L_corners);
				R_image_points.push_back(R_corners);
				
				object_points.push_back(obj);
									
				string LeftImageFileName ;
				string RightImageFileName ;
								
				LeftImageFileName = "L" + to_string (stored_image_pairs)+".jpg";
				RightImageFileName = "R" + to_string (stored_image_pairs)+".jpg";
				
				file_name.push_back(LeftImageFileName);
				file_name.push_back(RightImageFileName);
				
				fs.open ("imagecalibration.json", FileStorage::WRITE);				
				fs << "TotalNumber" << stored_image_pairs;
				fs << "File Name" << "[" << file_name << "]";
								
				fs.release();
				
				
					
				imwrite ("./images/"+LeftImageFileName, luframergb);
				imwrite ("./images/"+RightImageFileName, ruframergb);
				
				drawChessboardCorners(luframergb, board_sz, L_corners, Lfound);
				drawChessboardCorners(ruframergb, board_sz, R_corners, Rfound);
				
				imshow("Left", luframergb);
				imshow("Right", ruframergb);
				
				//re-initialize parameters
				
				Lfound=false; Rfound=false;
		
			}	// 'SPACE' == 32
			

		key=waitKey(10);

		free(left_buf);
		free(right_buf);
		
		luframegrayrsz.release();
		luframegrayrsz.release();
		luframegray.release();
		ruframegray.release();
		luframergb.release();
		ruframergb.release();
		luframe.release();
		ruframe.release();
		lframe.release();
		rframe.release();
		
		if (key == 27) //'ESC'key == (int)27
		{
			if (stored_image_pairs < 20)
			{
				AppMode = ERROR;
				break;
			}
			else 
			{
				AppMode = Calib_Process;
				
				break;
			}
		}
		
		if (skipframe>0) skipframe -= 1;	
	}

/// AppMode : Calib_Process
	
	destroyAllWindows();
	
	if (AppMode == ERROR) 
	{
		printf ("ERROR");
		return;
	}
	
	namedWindow("L-Undistorted", CV_WINDOW_NORMAL);
	namedWindow("R-Undistorted",CV_WINDOW_NORMAL);
	resizeWindow("L-Undistorted", 640, 480);
	resizeWindow("R-Undistorted", 640, 480);
	moveWindow("L-Undistorted", 400, 300);
	moveWindow("R-Undistorted",1100, 300);

		
	AppMode = Calib_Process;
	
	Mat L_intrinsic = Mat(3, 3, CV_32FC1);
	Mat R_intrinsic = Mat(3, 3, CV_32FC1);
	Mat L_distCoeffs;
	Mat R_distCoeffs;
	
    vector<Mat> L_rvecs;
    vector<Mat> R_rvecs;
    vector<Mat> L_tvecs;
    vector<Mat> R_tvecs;
    
    L_intrinsic.ptr<float>(0)[0] = 1;
    R_intrinsic.ptr<float>(0)[0] = 1;
    
    L_intrinsic.ptr<float>(1)[1] = 1;
    R_intrinsic.ptr<float>(1)[1] = 1;
    
    calibrateCamera(object_points, L_image_points, Size(cam_width,cam_height), L_intrinsic, L_distCoeffs, L_rvecs, L_tvecs);
    calibrateCamera(object_points, R_image_points, Size(cam_width,cam_height), R_intrinsic, R_distCoeffs, R_rvecs, R_tvecs);
    
    Mat luframergbUndistorted;
	Mat ruframergbUndistorted;
    
    /// Calib_Undistortion    
    
    AppMode = Calib_Undistortion;
    
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
		
		undistort(luframergb, luframergbUndistorted, L_intrinsic, L_distCoeffs);
		undistort(ruframergb, ruframergbUndistorted, R_intrinsic, R_distCoeffs);
	
		imshow("L-Undistorted",luframergbUndistorted);
		imshow("R-Undistorted",luframergbUndistorted); 
	
		key=waitKey(30);
		
		free(left_buf);
		free(right_buf);
		
		luframergbUndistorted.release();
		ruframergbUndistorted.release();
		luframergb.release();
		ruframergb.release();
		luframe.release();
		ruframe.release();
		lframe.release();
		rframe.release();
		
		if (key == 27) 
		{ 
			AppMode = ERROR;
			break;
		}
		if ((key == 'Y') |( key =='y')) 
		{
		// SAVE PARAMETERS
			break;
			
		}
	}
	
	destroyAllWindows();
	
	return ;
}


/// /////////////////////////////////////////////////////////
void CalibRead(camera_t* camera)
/// /////////////////////////////////////////////////////////
{
	
	vector<string> file_name;
	int total (0);
	char key(-1);
	
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
	string LeftImageFileName;
	string RightImageFileName;
	
	vector < vector <Point3f> > object_points;
    vector < vector <Point2f> > L_image_points;
    vector < vector <Point2f> > R_image_points;
	vector < Point2f > L_corners;
	vector < Point2f > R_corners;
	
	
	// ChessBoard 
	int numCornersHor (9);
    int numCornersVer (7);
    float square_size (0.025f);
     
    Size board_sz = Size(numCornersHor, numCornersVer);
	
	vector<Point3f> obj;
    
    ChessBoardSize (board_sz, square_size, obj);
	
	Mat lframe, rframe, lframergb, rframergb;
	UMat luframe,ruframe, luframergb, ruframergb;
	
	lframergb = Mat(cam_height,cam_width,CV_8UC3);
	rframergb = Mat(cam_height,cam_width,CV_8UC3);
	
	lframe = Mat(cam_height,cam_width,CV_8UC1);
	rframe = Mat(cam_height,cam_width,CV_8UC1);
	
	FileStorage fs ("imagecalibration.json", FileStorage::READ);
	
	fs["TotalNumber"] >> total;
	
	FileNode nd = fs["File Name"];                         // Read string sequence - Get node
	if (nd.type() != FileNode::SEQ)
	{
		quit("strings is not a sequence! FAIL");
	}
	
	FileNodeIterator it = nd.begin(), it_end = nd.end(); // Go through the node
	for (; it != it_end; ++it)
    {
		file_name.push_back ((string)*it);
	}
	
	fs.release();
	 
	for (int i=0; i<total; i++)
	{
		LeftImageFileName = file_name.at(2*i);
		RightImageFileName = file_name.at(2*i+1);
	
		cout << LeftImageFileName << endl << RightImageFileName << endl;
		lframergb= imread ("./images/"+LeftImageFileName);
		rframergb= imread ("./images/"+RightImageFileName);
		
		cvtColor (lframergb,lframe,CV_RGB2GRAY);
		cvtColor (rframergb,rframe,CV_RGB2GRAY);
	
		findChessboardCorners(lframergb,
							  board_sz,
							  L_corners,
							  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	
		cornerSubPix(lframe,
					 L_corners,
					 Size(11, 11),
					 Size(-1, -1),
					 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	
		findChessboardCorners(rframergb,
							  board_sz,
							  R_corners,
							  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	
		cornerSubPix(rframe,
					 R_corners,
					 Size(11, 11),
					 Size(-1, -1),
					 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	
		L_image_points.push_back(L_corners);
		R_image_points.push_back(R_corners);
		object_points.push_back(obj);
	
	}
	lframergb.release();
	rframergb.release();
	
	namedWindow("L-Undistorted", CV_WINDOW_NORMAL);
	namedWindow("R-Undistorted",CV_WINDOW_NORMAL);
	resizeWindow("L-Undistorted", 640, 480);
	resizeWindow("R-Undistorted", 640, 480);
	moveWindow("L-Undistorted", 400, 300);
	moveWindow("R-Undistorted",1100, 300);

	AppMode = Calib_Process;
	
	Mat L_intrinsic = Mat(3, 3, CV_32FC1);
	Mat R_intrinsic = Mat(3, 3, CV_32FC1);
	Mat L_distCoeffs;
	Mat R_distCoeffs;
	
    vector<Mat> L_rvecs;
    vector<Mat> R_rvecs;
    vector<Mat> L_tvecs;
    vector<Mat> R_tvecs;
    
    L_intrinsic.ptr<float>(0)[0] = 1;
    R_intrinsic.ptr<float>(0)[0] = 1;
    
    L_intrinsic.ptr<float>(1)[1] = 1;
    R_intrinsic.ptr<float>(1)[1] = 1;
    
    calibrateCamera(object_points, L_image_points, Size(cam_width,cam_height), L_intrinsic, L_distCoeffs, L_rvecs, L_tvecs);
    calibrateCamera(object_points, R_image_points, Size(cam_width,cam_height), R_intrinsic, R_distCoeffs, R_rvecs, R_tvecs);
    
    Mat luframergbUndistorted;
	Mat ruframergbUndistorted;
    
    /// Calib_Undistortion    
    
    AppMode = Calib_Undistortion;
    
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
		
		undistort(luframergb, luframergbUndistorted, L_intrinsic, L_distCoeffs);
		undistort(ruframergb, ruframergbUndistorted, R_intrinsic, R_distCoeffs);
	
		imshow("L-Undistorted",luframergbUndistorted);
		imshow("R-Undistorted",luframergbUndistorted); 
	
		key=waitKey(30);
		
		free(left_buf);
		free(right_buf);
		
		luframergbUndistorted.release();
		ruframergbUndistorted.release();
		luframergb.release();
		ruframergb.release();
		luframe.release();
		ruframe.release();
		lframe.release();
		rframe.release();
		
		if (key == 27) 
		{ 
			AppMode = ERROR;
			break;
		}
		if ((key == 'Y') |( key =='y')) 
		{
		// SAVE PARAMETERS
			break;
			
		}
	
	

	}
	
	destroyAllWindows();
	return;
}

/// ////////////////////////////////////////////////////////////////////
/// ////////////////////////////////////////////////////////////////////
int main() {
	AppMode = Start;
    
	camera_t* camera = camera_open("/dev/video0",
								   cam_width,
								   cam_height);
	camera_init(camera);
	camera_start(camera);

		
	AppMode = Reference_View;
	
	ReferenceView (camera);
	
	if (AppMode==USER_STOP) 
		{	
				
		camera_stop(camera);
		camera_finish(camera);
		camera_close(camera);
	
		return 0;
		}
		
	if (AppMode==Calib_Image)
	{
		Calibrate (camera);
		
		if (AppMode == ERROR) 
		{
		}	
	}	
	
	if (AppMode==Calib_Read)
	{
		CalibRead(camera);
	}
				
	camera_stop(camera);
	camera_finish(camera);
	camera_close(camera);
	
	return 0;
}
