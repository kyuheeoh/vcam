
/*
* camera_functions.hpp
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

using namespace cv;
using namespace std;

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

////////////////////////////////////////////////////////////////////////
/// insert below to livecast function for setting timeout  
/// struct timeval timeout;
///	timeout.tv_sec = 1;
/// timeout.tv_usec = 0;
////////////////////////////////////////////////////////////////////////

static const int cam_width (1280);
static const int cam_height (960);

void quit(const char * msg);
int xioctl(int fd, int request, void* arg);
camera_t* camera_open(const char * device, uint32_t width, uint32_t height);
void camera_init(camera_t* camera);
void camera_start(camera_t* camera);
void camera_stop(camera_t* camera);
void camera_finish(camera_t* camera);
void camera_close(camera_t* camera);

int camera_capture(camera_t* camera);
int camera_frame(camera_t* camera, struct timeval timeout);

int cutoff_0_FF(int value);

void GetMatLeftEye(uint16_t* yuyv, uint32_t width, uint32_t height,uint8_t* left_buf);
void GetMatRightEye(uint16_t* yuyv, uint32_t width, uint32_t height,uint8_t* right_buf);

void ChessBoardSize (Size board_sz, float square_size, vector<Point3f>& obj);

void SetupCamera(camera_t* camera, int num);
void ShutdownCamera(camera_t* camera);


