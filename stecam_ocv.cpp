/*
* stecam_ocv.cpp
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
* 										Aug.29.2016 Kyu H Oh a.k.a Dr.Q.
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

void quit(const char * msg)
{
	fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

int xioctl(int fd, int request, void* arg)
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

camera_t* camera_open(const char * device, uint32_t width, uint32_t height)
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


void camera_init(camera_t* camera)
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


void camera_start(camera_t* camera)
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

void camera_stop(camera_t* camera)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1)
		quit("ERROR CODE: VIDIOC_STREAMOFF");
}

void camera_finish(camera_t* camera)
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

void camera_close(camera_t* camera)
{

	if (close(camera->fd) == -1)
		quit("Video Device Closing Failed" );
		
	free(camera);
}


int camera_capture(camera_t* camera)
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

int camera_frame(camera_t* camera, struct timeval timeout)
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
int cutoff_0_FF(int value)
{
return (value < 0) ? 0 : (0xFF < value) ? 0xFF : value;
}

void GetMatEachEye(uint16_t* yuyv, uint32_t width, uint32_t height,uint8_t* lr)
{
		
	unsigned int G,H;
										
	for (size_t y = 0; y < height; y +=2)
		for (size_t x = 0; x < width; x += 2)
		{
			size_t index = y * width + x;	
			
			G = yuyv[index]%256;
			H = yuyv[index]>>8;
			
			lr[index + y*width ] = cutoff_0_FF(G);								
			lr[index + width+ y*width ] = cutoff_0_FF(H);							
			

			G = yuyv[index+1]%256;
			H = yuyv[index+1]>>8;
			
			lr[index + y*width + 1 ] = cutoff_0_FF(G);
			lr[index + width + y*width + 1 ] = cutoff_0_FF(H);
			
			G = yuyv[index+width]%256;
			H = yuyv[index+width]>>8;
			
			lr[index + y*width + 2*width ] = cutoff_0_FF(G);
			lr[index + width  + y*width + 2*width ] = cutoff_0_FF(H);

			G = yuyv[index+width+1]%256;
			H = yuyv[index+width+1]>>8;
			
			lr[index + y*width + 2*width +1 ] = cutoff_0_FF(G);
			lr[(index + width) + y*width + 2*width + 1 ] = cutoff_0_FF(H);
		}
		
	return;

}

/*
uint8_t* GetMatEachEye_C3(uint16_t* yuyv, uint32_t width, uint32_t height)
{
	uint8_t* rgblr = (uint8_t*) calloc(width * height * 3 * 2,
										sizeof (uint8_t));
	
	unsigned int G,H;
										
	for (size_t y = 0; y < height; y +=2)
		for (size_t x = 0; x < width; x += 2)
		{
			size_t index = y * width + x;	
			
			G = yuyv[index]%256;
			H = yuyv[index]>>8;
			
			rgblr[index * 3 + 3*y*width + 0 ] = cutoff_0_FF(G);	//R							
			rgblr[index * 3 + 3*y*width + 1 ] = 0;				//G
			rgblr[index * 3 + 3*y*width + 2 ] = 0;					//B
			
			rgblr[(index + width) * 3 + 3*y*width + 0 ] = cutoff_0_FF(H);	//R							
			rgblr[(index + width) * 3 + 3*y*width + 1 ] = 0;				//G
			rgblr[(index + width) * 3 + 3*y*width + 2 ] = 0;					//B
			

			G = yuyv[index+1]%256;
			H = yuyv[index+1]>>8;
			
			rgblr[index * 3 + 3*y*width + 3 ] = 0;				//R	
			rgblr[index * 3 + 3*y*width + 4 ] = cutoff_0_FF(G);	//G
			rgblr[index * 3 + 3*y*width + 5 ] = 0;				//B
			
			rgblr[(index + width) * 3 + 3*y*width + 3 ] = 0;				//R
			rgblr[(index + width) * 3 + 3*y*width + 4 ] = cutoff_0_FF(H);	//G
			rgblr[(index + width) * 3 + 3*y*width + 5 ] = 0;				//B


			G = yuyv[index+width]%256;
			H = yuyv[index+width]>>8;
			
			rgblr[index * 3 + 3*y*width + 0 ] = 0;				//R	
			rgblr[index * 3 + 3*y*width + 1 ] = cutoff_0_FF(G);	//G
			rgblr[index * 3 + 3*y*width + 2 ] = 0;					//B
			
			rgblr[(index + width) * 3 + 3*y*width + 0 ] = 0;				//R
			rgblr[(index + width) * 3 + 3*y*width + 1 ] = cutoff_0_FF(H);	//G
			rgblr[(index + width) * 3 + 3*y*width + 2 ] = 0;					//B
			

			G = yuyv[index+width+1]%256;
			H = yuyv[index+width+1]>>8;
			
			rgblr[index * 3 + 3*y*width + 3 ] = 0;				//R	
			rgblr[index * 3 + 3*y*width + 4 ] = 0;				//G
			rgblr[index * 3 + 3*y*width + 5 ] = cutoff_0_FF(G);	//B
			
			rgblr[(index + width) * 3 + 3*y*width + 3 ] = 0;				//R
			rgblr[(index + width) * 3 + 3*y*width + 4 ] = 0;				//G
			rgblr[(index + width) * 3 + 3*y*width + 5 ] = cutoff_0_FF(H);	//B
		}
		
	return rgblr;

}
*/
int main()
{
	char key;
	
	
	cv::ocl::setUseOpenCL(true);
	
	Mat lrframe;
	UMat lruframe,lruframergb,lruframergbrsz;
	
	lrframe = Mat(cam_height, (2 * cam_width), CV_8U);
	lruframe = UMat(cam_height, (2 * cam_width), CV_8U, USAGE_DEFAULT);
	lruframergb = UMat(cam_height, (2 * cam_width), CV_8UC3, USAGE_DEFAULT);
	lruframergbrsz = UMat (cam_height*0.5,  cam_width, CV_8UC3, USAGE_DEFAULT);
	
	camera_t* camera = camera_open("/dev/video0",
								   cam_width,
								   cam_height);
	camera_init(camera);
	camera_start(camera);

	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	
		/* skip 5 frames for booting a cam */
	for (int i = 0; i < 5; i++)
	{
		camera_frame(camera, timeout);
	}

	namedWindow("Output", CV_WINDOW_AUTOSIZE);
	
	moveWindow("Output", 1300, 300);
	
	while(1)
	{
		camera_frame(camera, timeout);
		
		uint8_t* rgblr = (uint8_t*) calloc((camera->width) * (camera->height) * 2, sizeof (uint8_t));
		// uint8_t* lr = (uint8_t*) calloc(width * height * 2, sizeof (uint8_t));
		
		GetMatEachEye(camera->head.start,
							  camera->width,
							  camera->height,
							  rgblr);
		
		/*
		Here shoud be artificial object insertion
		 */
		
		
		
		
					 
		lrframe = Mat(cam_height,(2*cam_width),CV_8UC1,rgblr);
		
		lrframe. copyTo(lruframe);
		
		cvtColor(lruframe,lruframergb,CV_BayerGR2RGB);
		resize (lruframergb,lruframergbrsz,Size(),0.5,0.5,CV_INTER_AREA);
		
		imshow("Output", lruframergbrsz);
		
		key=waitKey(20);
		
		if (key == 27) //'ESC'key == (int)27
			break;
			
		free(rgblr);
		lruframergbrsz.release();
		lruframergb.release();
		lruframe.release();
		lrframe.release();
		
		
	}

	destroyAllWindows();
		
	camera_stop(camera);
	camera_finish(camera);
	camera_close(camera);
	
	return 0;
}
