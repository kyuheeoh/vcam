
/*
* camera_functions.cpp
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

#include "camera_functions.hpp"
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



void SetupCamera(camera_t* camera, int num)
{
	char char1[100];
	sprintf(char1,"/dev/video%d",num);
		
	
	camera = camera_open(char1, cam_width, cam_height);
	camera_init(camera);
	camera_start(camera);

	return;
}



void ShutdownCamera(camera_t* camera)
{
	
	camera_stop(camera);
	camera_finish(camera);
	camera_close(camera);
		
	return ;
		
}

