/*    
 * framebuffertest.cpp
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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
 *To test that the Linux framebuffer is set up correctly, and that the device permissions
 *are correct, use the program below which opens the frame buffer and draws a gradient-
 *filled red square:

 *retrieved from:
 *Testing the Linux Framebuffer for Qtopia Core (qt4-x11-4.2.2)

 *http://cep.xor.aps.anl.gov/software/qt4-x11-4.2.2/qtopiacore-testingframebuffer.html
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

using namespace std;

int main()
{
    int fbfd = 0;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize = 0;
    char *fbp = 0;
    int x = 0, y = 0;
    long int location = 0;

    // Open the file for reading and writing
    fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1)
    {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");

    // Get fixed screen information
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) 
    {
        perror("Error reading fixed information");
        exit(2);
    }

    // Get variable screen information
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) 
    {
        perror("Error reading variable information");
        exit(3);
    }

    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;

    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    
    if (!fbp) 
    {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
    
    printf("The framebuffer device was mapped to memory successfully.\n");

    x = 100; y = 100;       // Where we are going to put the pixel

    // Figure out where in memory to put the pixel
    for (y = 100; y < 300; y++)
        for (x = 100; x < 300; x++)
        {
			location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                       (y+vinfo.yoffset) * finfo.line_length;

            if (vinfo.bits_per_pixel == 32)
            {
                *(fbp + location) = 100;        		// B
                *(fbp + location + 1) = 15+(x-100)/2;   // G
                *(fbp + location + 2) = 200-(y-100)/5;  // R
                *(fbp + location + 3) = 0;      		// No transparency
														//location += 4;
            }

        }
    munmap(fbp, screensize);
    close(fbfd);
    return 0;
}