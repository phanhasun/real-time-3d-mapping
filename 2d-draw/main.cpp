/*
 *  SLAMTEC LIDAR
 *  Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <vector>

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
using namespace sf;


#include "include/sl_lidar.h" 
#include "include/sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

void print_usage(int argc, const char * argv[])
{
    printf("Simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version:  %s \n"
           "Usage:\n"
           " For serial channel %s --channel --serial <com port> [baudrate]\n"
           "The baudrate is 115200(for A2) , 256000(for A3 and S1), 1000000(for S2).\n"
		   " For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
           "The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , "SL_LIDAR_SDK_VERSION",  argv[0], argv[0]);
}


void plot_histogram(sl_lidar_response_measurement_node_hq_t * nodes, size_t count)
{
    const int BARCOUNT =  75;
    const int MAXBARHEIGHT = 100;
    const float ANGLESCALE = 360.0f/BARCOUNT;

    float histogram[BARCOUNT];
    for (int pos = 0; pos < _countof(histogram); ++pos) {
        histogram[pos] = 0.0f;
    }

    float max_val = 0;
    for (int pos =0 ; pos < (int)count; ++pos) {
        int int_deg = (int)(nodes[pos].angle_z_q14 * 90.f / 16384.f);
        if (int_deg >= BARCOUNT) int_deg = 0;
        float cachedd = histogram[int_deg];
        if (cachedd == 0.0f ) {
            cachedd = nodes[pos].dist_mm_q2/4.0f;
        } else {
            cachedd = (nodes[pos].dist_mm_q2/4.0f + cachedd)/2.0f;
        }

        if (cachedd > max_val) max_val = cachedd;
        histogram[int_deg] = cachedd;
    }

    for (int height = 0; height < MAXBARHEIGHT; ++height) {
        float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val/MAXBARHEIGHT);
        for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
            if (histogram[xpos] >= threshold_h) {
                putc('*', stdout);
            }else {
                putc(' ', stdout);
            }
        }
        printf("\n");
    }
    for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
        putc('-', stdout);
    }
    printf("\n");
}

sl_result capture_and_display(ILidarDriver * drv)
{
    sl_result ans;
    
	sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);

    printf("waiting for data...\n");

    ans = drv->grabScanDataHq(nodes, count, 0);
    if (SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
        drv->ascendScanData(nodes, count);
        plot_histogram(nodes, count);

        printf("Do you want to see all the data? (y/n) ");
        int key = getchar();
        if (key == 'Y' || key == 'y') {
            for (int pos = 0; pos < (int)count ; ++pos) {
				printf("%s theta: %03.2f Dist: %08.2f \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f);
            }
        }
    } else {
        printf("error code: %x\n", ans);
    }

    return ans;
}
void EventHandler(RenderWindow& window){
	static bool LeftButtonState;
	Event event;
	while(window.pollEvent(event)) {
		if(event.type == Event::Closed) 
			window.close();
		else if(event.type == Event::KeyPressed) {
			if(Keyboard::isKeyPressed(Keyboard::Space)) {
				//
			} else if(Keyboard::isKeyPressed(Keyboard::Escape)) {
				window.close();
			} else if(Keyboard::isKeyPressed(Keyboard::W) && event.key.control){
				window.close();
			}
		} else if (event.mouseButton.button == Mouse::Left){
			LeftButtonState = !LeftButtonState;
		}
	}
}

#define WIN_H (600)
#define WIN_W (600)
#define WIN_HCENTER (WIN_H/2)
#define WIN_WCENTER (WIN_W/2)
#define WIN_CENTER (sf::Vector2f(WIN_WCENTER, WIN_HCENTER))
#define PIXEL_PER_W ((float)WIN_W/1024.f)
#define PIXEL_PER_H ((float)WIN_H/1000.f)

sf::Vector2f polar_to_cartesian(float theta, float dist) 
{
	float x = dist*sin(theta);
	float y = dist*cos(theta);
	return sf::Vector2f(x,y);
}

float distance(sf::Vector2f p1, sf::Vector2f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float result = sqrt(dx*dx + dy*dy);
	return result;
}

#define D_THRESHOLD 10.0 // mm
#define NOUTLINER_THRESHOLD 30
#define NPOINTS 10 // points
int connect_small_distance(VertexArray &points, std::vector<VertexArray*>& vertexArrays) {
	auto *connected_points = new VertexArray;
	connected_points->setPrimitiveType(sf::LinesStrip);
	vertexArrays.push_back(connected_points);

	Vertex v = points[0];
	v.color = sf::Color::Blue;
	connected_points->append(v);

	for(int i=1; i<points.getVertexCount()-NPOINTS; i++) {

		int j=1;
		for(; j<NPOINTS; j++) {
			if(distance(points[i].position, WIN_CENTER) == 0.0) continue;
			if(distance(points[i].position, points[i+j].position)<D_THRESHOLD) {
				// connected_points->append(points[j]);
				v = points[i+j];
				v.color = sf::Color::Blue;
				connected_points->append(v);
				break;
			}
		}
		if (j==NPOINTS) {
			// if (connected_points->getVertexCount()<NOUTLINER_THRESHOLD) {
			// 	vertexArrays.pop_back();
			// 	delete connected_points;
			// }
			connected_points = new VertexArray;
			connected_points->setPrimitiveType(sf::LinesStrip);
			vertexArrays.push_back(connected_points);
			i++;
		}
		i += j;
	}

	return vertexArrays.size();
}
void filter_zero(VertexArray &points, VertexArray& dest_points)
{
	for(int i=0; i<points.getVertexCount(); i++) {
		if(distance(WIN_CENTER, points[i].position) > 0.01f) {
			Vertex v = points[i];
			v.color = sf::Color::Yellow;
			dest_points.append(v);
		}
	}
}



void grab_n_process(ILidarDriver* ldr)
{
	// init drawing window
	ContextSettings settings;
	settings.antialiasingLevel = 8;
	RenderWindow window(VideoMode(WIN_W,WIN_H),"Free fall sim", Style::Default, settings);
	window.setFramerateLimit(50);
	// invert y axis
	View view = window.getDefaultView();
	view.setSize(WIN_W, -WIN_H);
	window.setView(view); 

	while(window.isOpen()) {
		EventHandler(window);

		sl_lidar_response_measurement_node_hq_t nodes[1024];
		size_t count = _countof(nodes);
		sl_result ans = ldr->grabScanDataHq(nodes, count, 0);
		if(SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
			ldr->ascendScanData(nodes, count);

		} else {
			std::cout << "error code: " << std::hex << ans << std::endl;
			exit(-1);
		}

		 window.clear();
		VertexArray line, points, ze;
		line.setPrimitiveType(sf::LinesStrip);
		points.setPrimitiveType(sf::Points);
		ze.setPrimitiveType(sf::Points);
		line.resize(1024);
		points.resize(1024);
		for(int i=0; i<1024; i++) {
			//line[i] = sf::Vertex(sf::Vector2f(i*PIXEL_PER_W, WIN_HCENTER+ (nodes[i].dist_mm_q2/4.0f)*PIXEL_PER_H/8.f), sf::Color::Green);
			line[i] = sf::Vertex(WIN_CENTER + polar_to_cartesian(i*((2*M_PI)/1024.f), nodes[i].dist_mm_q2)/4.f*PIXEL_PER_H/8.f, sf::Color::Green);
			points[i].position = line[i].position;
			points[i].color = sf::Color::Red;
		}
		std::vector<VertexArray*> vec;
		vec.resize(0);
		connect_small_distance(points, vec);
		filter_zero(points, ze);
		for(auto i: vec) {
			window.draw(*i);
		}
		//window.draw(line);
		window.draw(points);
		//window.draw(line);
		 //window.draw(ze);
		window.display();
		int a=0;
		for(auto i: vec) {
			a++;
			delete i;
		}
		vec.clear();

	}
}

int main(int argc, const char * argv[]) {
	const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
    sl_u32         opt_channel_param_second = 0;
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

    IChannel* _channel;

    if (argc < 5) {
        print_usage(argc, argv);
        return -1;
    }

	const char * opt_is_channel = argv[1];
	if(strcmp(opt_is_channel, "--channel")==0)
	{
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			opt_channel_param_first = argv[3];
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			opt_channel_param_first = argv[3];
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
    else
	{
		print_usage(argc, argv);
		return -1;
	}

    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_health_t healthinfo;
    sl_lidar_response_device_info_t devinfo;

    do {
        // try to connect
        if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
        }
        else if (opt_channel_type == CHANNEL_TYPE_UDP) {
            _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        }
        
        if (SL_IS_FAIL((drv)->connect(_channel))) {
			switch (opt_channel_type) {	
				case CHANNEL_TYPE_SERIALPORT:
					fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
						, opt_channel_param_first);
					break;
				case CHANNEL_TYPE_UDP:
					fprintf(stderr, "Error, cannot connect to the ip addr  %s with the udp port %s.\n"
						, opt_channel_param_first, opt_channel_param_second);
					break;
			}
        }

        // retrieving the device info
        ////////////////////////////////////////
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                // you can check the detailed failure reason
                fprintf(stderr, "Error, operation time out.\n");
            } else {
                fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
                // other unexpected result
            }
            break;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("SLAMTEC LIDAR S/N: ");
        for (int pos = 0; pos < 16 ;++pos) {
            printf("%02X", devinfo.serialnum[pos]);
        }

        printf("\n"
                "Version:  %s \n"
                "Firmware Ver: %d.%02d\n"
                "Hardware Rev: %d\n"
                , "SL_LIDAR_SDK_VERSION"
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);


        // check the device health
        ////////////////////////////////////////
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("Lidar health status : ");
            switch (healthinfo.status) 
			{
				case SL_LIDAR_STATUS_OK:
					printf("OK.");
					break;
				case SL_LIDAR_STATUS_WARNING:
					printf("Warning.");
					break;
				case SL_LIDAR_STATUS_ERROR:
					printf("Error.");
					break;
            }
            printf(" (errorcode: %d)\n", healthinfo.error_code);

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            break;
        }


        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            break;
        }

		switch (opt_channel_type) 
		{	
			case CHANNEL_TYPE_SERIALPORT:
				drv->setMotorSpeed();
			break;
		}

        // take only one 360 deg scan and display the result as a histogram
        ////////////////////////////////////////////////////////////////////////////////
        if (SL_IS_FAIL(drv->startScan( 0,1 ))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
        {
            fprintf(stderr, "Error, cannot start the scan operation.\n");
            break;
        }

		delay(3000);

        if (SL_IS_FAIL(capture_and_display(drv))) {
            fprintf(stderr, "Error, cannot grab scan data.\n");
            break;

        }

    } while(0);

	// grab data and draw
	grab_n_process(drv);

	

    drv->stop();
    switch (opt_channel_type) 
	{	
		case CHANNEL_TYPE_SERIALPORT:
			delay(20);
			drv->setMotorSpeed(0);
		break;
	}
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}
