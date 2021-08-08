/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

 /* Authors: SP Kong, JH Yang */
 /* maintainer: Pyo */

#include <string>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <vector>

const uint8_t KSYNC0 = 0xAA;
const uint8_t KSYNC1 = 0x55;


enum State {SYNC0, SYNC1, HEADER, DATA};

typedef struct 
{
	float x;
	float y;
	int8_t intensity;
}Point_2d;

typedef struct 
{
	float distance;
	float angle;
	int8_t intensity;
}Vector_2d;

typedef struct 
{
	uint8_t type;
    uint8_t data_length;
    uint16_t start_angle;
    uint16_t stop_angle; 
}Package_header;

class Mb_1r2t
{
public:
	uint16_t rpms;

	Mb_1r2t(const std::string& port, uint32_t baud_rate);
	~Mb_1r2t();
	void parsePacket();
	bool scanReady();
	void close() { shutting_down_ = true; }
	float getRotationSpeed();
	void setCloudCallback(void (*callback)(std::vector<Point_2d> *points));
	void setScanCallback(void (*callback)(std::vector<Vector_2d> *vectors));
	
	

private:
	
	std::vector<Point_2d> points_; //cloud_data
	std::vector<Vector_2d> vectors_; // scan_data
		
	
	void (*cloud_callback_)(std::vector<Point_2d> *points) = nullptr;
	void (*scan_callback_)(std::vector<Vector_2d> *vectors) = nullptr;

	float last_angle_;
	bool scan_ready_;
	std::string port_;
	uint32_t baud_rate_;
	bool shutting_down_;
	boost::asio::io_service io_;
	boost::asio::serial_port serial_;
	float rotationSpeed_; 
};

