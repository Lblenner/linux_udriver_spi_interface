#ifndef SPI_INTERFACE_HPP_
#define SPI_INTERFACE_HPP_

#include <iostream>
#include <string>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <iomanip>

#include <unistd.h>
#include <linux/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

using namespace std;

#define DEBUG(pr) \
	if (debug)    \
	{             \
		pr        \
	}

struct command_packet
{
	bool enable_system;
	bool enable_motor1;
	bool enable_motor2;
	bool position_rollover;
	bool index_offset_compensation_motor1;
	bool index_offset_compensation_motor2;
	uint8_t timeout;
	double position_motor1; // Unit : turn 	 	Min : -128	Max : 127,999999940395
	double position_motor2;
	double velocity_motor1; // Unit : k rpm 	Min : -16 	Max : 15,9995117188
	double velocity_motor2;
	double iq_motor1; 		// Unit : A 		Min : -32 	Max : 31,9990234375
	double iq_motor2;
	double Kp_motor1; 		// Unit : A/rot 	Min : 0 	Max : 31.99951171875
	double Kp_motor2; 
	double Kd_motor1;		// Unit : A/k rpm 	Min : 0 	Max : 63.9990234375
	double Kd_motor2;
	double I_sat_motor1; 	// Unit : A 		Min : 0 	Max : 31,875
	double I_sat_motor2;
	uint16_t index;
};

struct sensor_packet
{
	bool system_enabled;
	bool motor1_enabled;
	bool motor2_enabled;
	bool motor1_ready;
	bool motor2_ready;
	bool index_1_detected;
	bool index_2_detected;
	bool index_1_toggle;
	bool index_2_toggle;
	uint8_t error_code; // error message from https://github.com/open-dynamic-robot-initiative/blmc_drivers/files/4856546/BLMC_CAN_Interface-010720.pdf
	double position_motor1;					// Unit : turn 	 	Min : -128	Max : 127,999999940395
	double position_motor2;
	double velocity_motor1; 				// Unit : k rpm 	Min : -16 	Max : 15,9995117188
	double velocity_motor2;
	double iq_motor1;						// Unit : A 		Min : -32 	Max : 31,9990234375
	double iq_motor2;
	double coil_resistance_motor1;			// Unit : Ohm 		Min : 0 	Max : 1,9999694824
	double coil_resistance_motor2;
	double adc_motor1;						// Unit : V 		Min : 0 	Max : 3.99993896484
	double adc_motor2;
	uint16_t last_index;
	uint16_t timestamp;
	uint32_t crc;
};

class Spi_Interface
{
public:
	/* if debug is true, informations will be printed */
	Spi_Interface(bool debug = false);

	/* send the send packet and fill the rcv packet */
	/* returns 0 on success */
	int transmit(command_packet send, sensor_packet *rcv);

	/* open the specified device (/dev/spidev0.0)*/
	/* and configures it */
	/* returns 0 on success */
	int start(const char *device);

	/* close the open device */
	/* returns 0 on success */
	int stop();

private:
	struct spi_ioc_transfer xfer;
	uint8_t buf[34];
	int fd;
	bool debug;
	bool initialized = false;

	void fill_buf(command_packet p);
	void read_buf(sensor_packet *rcv);
};

#endif