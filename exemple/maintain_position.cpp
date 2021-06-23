#include <string>
#include <iostream>
#include <unistd.h>

#include <Spi_Interface.hpp>

using namespace std;

int main()
{

	command_packet send;
	sensor_packet rcv;
	Spi_Interface *spi = new Spi_Interface(true);

	cout << "Starting...\n";

	if (spi->start("/dev/spidev0.0") != 0) {
		return 1;
	}

	memset(&send, 0, sizeof send);
	memset(&send, 0, sizeof send);

	send.enable_system = true;
	send.enable_motor1 = true;

	send.position_motor1 = 0;
	send.Kp_motor1 = 10;
	send.Kd_motor1 = 0.1;

	unsigned int t = 1000000; //every seconds

	while (true)
	{
		spi->transmit(send, &rcv);
		cout << "\rMotor1 enabled : " << rcv.motor1_enabled << "	ready : " << rcv.motor1_ready << endl;
		usleep(t);
	}

	return 0;
}