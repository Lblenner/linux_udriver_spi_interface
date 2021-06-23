#include "format.hpp"
#include "Spi_Interface.hpp"

Spi_Interface::Spi_Interface(bool debug)
{
	this->debug = debug;
	memset(&xfer, 0, sizeof xfer);
	xfer.tx_buf = (unsigned long)buf;
	xfer.rx_buf = (unsigned long)buf;
	xfer.len = 34;
}

int Spi_Interface::transmit(command_packet send, sensor_packet *rcv)
{
	if (!initialized)
	{
		DEBUG(std::cout << "Interface was not initialized" << std::endl;)
		return 1;
	}
	fill_buf(send);

	int status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0)
	{
		DEBUG(std::cout << "Ioctl error : " << strerror(errno) << std::endl;)
		return 1;
	}
	read_buf(rcv);
	return 0;
}

int Spi_Interface::start(const char *device)
{
	initialized = false;
	int ret;

	DEBUG(std::cout << "Opening SPI device " << device << " ..." << std::endl;)

	fd = open(device, O_RDWR);

	if (fd < 0)
	{
		DEBUG(std::cout << "Can't open " << device << std::endl;)
		return 1;
	}

	DEBUG(std::cout << "SPI device " << device << " opened" << std::endl;)

	int mode = SPI_MODE_0;
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);

	if (ret < 0)
	{
		DEBUG(std::cout << "Can't set the spi mode : " << std::endl
				   << strerror(errno) << std::endl;)
		close(fd);
		return 1;
	}

	DEBUG(std::cout << "SPI mode set to SPI_MODE_0" << std::endl;)

	// Set clock speed
	uint32_t speed = 8000000;
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	if (ret < 0)
	{
		DEBUG(std::cout << "Clock speed could not be changed : " << std::endl
				   << strerror(errno) << std::endl;)
		close(fd);
		return -1;
	}

	DEBUG(std::cout << "SPI clock speed set to 8MHz" << std::endl;)

	initialized = true;

	return 0;
}

int Spi_Interface::stop()
{
	if (initialized)
	{
		close(fd);
		return 0;
	}
	else
	{
		DEBUG(std::cout << "Interface was not initialized" << std::endl;)
		return -1;
	}
}

void Spi_Interface::fill_buf(command_packet p)
{
	memset(buf, 0, sizeof buf);

	// Mode
	buf[0] = ((p.enable_system & 0b1) << 7) + ((p.enable_motor1 & 0b1) << 6) +
			 ((p.enable_motor2 & 0b1) << 5) + ((p.position_rollover & 0b1) << 4) +
			 ((p.index_offset_compensation_motor1 & 0b1) << 3) + ((p.index_offset_compensation_motor2 & 0b1) << 2) +
			 ((p.timeout >> 6) & 0b11);
	buf[1] = p.timeout & 0b00111111;

	// Position
	int32_t pos1 = FLOAT_TO_D32QN(p.position_motor1, UD_QN_POS);
	buf[2] = (pos1 >> 24) & 0b11111111;
	buf[3] = (pos1 >> 16) & 0b11111111;
	buf[4] = (pos1 >> 8) & 0b11111111;
	buf[5] = pos1 & 0b11111111;

	int32_t pos2 = FLOAT_TO_D32QN(p.position_motor2, UD_QN_POS);
	buf[6] = (pos2 >> 24) & 0b11111111;
	buf[7] = (pos2 >> 16) & 0b11111111;
	buf[8] = (pos2 >> 8) & 0b11111111;
	buf[9] = pos2 & 0b11111111;

	// Velocity
	int16_t v1 = FLOAT_TO_D16QN(p.velocity_motor1, UD_QN_VEL);
	buf[10] = (v1 >> 8) & 0b11111111;
	buf[11] = v1 & 0b11111111;

	int16_t v2 = FLOAT_TO_D16QN(p.velocity_motor2, UD_QN_VEL);
	buf[12] = (v2 >> 8) & 0b11111111;
	buf[13] = v2 & 0b11111111;

	// Iq
	int16_t iq1 = FLOAT_TO_D16QN(p.iq_motor1 * 10, UD_QN_IQ);
	buf[14] = (iq1 >> 8) & 0b11111111;
	buf[15] = iq1 & 0b11111111;

	int16_t iq2 = FLOAT_TO_D16QN(p.iq_motor2 * 10, UD_QN_IQ);
	buf[16] = (iq2 >> 8) & 0b11111111;
	buf[17] = iq2 & 0b11111111;

	// Kp
	int16_t kp1 = FLOAT_TO_D16QN(p.Kp_motor1, UD_QN_KP);
	buf[18] = (kp1 >> 8) & 0b11111111;
	buf[19] = kp1 & 0b11111111;

	int16_t kp2 = FLOAT_TO_D16QN(p.Kp_motor2, UD_QN_KP);
	buf[20] = (kp2 >> 8) & 0b11111111;
	buf[21] = kp2 & 0b11111111;

	// Kd
	int16_t kd1 = FLOAT_TO_D16QN(p.Kd_motor1, UD_QN_KD);
	buf[22] = (kd1 >> 8) & 0b11111111;
	buf[23] = kd1 & 0b11111111;

	int16_t kd2 = FLOAT_TO_D16QN(p.Kd_motor2, UD_QN_KD);
	buf[24] = (kd2 >> 8) & 0b11111111;
	buf[25] = kd2 & 0b11111111;

	// Isat
	buf[26] = FLOAT_TO_uD8QN(p.I_sat_motor1 * 10, UD_QN_ISAT);
	buf[27] = FLOAT_TO_uD8QN(p.I_sat_motor2 * 10, UD_QN_ISAT);

	// Index
	buf[28] = (p.index >> 8) & 0b11111111;
	buf[29] = p.index & 0b11111111;

	// Add CRC
	uint32_t crc = CRC_compute(buf, 30);
	buf[30] = (crc >> 24) & 0b11111111;
	buf[31] = (crc >> 16) & 0b11111111;
	buf[32] = (crc >> 8) & 0b11111111;
	buf[33] = crc & 0b11111111;
}

void Spi_Interface::read_buf(sensor_packet *rcv)
{
	// Mode
	rcv->system_enabled = (buf[0] >> 7) & 0b00000001;
	rcv->motor1_enabled = (buf[0] >> 6) & 0b00000001;
	rcv->motor1_ready = (buf[0] >> 5) & 0b00000001;
	rcv->motor2_enabled = (buf[0] >> 4) & 0b00000001;
	rcv->motor2_ready = (buf[0] >> 3) & 0b00000001;
	rcv->index_1_detected = (buf[0] >> 2) & 0b00000001;
	rcv->index_2_detected = (buf[0] >> 1) & 0b00000001;
	rcv->index_1_toggle = buf[0] & 0b00000001;

	rcv->index_2_toggle = (buf[1] >> 7) & 0b00000001;
	rcv->error_code = buf[1] & 0b00001111;

	// Timestamp
	rcv->timestamp = (buf[2] << 8) + buf[3];

	// Position
	int32_t p1 = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + (buf[7]);
	rcv->position_motor1 = D32QN_TO_FLOAT(p1, UD_QN_POS);
	int32_t p2 = (buf[8] << 24) + (buf[9] << 16) + (buf[10] << 8) + (buf[11]);
	rcv->position_motor2 = D32QN_TO_FLOAT(p2, UD_QN_POS);

	// Velocities
	int16_t v1 = (buf[12] << 8) + (buf[13]);
	rcv->velocity_motor1 = D16QN_TO_FLOAT(v1, UD_QN_VEL);
	int16_t v2 = (buf[14] << 8) + (buf[15]);
	rcv->velocity_motor2 = D16QN_TO_FLOAT(v2, UD_QN_VEL);

	// Iq
	int16_t iq1 = (buf[16] << 8) + (buf[17]);
	rcv->iq_motor1 = D16QN_TO_FLOAT(iq1, UD_QN_IQ) / 10;
	int16_t iq2 = (buf[18] << 8) + (buf[19]);
	rcv->iq_motor2 = D16QN_TO_FLOAT(iq2, UD_QN_IQ) / 10;

	// Coils resistances
	uint16_t coil1 = (buf[20] << 8) + (buf[21]);
	rcv->coil_resistance_motor1 = D16QN_TO_FLOAT(coil1, UD_QN_CR);
	uint16_t coil2 = (buf[22] << 8) + (buf[23]);
	rcv->coil_resistance_motor2 = D16QN_TO_FLOAT(coil2, UD_QN_CR);

	// ADC
	uint16_t adc1 = (buf[24] << 8) + (buf[25]);
	rcv->adc_motor1 = D16QN_TO_FLOAT(adc1, UD_QN_ADC);
	uint16_t adc2 = (buf[26] << 8) + (buf[27]);
	rcv->adc_motor2 = D16QN_TO_FLOAT(adc2, UD_QN_ADC);

	// Index
	rcv->last_index = (buf[28] << 8) + (buf[29]);

	rcv->crc = (buf[30] << 24) + (buf[31] << 16) + (buf[32] << 8) + (buf[33]);
}
