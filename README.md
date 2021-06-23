# Linux udriver spi interface implementation

This was developped in order to communicate with the udriver boards using a raspberry following [this description of the protocol](https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md).

This implementation uses the `spidev` kernel module.  

```
modinfo spidev
```

In order to use the sofware, you will need at an `spidev` device detected by linux.  
We briefly explain how to achieve that on a raspberry below.  

# Usage

## Driver

Using `make` inside `src` folder will create a `build` folder containing the headers and the shared library file.

## Exemple

Using `make` inside the exemple folder will create an `exec` file.  
This program will open the device `/dev/spidev0.0` and send a packet every second to the udriver board.  
The board should initialize motor1 and maintain its position.

`exec` has to be launch with the appropriate permission in order to access `/dev/spidev0.0`.

# Raspberry configuration

In order to activate spi capabilities, the following line should be added to `config.txt`

```
dtparam=spi=on
```

After reboot, `/dev/spidev0.0` and `/devspidev0.1` should appear.

`/dev/spidev0.0` for the CE0 slave line and `/dev/spidev0.1` for the CE1 slave line.

You can now plug your spi device using [the right GPIO pins](https://www.raspberrypi.org/documentation/usage/gpio/).

For `/dev/spidev0.0` and `/dev/spidev0.1`, its gpio 7,8,9,10 and 11.

You can also activate other spi device on the gpio pins with the following lines in `config.txt`.  
 
```
dtoverlay=spi1-1cs  #1 chip select
dtoverlay=spi1-2cs  #2 chip select
dtoverlay=spi1-3cs  #3 chip select
```

Depending of the raspberry version some might be available or not.


For more info : [raspberry spi readme](https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md)

# Documentation

An `Spi_Interface` can be created in order to communicate with one `spidev` device.

```cpp
Spi_Interface(bool debug = false);
```

If `debug` is true, some feedback will be provided on stdout.

The functions return 0 on success.

#

```cpp
int start(const char *device);
int stop();
```

`start` will open the specified device (`/dev/spidev0.0` for exemple) and configure it.  

`stop` will close the device.

#

```cpp
int transmit(command_packet send, sensor_packet *rcv);
```

`transmit` will send the send packet and fill the rcv packet.

Here are structs describing the command and sensor packets provided

```cpp
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
```

