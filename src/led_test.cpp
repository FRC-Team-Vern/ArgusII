#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <condition_variable>
#include "WPILib.h"
#include "networktables/NetworkTable.h"
#include "tables/ITableListener.h"
#include <mutex>              // std::mutex, std::unique_lock

//using namespace std;

#define DAEMON_NAME "vdaemon"

std::mutex cond_var_mtx;
std::mutex led_pattern_mtx;
std::condition_variable cond_var;

volatile std::pair<bool,double> pending_pattern = std::make_pair(false,0.0);

void update_pixel( int pixel, unsigned char r, unsigned char g, unsigned char b );
bool write_strip( void );
void process();
void make_all_LEDs_red();
void make_all_LEDs_green();
void make_all_LEDs_blue();
void make_all_LEDs_off();
void make_all_LEDs_white();
void make_rainbow_wave();

enum Pattern { off, white, red, green, blue, rainbow_wave };

//SPI interface
int fd;
const char *device = "/dev/spidev0.0";
const uint8_t mode = SPI_MODE_0;
const uint8_t msb = 0;
const uint8_t bits = 8;
const uint32_t speed = 800000;

#define NUM_LEDS 80

unsigned char master_array[ NUM_LEDS * 3 ];

unsigned char r_array[ NUM_LEDS ];
unsigned char g_array[ NUM_LEDS ];
unsigned char b_array[ NUM_LEDS ];

volatile double pattern = 0.0;

int total_size = NUM_LEDS * 3;

int delay = 5000;


class LED_DataTableListener : public ITableListener {

	virtual void ValueChanged(ITable* source, llvm::StringRef key, std::shared_ptr<nt::Value> value, bool isNew) {
		//std::cout << "ValueChange called:" << std::endl;
		double pattern = value->GetDouble();
		//std::cout << "Pattern: " << pattern << std::endl;
		std::unique_lock<std::mutex> lck(led_pattern_mtx);
		pending_pattern.first = true;
		pending_pattern.second = pattern;
		std::unique_lock<std::mutex> cond_var_lck(cond_var_mtx);
		cond_var.notify_all();
	}
};

void update_pixel( int pixel, unsigned char r, unsigned char g, unsigned char b ) {

	if( pixel >= NUM_LEDS ) {

		return;
	}

	r_array[ pixel ] = r;
	g_array[ pixel ] = g;
	b_array[ pixel ] = b;
}


bool write_strip( void ) {
	struct spi_ioc_transfer mesg;

	int master_offset = 0;
	int ret;

	for(int i = 0; i < NUM_LEDS; i++ ) {
		master_array[ master_offset ] = r_array[ i ];
		master_offset++;

		master_array[ master_offset ] = g_array[ i ];
		master_offset++;

		master_array[ master_offset ] = b_array[ i ];
		master_offset++;
	}

	memset( &mesg, 0, sizeof( mesg ) );

	mesg.tx_buf = (unsigned long)master_array;
	mesg.rx_buf = (unsigned long) NULL;
	mesg.len = sizeof( master_array );
	mesg.delay_usecs = delay;
	mesg.speed_hz = speed;
	mesg.bits_per_word = bits;
	mesg.pad = 0;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &mesg);
	if (ret ==-1) {
		printf("can't send data\n");
		perror( "error" );
	}

	std::unique_lock<std::mutex> lck(led_pattern_mtx);
	return !pending_pattern.first;
}


void process() {

	syslog (LOG_NOTICE, "Writing to my Syslog");
}


void initSPI() {
	//SPI setup
	int ret = 0;
	fd = 0;
	fd = open(device, O_WRONLY);
	if (fd <0) {
		printf("can't open device\n");
	}

	/*****************************************************
	 * spi mode
	 */
	/*ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	    if (ret ==-1)
	    {
	    	printf("can't set spi mode 1\n");
	    }
	 */

	ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &msb);
	if (ret ==-1) {
		printf("can't set spi mode 2\n");
	}
	printf("MSB mode: %d\n",msb);

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret ==-1) {
		printf("Can't set bits per word\n");
	}

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret ==-1) {
		printf("can't set max speed hz\n");
	}

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed /1000);
	printf("sizeof master_array %d\n", sizeof( master_array ) );
}


void runLEDs( void ) {
	initSPI();

	make_all_LEDs_off();
	make_all_LEDs_red();
	make_all_LEDs_off();
	make_all_LEDs_green();
	make_all_LEDs_off();
	make_all_LEDs_blue();
	make_all_LEDs_off();
	make_rainbow_wave();
	make_all_LEDs_off();
}


void make_rainbow_wave() {
	int pos;

	bool continue_pattern = true;
	for(int k= 0; k < 10 && continue_pattern; k++ ) {
		for(int j = 0; j < 256 && continue_pattern; j++ ) {

			for(int i = 0; i < NUM_LEDS; i++ ) {

				pos = ( ( ( i * 256 / NUM_LEDS ) + j ) % 256 );
				if( pos < 85 ) {

					update_pixel( i, pos * 3, 255 - pos * 3, 0 );
				}
				else if( pos < 170 ) {

					pos -= 85;
					update_pixel( i, 255 - pos * 3, 0, pos * 3 );
				}
				else {

					pos -= 170;
					update_pixel( i, 0, pos * 3, 255 - pos * 3 );
				}
			}
			continue_pattern = write_strip();
			usleep( 10000 );
		}
	}
}


void make_all_LEDs_blue(){
	for( int i = 0; i < NUM_LEDS; i++ ) {
		update_pixel( i, 0, 0, 0xff );
	}
	write_strip();
	sleep(1);
}


void make_all_LEDs_off(){
	for(int i = 0; i < NUM_LEDS; i++ ) {
		update_pixel( i, 0, 0, 0 );
	}
	write_strip();
	sleep(1);
}


void make_all_LEDs_red(){
	for( int i = 0; i < NUM_LEDS; i++ ) {
		update_pixel( i, 0xff, 0, 0 );
	}
	write_strip();
	sleep(1);
}


void make_all_LEDs_green(){
	for( int i = 0; i < NUM_LEDS; i++ ) {
		update_pixel( i, 0, 0xff, 0 );
	}
	write_strip();
	sleep(1);
}


void make_all_LEDs_white(){
	for( int i = 0; i < NUM_LEDS; i++ ) {

		update_pixel( i, 0xff, 0xff, 0xff);
	}
	write_strip();
	sleep( 1 );
}

int main(int argc, char *argv[]) {

	//Set our Logging Mask and open the Log
	setlogmask(LOG_UPTO(LOG_NOTICE));
	openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);

	syslog(LOG_INFO, "Entering Daemon");

	pid_t pid, sid;

	//Fork the Parent Process
	pid = fork();

	if (pid < 0) { exit(EXIT_FAILURE); }

	//We got a good pid, Close the Parent Process
	if (pid > 0) { exit(EXIT_SUCCESS); }

	//Change File Mask
	umask(0);

	//Create a new Signature Id for our child
	sid = setsid();
	if (sid < 0) { exit(EXIT_FAILURE); }

	//Change Directory
	//If we cant find the directory we exit with failure.
	if ((chdir("/")) < 0) { exit(EXIT_FAILURE); }

	//Close Standard File Descriptors
	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	//----------------
	//Main Process
	//----------------

	initSPI();

	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress("localhost");
	std::shared_ptr<NetworkTable> table= NetworkTable::GetTable("datatable");
	table->AddTableListener(new LED_DataTableListener());

	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	bool pending = false;
	double pattern = 0.0;
	while(true) {
		//std::cout << "Checking for new Pattern: " << std::endl;

		led_pattern_mtx.lock();
		pending = pending_pattern.first;
		pattern = pending_pattern.second;
		led_pattern_mtx.unlock();

		if (pending) {
			led_pattern_mtx.lock();
			pending_pattern.first = false;
			led_pattern_mtx.unlock();
		} else {
			//std::cout << "No pattern pending:" << std::endl;
			std::unique_lock<std::mutex> lck(cond_var_mtx);
			cond_var.wait(lck);
			led_pattern_mtx.lock();
			pattern = pending_pattern.second;
			led_pattern_mtx.unlock();
		}

		switch((int)pattern) {
		case Pattern::off:
			make_all_LEDs_off();
			break;
		case Pattern::white:
			make_all_LEDs_white();
			break;
		case Pattern::red:
			make_all_LEDs_red();
			break;
		case Pattern::green:
			make_all_LEDs_green();
			break;
		case Pattern:: blue:
			make_all_LEDs_blue();
			break;
		case Pattern::rainbow_wave:
			make_rainbow_wave();
			break;
		default:
			break;
		}
	}

	//Close the log
	closelog ();
}

