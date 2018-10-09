/*nrf definition */
//#include "nrf.h"                        // Device header
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "boards.h" // it is needed
#include "bmi160.h"
#include "nrf_drv_gpiote.h"

// prototypes
void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
uint32_t config_gpio();

//#include "sdk_config.h"
//#include "nrf_gpio.h"


//#include "RTE_Components.h"             // Component selection
//#include "system_nrf52.h"               // NordicSemiconductor::Device:Startup


/*
#define SPI_ENABLED 1
#define SPI0_ENABLED 0
*/

#define SPI_INSTANCE 0

#define SPI_SS_PIN  26 /*grey wire  CS on the bmi */
#define SPI_MISO_PIN 23 /*white wire SCL on the BMI+*/
#define SPI_MOSI_PIN 24 /*blu wire SDX on the bmi*/
#define SPI_SCK_PIN 22 /*violet SCL on the bmi*/ 

#define BUTTON_PIN 27


static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0); 
static volatile bool spi_xfer_done; 

static uint8_t SPI_RX_Buffer[100]; // Allocate a buffer for SPI reads
struct bmi160_dev sensor; // An instance of bmi160 sensor
 
// 200 bytes -> ~7bytes per frame -> ~28 data frames
struct bmi160_sensor_data acc_data[28];

/**
* Function for reading FIFO data
*/
int8_t get_bmi160_fifo_data()
{
 int8_t rslt = BMI160_OK;
 uint8_t acc_frames_req = 28;
 // Read the fifo buffer using SPI
 rslt = bmi160_get_fifo_data(&sensor);

 // Parse the data and extract 28 gyro frames
 rslt = bmi160_extract_accel(acc_data, &acc_frames_req, &sensor);

 return rslt;
}

// Declare memory to store the raw FIFO buffer information
uint8_t fifo_buff[200];

// Modify the FIFO buffer instance and link to the device instance
struct bmi160_fifo_frame fifo_frame;

 void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	spi_xfer_done = true; // Set a flag when transfer is done
}
 

/**
* Function for setting up the SPI communication.
*/
uint32_t spi_config()
{
	 uint32_t err_code;

	 // Use nRF's default configurations
	 nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	 // Define each GPIO pin
	 spi_config.ss_pin = SPI_SS_PIN;
	 spi_config.miso_pin = SPI_MISO_PIN;
	 spi_config.mosi_pin = SPI_MOSI_PIN;
	 spi_config.sck_pin = SPI_SCK_PIN;

	 // Initialize the SPI peripheral and give it a function pointer to
	 // it’s event handler
	 err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);

	 return err_code;
 }

/**
* Function for writing to the BMI160 via SPI.
*/
int8_t bmi160_spi_bus_write(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
	 spi_xfer_done = false; // set the flag down during transfer

	 int32_t error = 0;
	 // Allocate array, which lenght is address + number of data bytes to be sent
	 uint8_t tx_buff[cnt+1];

	 uint16_t stringpos;
	 // AND address with 0111 1111; set msb to '0' (write operation)
	 tx_buff[0] = reg_addr & 0x7F;

	 for (stringpos = 0; stringpos < cnt; stringpos++) {
	 tx_buff[stringpos+1] = *(reg_data + stringpos);
	 }
	 // Do the actual SPI transfer
	 nrf_drv_spi_transfer(&spi, tx_buff, cnt+1, NULL, 0);

	 while (!spi_xfer_done) {}; // Loop until the transfer is complete

	 return (int8_t)error;
}

/**
* Function for reading from the BMI160 via SPI.
*/
int8_t bmi160_spi_bus_read(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	 spi_xfer_done = false; // set the flag down during transfer
	 int32_t error = 0;
	 uint8_t tx_buff = reg_addr | 0x80; // OR address with 1000 0000; Read -> set msbto 1
	 uint8_t * rx_buff_pointer;
	 uint16_t stringpos;
	 rx_buff_pointer = (uint8_t *) (SPI_RX_Buffer);

	 // Do the actual SPI transfer
	 nrf_drv_spi_transfer(&spi, &tx_buff, 1, rx_buff_pointer, len+1);

	 while (!spi_xfer_done) {} // Loop until the transfer is complete
	 // Copy received bytes to reg_data
	 for (stringpos = 0; stringpos < len; stringpos++)
	 *(reg_data + stringpos) = SPI_RX_Buffer[stringpos + 1];

	 return (int8_t)error;
}

/**
* Function for configuring the sensor
*/
int8_t sensor_config()
{
	 int8_t rslt = BMI160_OK;
	 sensor.id = 0; // We use SPI so id == 0
	 sensor.interface = BMI160_SPI_INTF;
	 // Give the driver the correct interfacing functions
	 sensor.read = bmi160_spi_bus_read;
	 sensor.write = bmi160_spi_bus_write;
	 sensor.delay_ms = nrf_delay_ms;
	 // Initialize the sensor and check if everything went ok
	 rslt = bmi160_init(&sensor);

		// Configure the accelerometer's sampling freq, range and modes
sensor.accel_cfg.odr = BMI160_ACCEL_ODR_25HZ;
sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G;
sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
// Set the configurations
rslt = bmi160_set_sens_conf(&sensor);

// Some fifo settings
fifo_frame.data = fifo_buff;
fifo_frame.length = 200;
sensor.fifo = &fifo_frame;

// Configure the sensor's FIFO settings
rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL, BMI160_ENABLE, &sensor);
// Create an instance for interrupt settings
struct bmi160_int_settg int_config;

// Interrupt channel/pin 1
int_config.int_channel = BMI160_INT_CHANNEL_1;
// Choosing fifo watermark interrupt
int_config.int_type = BMI160_ACC_GYRO_FIFO_WATERMARK_INT;

// Set fifo watermark level to 180
rslt = bmi160_set_fifo_wm((uint8_t) 180, &sensor);

// Enabling interrupt pins to act as output pin
int_config.int_pin_settg.output_en = BMI160_ENABLE;
// Choosing push-pull mode for interrupt pin
int_config.int_pin_settg.output_mode = BMI160_DISABLE;
// Choosing active high output
int_config.int_pin_settg.output_type = BMI160_ENABLE;
// Choosing edge triggered output
int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;
// Disabling interrupt pin to act as input
int_config.int_pin_settg.input_en = BMI160_DISABLE;
// Non-latched output
int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;
// Enabling FIFO watermark interrupt
int_config.fifo_WTM_int_en = BMI160_ENABLE;

// Set interrupt configurations
rslt = bmi160_set_int_config(&int_config, &sensor);
	 return rslt;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	/*Variables group*/
	uint32_t errorSpi, errorSensor, errorGPIO ; 
	int8_t result; 
  /* Configure board. */
  bsp_board_init(BSP_INIT_LEDS);
	/*Init the sensor*/
	errorSpi = spi_config();
	
	errorSensor = sensor_config();

	errorGPIO = config_gpio();
	// Configure the pin as input
	
	while (true)
	{
	}
}


/**
* Function for configuring General Purpose I/O.
*/

uint32_t config_gpio()
{

	uint32_t err_code = NRF_SUCCESS;

  if(!nrf_drv_gpiote_is_init())

               {

                              err_code = nrf_drv_gpiote_init();

               }

// Set which clock edge triggers the interrupt

               nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

// Configure the internal pull up resistor

               config.pull = NRF_GPIO_PIN_PULLUP;

// Configure the pin as input

               err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_handler);

              

               if (err_code != NRF_SUCCESS)

               {

               // handle error condition

               }

  // Enable events
  nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
               return err_code;

}


/**
* Handler for GPIO events.
*/
void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
 		int8_t error = get_bmi160_fifo_data();
		
}



// configuration button
