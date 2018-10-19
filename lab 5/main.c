/*nrf definition */
#include "nrf.h"                        // Device header
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "boards.h" // it is needed
#include "bmi160.h"


#define SPI_INSTANCE 0

#define SPI_SS_PIN  26 /*grey wire  CS on the bmi */
#define SPI_MISO_PIN 23 /*white wire SCL on the BMI+*/
#define SPI_MOSI_PIN 24 /*blu wire SDX on the bmi*/
#define SPI_SCK_PIN 22 /*violet SCL on the bmi*/ 

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); 
static volatile bool spi_xfer_done; 

static uint8_t SPI_RX_Buffer[100]; // Allocate a buffer for SPI reads
struct bmi160_dev sensor; // An instance of bmi160 sensor
 
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

	 return rslt;
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
	/*Variables group*/
	uint32_t errorSpi, errorSensor ; 
	int8_t result; 
	
  /* Configure board. */
  bsp_board_init(BSP_INIT_LEDS);
	/*Init the sensor*/
	errorSpi = spi_config();
	
	errorSensor = sensor_config();

	while (true)
	{

	}
}
