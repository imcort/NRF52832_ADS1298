#include "mpu6050.h"
#include "nrf_drv_twi.h"

#include "nrf_drv_ppi.h"
#include "nrf_queue.h"

#include "nrf_log.h"

#define MPU_TWI_TIMEOUT 					10000 
#define MPU_TWI_BUFFER_SIZE     	14
#define MPU_ADDRESS     					0x68

#define QUEUE_SIZE 1200

#define TWI_INSTANCE_ID     1
static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];
uint8_t twi_rx_buffer[6];

extern nrf_ppi_channel_t spi_end_transfer_channel;

NRF_QUEUE_DEF(int16_t, m_accx_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, m_accy_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, m_accz_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
		
		twi_tx_buffer[0] = MPU_REG_ACCEL_XOUT_H;
	
		nrf_drv_twi_xfer_desc_t xfer_desc;
		xfer_desc.address = MPU_ADDRESS;
		xfer_desc.primary_length = 1;
		xfer_desc.p_primary_buf = twi_tx_buffer;
		xfer_desc.p_secondary_buf = twi_rx_buffer;
		xfer_desc.secondary_length = 6;
		xfer_desc.type = NRF_DRV_TWI_XFER_TXRX;
		
		nrf_drv_twi_xfer(&m_twi, &xfer_desc, NRF_DRV_TWI_FLAG_HOLD_XFER);
		
		int16_t channel_data;
		
		channel_data = (twi_rx_buffer[0] << 8) | twi_rx_buffer[1];
    nrf_queue_push(&m_accx_queue, &channel_data);
		//NRF_LOG_INFO("%d",channel_data);

    channel_data = (twi_rx_buffer[2] << 8) | twi_rx_buffer[3];
    nrf_queue_push(&m_accy_queue, &channel_data);

    channel_data = (twi_rx_buffer[4] << 8) | twi_rx_buffer[5];
    nrf_queue_push(&m_accz_queue, &channel_data);
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_mpu6050_config = {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_mpu6050_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}

uint32_t nrf_drv_mpu_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, reg, p_data, length);

    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = MPU_ADDRESS;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

    // Transferring
    err_code = nrf_drv_twi_xfer(&m_twi, &xfer_desc, 0);

		while((!m_xfer_done) && --timeout);
		if(!timeout) return NRF_ERROR_TIMEOUT;
		m_xfer_done = false;

    return err_code;
}

uint32_t nrf_drv_mpu_write_single_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    uint8_t packet[2] = {reg, data};

    err_code = nrf_drv_twi_tx(&m_twi, MPU_ADDRESS, packet, 2, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!m_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    m_xfer_done = false;

    return err_code;
}

uint32_t nrf_drv_mpu_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, MPU_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!m_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    m_xfer_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, MPU_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = MPU_TWI_TIMEOUT;
    while((!m_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    m_xfer_done = false;

    return err_code;
}

uint32_t app_mpu_init(void)
{
    uint32_t err_code;
	
		// Initate TWI or SPI driver dependent on what is defined from the project
		twi_init();

    uint8_t reset_value = 7; // Resets gyro, accelerometer and temperature sensor signal paths.
    err_code = nrf_drv_mpu_write_single_register(MPU_REG_SIGNAL_PATH_RESET, reset_value);
    if(err_code != NRF_SUCCESS) return err_code;

    // Chose  PLL with X axis gyroscope reference as clock source
    err_code = nrf_drv_mpu_write_single_register(MPU_REG_PWR_MGMT_1, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}

uint32_t app_mpu_accel_only_mode(void)
{

		uint32_t err_code = nrf_drv_mpu_write_single_register(MPU_REG_PWR_MGMT_1, 0x21); //Turn off Gyro
    if(err_code != NRF_SUCCESS)
		   return err_code;
		
		err_code = nrf_drv_mpu_write_single_register(MPU_REG_PWR_MGMT_2, 0x07); //Turn off Gyro
    if(err_code != NRF_SUCCESS)
		   return err_code;
		
		err_code = nrf_drv_mpu_write_single_register(MPU_REG_LP_ACCEL_ODR, 0x01); //Turn off Gyro
    if(err_code != NRF_SUCCESS)
		   return err_code;

    return NRF_SUCCESS;

}

uint32_t app_mpu_read_accel(accel_values_t * accel_values)
{
    uint32_t err_code;
    uint8_t raw_values[6];
    err_code = nrf_drv_mpu_read_registers(MPU_REG_ACCEL_XOUT_H, raw_values, 6);
    if(err_code != NRF_SUCCESS) return err_code;

    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}

uint32_t app_mpu_config(app_mpu_config_t * config)
{
    uint8_t *data;
    data = (uint8_t*)config;
    return nrf_drv_mpu_write_registers(MPU_REG_SMPLRT_DIV, data, 4);
}

void mpu_ppi_chn_config(void)
{
		ret_code_t ret;
		
   		twi_tx_buffer[0] = MPU_REG_ACCEL_XOUT_H;
//		
//		NRF_TWIM0->TXD.MAXCNT = 1;
//    NRF_TWIM0->RXD.MAXCNT = 6;
//    NRF_TWIM0->TXD.LIST =	1;
//    NRF_SPIM0->TXD.PTR = (uint32_t)&twi_tx_buffer;
//    NRF_TWIM0->RXD.LIST =	1;
//    NRF_TWIM0->RXD.PTR = (uint32_t)&twi_rx_buffer;
	
		nrf_drv_twi_xfer_desc_t xfer_desc;
		xfer_desc.address = MPU_ADDRESS;
		xfer_desc.primary_length = 1;
		xfer_desc.p_primary_buf = twi_tx_buffer;
		xfer_desc.p_secondary_buf = twi_rx_buffer;
		xfer_desc.secondary_length = 6;
		xfer_desc.type = NRF_DRV_TWI_XFER_TXRX;
		
		nrf_drv_twi_xfer(&m_twi, &xfer_desc, NRF_DRV_TWI_FLAG_HOLD_XFER);
	
		ret = nrf_drv_ppi_channel_fork_assign(spi_end_transfer_channel, nrf_drv_twi_start_task_get(&m_twi, NRF_DRV_TWI_XFER_TXRX));
		APP_ERROR_CHECK(ret);

}

void get_data_three_chn(int16_t* data)
{
	ret_code_t ret;
	
	int16_t val = 0;
	
	nrf_queue_pop(&m_accx_queue, &val);
	data[0] = val;
	
	nrf_queue_pop(&m_accy_queue, &val);
	data[1] = val;
	
	nrf_queue_pop(&m_accz_queue, &val);
	data[2] = val;
	
}
