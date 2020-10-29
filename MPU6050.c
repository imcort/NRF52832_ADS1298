#include "mpu6050.h"
#include "nrf_drv_twi.h"

#define MPU_TWI_TIMEOUT 					10000 
#define MPU_TWI_BUFFER_SIZE     	14
#define MPU_ADDRESS     					0x68

#define TWI_INSTANCE_ID     1
static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];



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
