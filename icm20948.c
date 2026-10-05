#include "icm20948.h"
#include "icm20948_registers.h"
#include "ak09916_registers.h"

/*
 * Icm20948 device require a DMP image to be loaded on init
 * Provide such images by mean of a byte array
*/
#if CONFIG_ICM_20948_USE_DMP

#if defined(ARDUINO_ARCH_MBED) // ARDUINO_ARCH_MBED (APOLLO3 v2) does not support or require pgmspace.h / PROGMEM
const uint8_t dmp3_image[] = {
#elif (defined(__AVR__) || defined(__arm__) || defined(__ARDUINO_ARC__) || defined(ESP8266)) && !defined(__linux__) // Store the DMP firmware in PROGMEM on older AVR (ATmega) platforms
#define ICM_20948_USE_PROGMEM_FOR_DMP
#include <avr/pgmspace.h>
const uint8_t dmp3_image[] PROGMEM = {
#else
const uint8_t dmp3_image[] = {
#endif

#include "icm20948_img.dmp3a.h"
};
#endif

// ICM-20948 data is big-endian. We need to make it little-endian when writing into icm_20948_DMP_data_t
const int DMP_Quat9_Byte_Ordering[ICM_20948_DMP_QUAT9_BYTES] =
    {
        3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 13, 12 // Also used for Geomag
};
const int DMP_Quat6_Byte_Ordering[ICM_20948_DMP_QUAT6_BYTES] =
    {
        3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8 // Also used for Gyro_Calibr, Compass_Calibr
};
const int DMP_PQuat6_Byte_Ordering[ICM_20948_DMP_PQUAT6_BYTES] =
    {
        1, 0, 3, 2, 5, 4 // Also used for Raw_Accel, Compass
};
const int DMP_Raw_Gyro_Byte_Ordering[ICM_20948_DMP_RAW_GYRO_BYTES + ICM_20948_DMP_GYRO_BIAS_BYTES] =
    {
        1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10};
const int DMP_Activity_Recognition_Byte_Ordering[ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES] =
    {
        0, 1, 5, 4, 3, 2};
const int DMP_Secondary_On_Off_Byte_Ordering[ICM_20948_DMP_SECONDARY_ON_OFF_BYTES] =
    {
        1, 0};

const uint16_t inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX] =
    {
        // Data output control 1 register bit definition
        // 16-bit accel                                0x8000
        // 16-bit gyro                                 0x4000
        // 16-bit compass                              0x2000
        // 16-bit ALS                                  0x1000
        // 32-bit 6-axis quaternion                    0x0800
        // 32-bit 9-axis quaternion + heading accuracy 0x0400
        // 16-bit pedometer quaternion                 0x0200
        // 32-bit Geomag rv + heading accuracy         0x0100
        // 16-bit Pressure                             0x0080
        // 32-bit calibrated gyro                      0x0040
        // 32-bit calibrated compass                   0x0020
        // Pedometer Step Detector                     0x0010
        // Header 2                                    0x0008
        // Pedometer Step Indicator Bit 2              0x0004
        // Pedometer Step Indicator Bit 1              0x0002
        // Pedometer Step Indicator Bit 0              0x0001
        // Unsupported Sensors are 0xFFFF

        0xFFFF, // 0  Meta Data
        0x8008, // 1  Accelerometer
        0x0028, // 2  Magnetic Field
        0x0408, // 3  Orientation
        0x4048, // 4  Gyroscope
        0x1008, // 5  Light
        0x0088, // 6  Pressure
        0xFFFF, // 7  Temperature
        0xFFFF, // 8  Proximity <----------- fixme
        0x0808, // 9  Gravity
        0x8808, // 10 Linear Acceleration
        0x0408, // 11 Rotation Vector
        0xFFFF, // 12 Humidity
        0xFFFF, // 13 Ambient Temperature
        0x2008, // 14 Magnetic Field Uncalibrated
        0x0808, // 15 Game Rotation Vector
        0x4008, // 16 Gyroscope Uncalibrated
        0x0000, // 17 Significant Motion
        0x0018, // 18 Step Detector
        0x0010, // 19 Step Counter <----------- fixme
        0x0108, // 20 Geomagnetic Rotation Vector
        0xFFFF, // 21 ANDROID_SENSOR_HEART_RATE,
        0xFFFF, // 22 ANDROID_SENSOR_PROXIMITY,

        0x8008, // 23 ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
        0x0028, // 24 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
        0x0408, // 25 ANDROID_SENSOR_WAKEUP_ORIENTATION,
        0x4048, // 26 ANDROID_SENSOR_WAKEUP_GYROSCOPE,
        0x1008, // 27 ANDROID_SENSOR_WAKEUP_LIGHT,
        0x0088, // 28 ANDROID_SENSOR_WAKEUP_PRESSURE,
        0x0808, // 29 ANDROID_SENSOR_WAKEUP_GRAVITY,
        0x8808, // 30 ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
        0x0408, // 31 ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
        0xFFFF, // 32 ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
        0xFFFF, // 33 ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
        0x2008, // 34 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
        0x0808, // 35 ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
        0x4008, // 36 ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
        0x0018, // 37 ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
        0x0010, // 38 ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
        0x0108, // 39 ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
        0xFFFF, // 40 ANDROID_SENSOR_WAKEUP_HEART_RATE,
        0x0000, // 41 ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
        0x8008, // 42 Raw Acc
        0x4048, // 43 Raw Gyr
};

const icm20948_serif_t NullSerif = {
    NULL, // write
    NULL, // read
    NULL, // user
};

// Private function prototypes

// Function definitions
icm20948_status_e icm20948_init_struct(icm20948_device_t *pdev)
{
  // Initialize all elements by 0 except for _last_bank
  // Initialize _last_bank to 4 (invalid bank number)
  // so icm20948_set_bank function does not skip issuing bank change operation
  static const icm20948_device_t init_device = { ._last_bank = 4 };
  *pdev = init_device;
  return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_link_serif(icm20948_device_t *pdev, const icm20948_serif_t *s)
{
  if (s == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }
  if (pdev == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }
  pdev->_serif = s;
  return ICM_20948_STAT_OK;
}

icm20948_status_e icm20948_execute_w(icm20948_device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len)
{
  if (pdev->_serif->write == NULL)
  {
    return ICM_20948_STAT_NOT_IMPL;
  }
  return (*pdev->_serif->write)(regaddr, pdata, len, pdev->_serif->user);
}

icm20948_status_e icm20948_execute_r(icm20948_device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len)
{
  if (pdev->_serif->read == NULL)
  {
    return ICM_20948_STAT_NOT_IMPL;
  }
  return (*pdev->_serif->read)(regaddr, pdata, len, pdev->_serif->user);
}

//Transact directly with an I2C device, one byte at a time
//Used to configure a device before it is setup into a normal 0-3 peripheral slot
icm20948_status_e icm20948_i2c_controller_periph4_txn(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  // Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86
  icm20948_status_e retval = ICM_20948_STAT_OK;

  addr = (((Rw) ? 0x80 : 0x00) | addr);

  retval = icm20948_set_bank(pdev, 3);
  retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_ADDR, (uint8_t *)&addr, 1);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_set_bank(pdev, 3);
  retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_REG, (uint8_t *)&reg, 1);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  icm20948_i2c_periph4_ctrl_t ctrl;
  ctrl.EN = 1;
  ctrl.INT_EN = false;
  ctrl.DLY = 0;
  ctrl.REG_DIS = !send_reg_addr;

  icm20948_i2c_mst_status_t i2c_mst_status;
  bool txn_failed = false;
  uint16_t nByte = 0;

  while (nByte < len)
  {
    if (!Rw)
    {
      retval = icm20948_set_bank(pdev, 3);
      retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_DO, (uint8_t *)&(data[nByte]), 1);
      if (retval != ICM_20948_STAT_OK)
      {
        return retval;
      }
    }

    // Kick off txn
    retval = icm20948_set_bank(pdev, 3);
    retval = icm20948_execute_w(pdev, AGB3_REG_I2C_PERIPH4_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_i2c_periph4_ctrl_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    // long tsTimeout = millis() + 3000;  // Emergency timeout for txn (hard coded to 3 secs)
    uint32_t max_cycles = 1000;
    uint32_t count = 0;
    bool peripheral4Done = false;
    while (!peripheral4Done)
    {
      retval = icm20948_set_bank(pdev, 0);
      retval = icm20948_execute_r(pdev, AGB0_REG_I2C_MST_STATUS, (uint8_t *)&i2c_mst_status, 1);

      peripheral4Done = (i2c_mst_status.I2C_PERIPH4_DONE /*| (millis() > tsTimeout) */); //Avoid forever-loops
      peripheral4Done |= (count >= max_cycles);
      count++;
    }
    txn_failed = (i2c_mst_status.I2C_PERIPH4_NACK /*| (millis() > tsTimeout) */);
    txn_failed |= (count >= max_cycles);
    if (txn_failed)
      break;

    if (Rw)
    {
      retval = icm20948_set_bank(pdev, 3);
      retval = icm20948_execute_r(pdev, AGB3_REG_I2C_PERIPH4_DI, &data[nByte], 1);
    }

    nByte++;
  }

  if (txn_failed)
  {
    //We often fail here if mag is stuck
    return ICM_20948_STAT_ERR;
  }

  return retval;
}

icm20948_status_e icm20948_i2c_master_single_w(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data)
{
  return icm20948_i2c_controller_periph4_txn(pdev, addr, reg, data, 1, false, true);
}

icm20948_status_e icm20948_i2c_master_single_r(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data)
{
  return icm20948_i2c_controller_periph4_txn(pdev, addr, reg, data, 1, true, true);
}

icm20948_status_e icm20948_set_bank(icm20948_device_t *pdev, uint8_t bank)
{
  if (bank > 3)
  {
    return ICM_20948_STAT_PARAM_ERR;
  } // Only 4 possible banks

  if (bank == pdev->_last_bank) // Do we need to change bank?
    return ICM_20948_STAT_OK;   // Bail if we don't need to change bank to avoid unnecessary bus traffic

  pdev->_last_bank = bank;   // Store the requested bank (before we bit-shift)
  bank = (bank << 4) & 0x30; // bits 5:4 of REG_BANK_SEL
  return icm20948_execute_w(pdev, REG_BANK_SEL, &bank, 1);
}

icm20948_status_e icm20948_sw_reset(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  reg.DEVICE_RESET = 1;

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  pdev->_last_bank = 0;
  pdev->_last_mems_bank = 255;
  pdev->_firmware_loaded = false;
  return retval;
}

icm20948_status_e icm20948_sleep(icm20948_device_t *pdev, bool on)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (on)
  {
    reg.SLEEP = 1;
  }
  else
  {
    reg.SLEEP = 0;
  }

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_low_power(icm20948_device_t *pdev, bool on)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (on)
  {
    reg.LP_EN = 1;
  }
  else
  {
    reg.LP_EN = 0;
  }

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_set_clock_source(icm20948_device_t *pdev, icm20948_pwr_mgmt_1_clksel_e source)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_pwr_mgmt_1_t reg;

  icm20948_set_bank(pdev, 0); // Must be in the right bank

  retval = icm20948_execute_r(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  reg.CLKSEL = source;

  retval = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_1, (uint8_t *)&reg, sizeof(icm20948_pwr_mgmt_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_get_who_am_i(icm20948_device_t *pdev, uint8_t *whoami)
{
  if (whoami == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }
  icm20948_set_bank(pdev, 0); // Must be in the right bank
  return icm20948_execute_r(pdev, AGB0_REG_WHO_AM_I, whoami, 1);
}

icm20948_status_e icm20948_check_id(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  uint8_t whoami = 0x00;
  retval = icm20948_get_who_am_i(pdev, &whoami);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (whoami != ICM_20948_WHOAMI)
  {
    return ICM_20948_STAT_WRONG_ID;
  }
  return retval;
}

icm20948_status_e icm20948_data_ready(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_int_status_1_t reg;
  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_INT_STATUS_1, (uint8_t *)&reg, sizeof(icm20948_int_status_1_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (!reg.RAW_DATA_0_RDY_INT)
  {
    retval = ICM_20948_STAT_NO_DATA;
  }
  return retval;
}

// Interrupt Configuration
icm20948_status_e icm20948_int_pin_cfg(icm20948_device_t *pdev, icm20948_int_pin_cfg_t *write, icm20948_int_pin_cfg_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank
  if (write != NULL)
  { // write first, if available
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)write, sizeof(icm20948_int_pin_cfg_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }
  if (read != NULL)
  { // then read, to allow for verification
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)read, sizeof(icm20948_int_pin_cfg_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }
  return retval;
}

icm20948_status_e icm20948_int_enable(icm20948_device_t *pdev, icm20948_int_enable_t *write, icm20948_int_enable_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_int_enable_0_t en_0;
  icm20948_int_enable_1_t en_1;
  icm20948_int_enable_2_t en_2;
  icm20948_int_enable_3_t en_3;

  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    en_0.I2C_MST_INT_EN = write->I2C_MST_INT_EN;
    en_0.DMP_INT1_EN = write->DMP_INT1_EN;
    en_0.PLL_READY_EN = write->PLL_RDY_EN;
    en_0.WOM_INT_EN = write->WOM_INT_EN;
    en_0.reserved_0 = 0; // Clear RAM garbage
    en_0.REG_WOF_EN = write->REG_WOF_EN;
    en_1.RAW_DATA_0_RDY_EN = write->RAW_DATA_0_RDY_EN;
    en_1.reserved_0 = 0; // Clear RAM garbage
    en_2.individual.FIFO_OVERFLOW_EN_4 = write->FIFO_OVERFLOW_EN_4;
    en_2.individual.FIFO_OVERFLOW_EN_3 = write->FIFO_OVERFLOW_EN_3;
    en_2.individual.FIFO_OVERFLOW_EN_2 = write->FIFO_OVERFLOW_EN_2;
    en_2.individual.FIFO_OVERFLOW_EN_1 = write->FIFO_OVERFLOW_EN_1;
    en_2.individual.FIFO_OVERFLOW_EN_0 = write->FIFO_OVERFLOW_EN_0;
    en_2.individual.reserved_0 = 0; // Clear RAM garbage
    en_3.individual.FIFO_WM_EN_4 = write->FIFO_WM_EN_4;
    en_3.individual.FIFO_WM_EN_3 = write->FIFO_WM_EN_3;
    en_3.individual.FIFO_WM_EN_2 = write->FIFO_WM_EN_2;
    en_3.individual.FIFO_WM_EN_1 = write->FIFO_WM_EN_1;
    en_3.individual.FIFO_WM_EN_0 = write->FIFO_WM_EN_0;
    en_3.individual.reserved_0 = 0; // Clear RAM garbage

    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE, (uint8_t *)&en_0, sizeof(icm20948_int_enable_0_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE_1, (uint8_t *)&en_1, sizeof(icm20948_int_enable_1_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE_2, (uint8_t *)&en_2, sizeof(icm20948_int_enable_2_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_w(pdev, AGB0_REG_INT_ENABLE_3, (uint8_t *)&en_3, sizeof(icm20948_int_enable_3_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE, (uint8_t *)&en_0, sizeof(icm20948_int_enable_0_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE_1, (uint8_t *)&en_1, sizeof(icm20948_int_enable_1_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE_2, (uint8_t *)&en_2, sizeof(icm20948_int_enable_2_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
    retval = icm20948_execute_r(pdev, AGB0_REG_INT_ENABLE_3, (uint8_t *)&en_3, sizeof(icm20948_int_enable_3_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    read->I2C_MST_INT_EN = en_0.I2C_MST_INT_EN;
    read->DMP_INT1_EN = en_0.DMP_INT1_EN;
    read->PLL_RDY_EN = en_0.PLL_READY_EN;
    read->WOM_INT_EN = en_0.WOM_INT_EN;
    read->REG_WOF_EN = en_0.REG_WOF_EN;
    read->RAW_DATA_0_RDY_EN = en_1.RAW_DATA_0_RDY_EN;
    read->FIFO_OVERFLOW_EN_4 = en_2.individual.FIFO_OVERFLOW_EN_4;
    read->FIFO_OVERFLOW_EN_3 = en_2.individual.FIFO_OVERFLOW_EN_3;
    read->FIFO_OVERFLOW_EN_2 = en_2.individual.FIFO_OVERFLOW_EN_2;
    read->FIFO_OVERFLOW_EN_1 = en_2.individual.FIFO_OVERFLOW_EN_1;
    read->FIFO_OVERFLOW_EN_0 = en_2.individual.FIFO_OVERFLOW_EN_0;
    read->FIFO_WM_EN_4 = en_3.individual.FIFO_WM_EN_4;
    read->FIFO_WM_EN_3 = en_3.individual.FIFO_WM_EN_3;
    read->FIFO_WM_EN_2 = en_3.individual.FIFO_WM_EN_2;
    read->FIFO_WM_EN_1 = en_3.individual.FIFO_WM_EN_1;
    read->FIFO_WM_EN_0 = en_3.individual.FIFO_WM_EN_0;
  }

  return retval;
}

icm20948_status_e icm20948_wom_logic(icm20948_device_t *pdev, icm20948_accel_intel_ctrl_t *write, icm20948_accel_intel_ctrl_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_accel_intel_ctrl_t ctrl;

  retval = icm20948_set_bank(pdev, 2); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    ctrl.ACCEL_INTEL_EN = write->ACCEL_INTEL_EN;
    ctrl.ACCEL_INTEL_MODE_INT = write->ACCEL_INTEL_MODE_INT;

    retval = icm20948_execute_w(pdev, AGB2_REG_ACCEL_INTEL_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_accel_intel_ctrl_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = icm20948_execute_r(pdev, AGB2_REG_ACCEL_INTEL_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_accel_intel_ctrl_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    read->ACCEL_INTEL_EN = ctrl.ACCEL_INTEL_EN;
    read->ACCEL_INTEL_MODE_INT = ctrl.ACCEL_INTEL_MODE_INT;
  }

  return retval;
}

icm20948_status_e icm20948_wom_threshold(icm20948_device_t *pdev, icm20948_accel_wom_thr_t *write, icm20948_accel_wom_thr_t *read)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_accel_wom_thr_t thr;

  retval = icm20948_set_bank(pdev, 2); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    thr.WOM_THRESHOLD = write->WOM_THRESHOLD;

    retval = icm20948_execute_w(pdev, AGB2_REG_ACCEL_WOM_THR, (uint8_t *)&thr, sizeof(icm20948_accel_wom_thr_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    retval = icm20948_execute_r(pdev, AGB2_REG_ACCEL_WOM_THR, (uint8_t *)&thr, sizeof(icm20948_accel_wom_thr_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }

    read->WOM_THRESHOLD = thr.WOM_THRESHOLD;
  }

  return retval;
}

icm20948_status_e icm20948_set_sample_mode(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_lp_config_cycle_e mode)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;
  icm20948_lp_config_t reg;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR | ICM_20948_INTERNAL_MST)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  retval = icm20948_set_bank(pdev, 0); // Must be in the right bank
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(icm20948_lp_config_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    reg.ACCEL_CYCLE = mode;
  } // Set all desired sensors to this setting
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    reg.GYRO_CYCLE = mode;
  }
  if (sensors & ICM_20948_INTERNAL_MST)
  {
    reg.I2C_MST_CYCLE = mode;
  }

  retval = icm20948_execute_w(pdev, AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(icm20948_lp_config_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // Check the data was written correctly
  retval = icm20948_execute_r(pdev, AGB0_REG_LP_CONFIG, (uint8_t *)&reg, sizeof(icm20948_lp_config_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    if (reg.ACCEL_CYCLE != mode) retval = ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    if (reg.GYRO_CYCLE != mode) retval = ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_MST)
  {
    if (reg.I2C_MST_CYCLE != mode) retval = ICM_20948_STAT_ERR;
  }

  return retval;
}

icm20948_status_e icm20948_set_full_scale(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_fss_t fss)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    icm20948_accel_config_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    reg.ACCEL_FS_SEL = fss.a;
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (reg.ACCEL_FS_SEL != fss.a) retval |= ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    icm20948_gyro_config_1_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    reg.GYRO_FS_SEL = fss.g;
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (reg.GYRO_FS_SEL != fss.g) retval |= ICM_20948_STAT_ERR;
  }
  return retval;
}

icm20948_status_e icm20948_set_dlpf_cfg(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_dlpcfg_t cfg)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    icm20948_accel_config_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    reg.ACCEL_DLPFCFG = cfg.a;
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (reg.ACCEL_DLPFCFG != cfg.a) retval |= ICM_20948_STAT_ERR;
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    icm20948_gyro_config_1_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    reg.GYRO_DLPFCFG = cfg.g;
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (reg.GYRO_DLPFCFG != cfg.g) retval |= ICM_20948_STAT_ERR;
  }
  return retval;
}

icm20948_status_e icm20948_enable_dlpf(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    icm20948_accel_config_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (enable)
    {
      reg.ACCEL_FCHOICE = 1;
    }
    else
    {
      reg.ACCEL_FCHOICE = 0;
    }
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(icm20948_accel_config_t));
    if (enable)
    {
      if (reg.ACCEL_FCHOICE != 1) retval |= ICM_20948_STAT_ERR;
    }
    else
    {
      if (reg.ACCEL_FCHOICE != 0) retval |= ICM_20948_STAT_ERR;
    }
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    icm20948_gyro_config_1_t reg;
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (enable)
    {
      reg.GYRO_FCHOICE = 1;
    }
    else
    {
      reg.GYRO_FCHOICE = 0;
    }
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    // Check the data was written correctly
    retval |= icm20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(icm20948_gyro_config_1_t));
    if (enable)
    {
      if (reg.GYRO_FCHOICE != 1) retval |= ICM_20948_STAT_ERR;
    }
    else
    {
      if (reg.GYRO_FCHOICE != 0) retval |= ICM_20948_STAT_ERR;
    }
  }
  return retval;
}

icm20948_status_e icm20948_set_sample_rate(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_smplrt_t smplrt)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  if (!(sensors & (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR)))
  {
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED;
  }

  if (sensors & ICM_20948_INTERNAL_ACC)
  {
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    uint8_t div1 = (smplrt.a >> 8); // Thank you @yanivamichy #109
    uint8_t div2 = (smplrt.a & 0xFF);
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_SMPLRT_DIV_1, &div1, 1);
    retval |= icm20948_execute_w(pdev, AGB2_REG_ACCEL_SMPLRT_DIV_2, &div2, 1);
  }
  if (sensors & ICM_20948_INTERNAL_GYR)
  {
    retval |= icm20948_set_bank(pdev, 2); // Must be in the right bank
    uint8_t div = (smplrt.g);
    retval |= icm20948_execute_w(pdev, AGB2_REG_GYRO_SMPLRT_DIV, &div, 1);
  }
  return retval;
}

// Interface Things
icm20948_status_e icm20948_i2c_master_passthrough(icm20948_device_t *pdev, bool passthrough)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_int_pin_cfg_t reg;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(icm20948_int_pin_cfg_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  reg.BYPASS_EN = passthrough;
  retval = icm20948_execute_w(pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t *)&reg, sizeof(icm20948_int_pin_cfg_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_i2c_master_enable(icm20948_device_t *pdev, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  // Disable BYPASS_EN
  retval = icm20948_i2c_master_passthrough(pdev, false);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  icm20948_i2c_mst_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 3);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB3_REG_I2C_MST_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_i2c_mst_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  ctrl.I2C_MST_CLK = 0x07; // corresponds to 345.6 kHz, good for up to 400 kHz
  ctrl.I2C_MST_P_NSR = 1;
  retval = icm20948_execute_w(pdev, AGB3_REG_I2C_MST_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_i2c_mst_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  icm20948_user_ctrl_t reg;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&reg, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  if (enable)
  {
    reg.I2C_MST_EN = 1;
  }
  else
  {
    reg.I2C_MST_EN = 0;
  }
  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&reg, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_i2c_master_reset(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_user_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrl.I2C_MST_RST = 1; //Reset!

  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_i2c_controller_configure_peripheral(icm20948_device_t *pdev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  uint8_t periph_addr_reg;
  uint8_t periph_reg_reg;
  uint8_t periph_ctrl_reg;
  uint8_t periph_do_reg;

  switch (peripheral)
  {
  case 0:
    periph_addr_reg = AGB3_REG_I2C_PERIPH0_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH0_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH0_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH0_DO;
    break;
  case 1:
    periph_addr_reg = AGB3_REG_I2C_PERIPH1_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH1_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH1_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH1_DO;
    break;
  case 2:
    periph_addr_reg = AGB3_REG_I2C_PERIPH2_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH2_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH2_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH2_DO;
    break;
  case 3:
    periph_addr_reg = AGB3_REG_I2C_PERIPH3_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH3_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH3_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH3_DO;
    break;
  default:
    return ICM_20948_STAT_PARAM_ERR;
  }

  retval = icm20948_set_bank(pdev, 3);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // Set the peripheral address and the Rw flag
  icm20948_i2c_periphx_addr_t address;
  address.ID = addr;
  if (Rw)
  {
    address.RNW = 1;
  }
  else
  {
    address.RNW = 0; // Make sure bit is clear (just in case there is any garbage in that RAM location)
  }
  retval = icm20948_execute_w(pdev, periph_addr_reg, (uint8_t *)&address, sizeof(icm20948_i2c_periphx_addr_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    icm20948_i2c_periphx_do_t dataOutByte;
    dataOutByte.DO = dataOut;
    retval = icm20948_execute_w(pdev, periph_do_reg, (uint8_t *)&dataOutByte, sizeof(icm20948_i2c_periphx_do_t));
    if (retval != ICM_20948_STAT_OK)
    {
      return retval;
    }
  }

  // Set the peripheral sub-address (register address)
  icm20948_i2c_periphx_reg_t subaddress;
  subaddress.REG = reg;
  retval = icm20948_execute_w(pdev, periph_reg_reg, (uint8_t *)&subaddress, sizeof(icm20948_i2c_periphx_reg_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  // Set up the control info
  icm20948_i2c_periphx_ctrl_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  retval = icm20948_execute_w(pdev, periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(icm20948_i2c_periphx_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_configure_magnetometer(icm20948_device_t *pdev, ak09916_mode_e mode)
{
  if (mode != AK09916_MODE_CONT_10_HZ && mode != AK09916_MODE_CONT_20_HZ &&
      mode != AK09916_MODE_CONT_50_HZ && mode != AK09916_MODE_CONT_100_HZ)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }

  uint8_t mode_register = mode;
  icm20948_status_e retval = icm20948_i2c_master_enable(pdev, true);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_i2c_master_single_w(pdev, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, &mode_register);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return icm20948_i2c_controller_configure_peripheral(pdev, 0, MAG_AK09916_I2C_ADDR,
                                                        AK09916_REG_ST1, 9, true, true,
                                                        false, false, false, 0);
}

// Higher Level
icm20948_status_e icm20948_get_agmt(icm20948_device_t *pdev, icm20948_agmt_t *pagmt)
{
  if (pagmt == NULL)
  {
    return ICM_20948_STAT_PARAM_ERR;
  }

  icm20948_status_e retval = ICM_20948_STAT_OK;
  const uint8_t numbytes = 14 + 9; //Read Accel, gyro, temp, and 9 bytes of mag
  uint8_t buff[numbytes];

  // Get readings
  retval |= icm20948_set_bank(pdev, 0);
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB0_REG_ACCEL_XOUT_H, buff, numbytes);

  pagmt->acc.axes.x = ((buff[0] << 8) | (buff[1] & 0xFF));
  pagmt->acc.axes.y = ((buff[2] << 8) | (buff[3] & 0xFF));
  pagmt->acc.axes.z = ((buff[4] << 8) | (buff[5] & 0xFF));

  pagmt->gyr.axes.x = ((buff[6] << 8) | (buff[7] & 0xFF));
  pagmt->gyr.axes.y = ((buff[8] << 8) | (buff[9] & 0xFF));
  pagmt->gyr.axes.z = ((buff[10] << 8) | (buff[11] & 0xFF));

  pagmt->tmp.val = ((buff[12] << 8) | (buff[13] & 0xFF));

  pagmt->magStat1 = buff[14];
  pagmt->mag.axes.x = ((buff[16] << 8) | (buff[15] & 0xFF)); //Mag data is read little endian
  pagmt->mag.axes.y = ((buff[18] << 8) | (buff[17] & 0xFF));
  pagmt->mag.axes.z = ((buff[20] << 8) | (buff[19] & 0xFF));
  pagmt->magStat2 = buff[22];

  // Get settings to be able to compute scaled values
  retval |= icm20948_set_bank(pdev, 2);
  icm20948_accel_config_t acfg;
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG, (uint8_t *)&acfg, 1 * sizeof(acfg));
  pagmt->fss.a = acfg.ACCEL_FS_SEL; // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range
                                    // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
  retval |= icm20948_set_bank(pdev, 2);
  icm20948_gyro_config_1_t gcfg1;
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&gcfg1, 1 * sizeof(gcfg1));
  pagmt->fss.g = gcfg1.GYRO_FS_SEL;
  icm20948_accel_config_2_t acfg2;
  retval |= icm20948_execute_r(pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG_2, (uint8_t *)&acfg2, 1 * sizeof(acfg2));

  return retval;
}

// FIFO

icm20948_status_e icm20948_enable_fifo(icm20948_device_t *pdev, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_user_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (enable)
    ctrl.FIFO_EN = 1;
  else
    ctrl.FIFO_EN = 0;

  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_reset_fifo(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_fifo_rst_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(icm20948_fifo_rst_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrl.FIFO_RESET = 0x1F; // Datasheet says "FIFO_RESET[4:0]"

  retval = icm20948_execute_w(pdev, AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(icm20948_fifo_rst_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  //delay ???

  ctrl.FIFO_RESET = 0x1E; // The InvenSense Nucleo examples write 0x1F followed by 0x1E

  retval = icm20948_execute_w(pdev, AGB0_REG_FIFO_RST, (uint8_t *)&ctrl, sizeof(icm20948_fifo_rst_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

icm20948_status_e icm20948_set_fifo_mode(icm20948_device_t *pdev, bool snapshot)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_fifo_mode_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(icm20948_fifo_mode_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (snapshot)
    ctrl.FIFO_MODE = 0x1F; // Datasheet says "FIFO_MODE[4:0]"
  else
    ctrl.FIFO_MODE = 0;

  retval = icm20948_execute_w(pdev, AGB0_REG_FIFO_MODE, (uint8_t *)&ctrl, sizeof(icm20948_fifo_mode_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_get_fifo_count(icm20948_device_t *pdev, uint16_t *count)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_fifo_counth_t ctrlh;
  icm20948_fifo_countl_t ctrll;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_COUNT_H, (uint8_t *)&ctrlh, sizeof(icm20948_fifo_counth_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrlh.FIFO_COUNTH &= 0x1F; // Datasheet says "FIFO_CNT[12:8]"

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_COUNT_L, (uint8_t *)&ctrll, sizeof(icm20948_fifo_countl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  *count = (((uint16_t)ctrlh.FIFO_COUNTH) << 8) | (uint16_t)ctrll.FIFO_COUNTL;

  return retval;
}

icm20948_status_e icm20948_read_fifo(icm20948_device_t *pdev, uint8_t *data, uint8_t len)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_FIFO_R_W, data, len);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  return retval;
}

// DMP

icm20948_status_e icm20948_enable_dmp(icm20948_device_t *pdev, bool enable)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_user_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  if (enable)
    ctrl.DMP_EN = 1;
  else
    ctrl.DMP_EN = 0;

  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_reset_dmp(icm20948_device_t *pdev)
{
  icm20948_status_e retval = ICM_20948_STAT_OK;

  icm20948_user_ctrl_t ctrl;
  retval = icm20948_set_bank(pdev, 0);
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  retval = icm20948_execute_r(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }

  ctrl.DMP_RST = 1;

  retval = icm20948_execute_w(pdev, AGB0_REG_USER_CTRL, (uint8_t *)&ctrl, sizeof(icm20948_user_ctrl_t));
  if (retval != ICM_20948_STAT_OK)
  {
    return retval;
  }
  return retval;
}

icm20948_status_e icm20948_firmware_load(icm20948_device_t *pdev)
{
#if CONFIG_ICM_20948_USE_DMP
    return (inv_icm20948_firmware_load(pdev, dmp3_image, sizeof(dmp3_image), DMP_LOAD_START));
#else
    return ICM_20948_STAT_DMP_NOT_SUPPORTED;
#endif
}

/** @brief Loads the DMP firmware from SRAM
* @param[in] data  pointer where the image
* @param[in] size  size if the image
* @param[in] load_addr  address to loading the image
* @return 0 in case of success, -1 for any error
*/
icm20948_status_e inv_icm20948_firmware_load(icm20948_device_t *pdev, const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{
  int write_size;
  icm20948_status_e result = ICM_20948_STAT_OK;
  unsigned short memaddr;
  const unsigned char *data;
  unsigned short size;
  unsigned char data_cmp[INV_MAX_SERIAL_READ];
  int flag = 0;

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED;

  if (pdev->_firmware_loaded)
    return ICM_20948_STAT_OK; // Bail with no error if firmware is already loaded

  result = icm20948_sleep(pdev, false); // Make sure chip is awake
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  result = icm20948_low_power(pdev, false); // Make sure chip is not in low power state
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  // Write DMP memory

  data = data_start;
  size = size_start;
  memaddr = load_addr;
  #ifdef ICM_20948_USE_PROGMEM_FOR_DMP
  unsigned char data_not_pg[INV_MAX_SERIAL_READ]; // Suggested by @HyperKokichi in Issue #63
  #endif
  while (size > 0)
  {
    //write_size = min(size, INV_MAX_SERIAL_WRITE); // Write in chunks of INV_MAX_SERIAL_WRITE
    if (size <= INV_MAX_SERIAL_WRITE) // Write in chunks of INV_MAX_SERIAL_WRITE
      write_size = size;
    else
      write_size = INV_MAX_SERIAL_WRITE;
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      // Moved across a bank
      write_size = (memaddr & 0xff) + write_size - 0x100;
    }
#ifdef ICM_20948_USE_PROGMEM_FOR_DMP
    memcpy_P(data_not_pg, data, write_size);  // Suggested by @HyperKokichi in Issue #63
    result = inv_icm20948_write_mems(pdev, memaddr, write_size, (unsigned char *)data_not_pg);
#else
    result = inv_icm20948_write_mems(pdev, memaddr, write_size, (unsigned char *)data);
#endif
    if (result != ICM_20948_STAT_OK)
      return result;
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }

  // Verify DMP memory

  data = data_start;
  size = size_start;
  memaddr = load_addr;
  while (size > 0)
  {
    //write_size = min(size, INV_MAX_SERIAL_READ); // Read in chunks of INV_MAX_SERIAL_READ
    if (size <= INV_MAX_SERIAL_READ) // Read in chunks of INV_MAX_SERIAL_READ
      write_size = size;
    else
      write_size = INV_MAX_SERIAL_READ;
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      // Moved across a bank
      write_size = (memaddr & 0xff) + write_size - 0x100;
    }
    result = inv_icm20948_read_mems(pdev, memaddr, write_size, data_cmp);
    if (result != ICM_20948_STAT_OK)
      flag++;                               // Error, DMP not written correctly
#ifdef ICM_20948_USE_PROGMEM_FOR_DMP
    memcpy_P(data_not_pg, data, write_size);  // Suggested by @HyperKokichi in Issue #63
    if (memcmp(data_cmp, data_not_pg, write_size))
#else
    if (memcmp(data_cmp, data, write_size)) // Compare the data
#endif
      return ICM_20948_STAT_DMP_VERIFY_FAIL;
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }

  //Enable LP_EN since we disabled it at begining of this function.
  const icm20948_status_e low_power_result = icm20948_low_power(pdev, true); // Put chip into low power state
  if (low_power_result != ICM_20948_STAT_OK)
    return low_power_result;

  if (!flag)
  {
    //Serial.println("DMP Firmware was updated successfully..");
    pdev->_firmware_loaded = true;
  }

  return result;
}

icm20948_status_e icm20948_set_dmp_start_address(icm20948_device_t *pdev, unsigned short address)
{
  icm20948_status_e result = ICM_20948_STAT_OK;

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED;

  unsigned char start_address[2];

  start_address[0] = (unsigned char)(address >> 8);
  start_address[1] = (unsigned char)(address & 0xff);

  // result = icm20948_sleep(pdev, false); // Make sure chip is awake
  // if (result != ICM_20948_STAT_OK)
  // {
  // 		return result;
  // }
  //
  // result = ICM_20948_low_power(pdev, false); // Make sure chip is not in low power state
  // if (result != ICM_20948_STAT_OK)
  // {
  // 		return result;
  // }

  result = icm20948_set_bank(pdev, 2); // Set bank 2
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  // Write the sensor control bits into memory address AGB2_REG_PRGM_START_ADDRH
  result = icm20948_execute_w(pdev, AGB2_REG_PRGM_START_ADDRH, (uint8_t *)start_address, 2);

  return result;
}

/**
*  @brief       Write data to a register in DMP memory
*  @param[in]   DMP memory address
*  @param[in]   number of byte to be written
*  @param[out]  output data from the register
*  @return     0 if successful.
*/
icm20948_status_e inv_icm20948_write_mems(icm20948_device_t *pdev, unsigned short reg, unsigned int length, const unsigned char *data)
{
  icm20948_status_e result = ICM_20948_STAT_OK;
  unsigned int bytesWritten = 0;
  unsigned int thisLen;
  unsigned char lBankSelected;
  unsigned char lStartAddrSelected;

  if (!data)
  {
    return ICM_20948_STAT_NO_DATA;
  }

  result = icm20948_set_bank(pdev, 0); // Set bank 0
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  while (bytesWritten < length)
  {
    lBankSelected = (reg >> 8);
    if (lBankSelected != pdev->_last_mems_bank)
    {
      pdev->_last_mems_bank = lBankSelected;
      result = icm20948_execute_w(pdev, AGB0_REG_MEM_BANK_SEL, &lBankSelected, 1);
      if (result != ICM_20948_STAT_OK)
      {
        return result;
      }
    }

    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    result = icm20948_execute_w(pdev, AGB0_REG_MEM_START_ADDR, &lStartAddrSelected, 1);
    if (result != ICM_20948_STAT_OK)
    {
      return result;
    }

    thisLen = length - bytesWritten;
    if (thisLen > INV_MAX_SERIAL_WRITE)
      thisLen = INV_MAX_SERIAL_WRITE;
    if (thisLen > DMP_MEM_BANK_SIZE - lStartAddrSelected)
      thisLen = DMP_MEM_BANK_SIZE - lStartAddrSelected;

    /* Write data */

    result = icm20948_execute_w(pdev, AGB0_REG_MEM_R_W, (uint8_t *)&data[bytesWritten], thisLen);
    if (result != ICM_20948_STAT_OK)
    {
      return result;
    }

    bytesWritten += thisLen;
    reg += thisLen;
  }

  return result;
}

/**
*  @brief      Read data from a register in DMP memory
*  @param[in]  DMP memory address
*  @param[in]  number of byte to be read
*  @param[in]  input data from the register
*  @return     0 if successful.
*/
icm20948_status_e inv_icm20948_read_mems(icm20948_device_t *pdev, unsigned short reg, unsigned int length, unsigned char *data)
{
  icm20948_status_e result = ICM_20948_STAT_OK;
  unsigned int bytesRead = 0;
  unsigned int thisLen;
  unsigned char lBankSelected;
  unsigned char lStartAddrSelected;

  if (!data)
  {
    return ICM_20948_STAT_NO_DATA;
  }

  result = icm20948_set_bank(pdev, 0); // Set bank 0
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  while (bytesRead < length)
  {
    lBankSelected = (reg >> 8);
    if (lBankSelected != pdev->_last_mems_bank)
    {
      pdev->_last_mems_bank = lBankSelected;
      result = icm20948_execute_w(pdev, AGB0_REG_MEM_BANK_SEL, &lBankSelected, 1);
      if (result != ICM_20948_STAT_OK)
      {
        return result;
      }
    }

    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		   Contents are changed after read or write of the selected memory.
		   This register must be written prior to each access to initialize the register to the proper starting address.
		   The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    result = icm20948_execute_w(pdev, AGB0_REG_MEM_START_ADDR, &lStartAddrSelected, 1);
    if (result != ICM_20948_STAT_OK)
    {
      return result;
    }

    thisLen = length - bytesRead;
    if (thisLen > INV_MAX_SERIAL_READ)
      thisLen = INV_MAX_SERIAL_READ;
    if (thisLen > DMP_MEM_BANK_SIZE - lStartAddrSelected)
      thisLen = DMP_MEM_BANK_SIZE - lStartAddrSelected;

    /* Read data */

    result = icm20948_execute_r(pdev, AGB0_REG_MEM_R_W, &data[bytesRead], thisLen);
    if (result != ICM_20948_STAT_OK)
    {
      return result;
    }

    bytesRead += thisLen;
    reg += thisLen;
  }

  return result;
}

icm20948_status_e inv_icm20948_set_dmp_sensor_period(icm20948_device_t *pdev, enum DMP_ODR_Registers odr_reg, uint16_t interval)
{
  // Set the ODR registers and clear the ODR counter

  // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate (225Hz) / ODR ) - 1
  // E.g. For a 25Hz ODR rate, value= (225/25) -1 = 8.

  // During run-time, if an ODR is changed, the corresponding rate counter must be reset.
  // To reset, write 2-byte {0,0} to DMP using keys below for a particular sensor:

  icm20948_status_e result = ICM_20948_STAT_OK;
  icm20948_status_e result2 = ICM_20948_STAT_OK;

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED;

  unsigned char odr_reg_val[2];
  odr_reg_val[0] = (unsigned char)(interval >> 8);
  odr_reg_val[1] = (unsigned char)(interval & 0xff);

  unsigned char odr_count_zero[2] = {0x00, 0x00};

  result = icm20948_sleep(pdev, false); // Make sure chip is awake
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  result = icm20948_low_power(pdev, false); // Make sure chip is not in low power state
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  switch (odr_reg)
  {
  case DMP_ODR_Reg_Cpass_Calibr:
  {
    result = inv_icm20948_write_mems(pdev, ODR_CPASS_CALIBR, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_CPASS_CALIBR, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Gyro_Calibr:
  {
    result = inv_icm20948_write_mems(pdev, ODR_GYRO_CALIBR, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_GYRO_CALIBR, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Pressure:
  {
    result = inv_icm20948_write_mems(pdev, ODR_PRESSURE, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_PRESSURE, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Geomag:
  {
    result = inv_icm20948_write_mems(pdev, ODR_GEOMAG, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_GEOMAG, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_PQuat6:
  {
    result = inv_icm20948_write_mems(pdev, ODR_PQUAT6, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_PQUAT6, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Quat9:
  {
    result = inv_icm20948_write_mems(pdev, ODR_QUAT9, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_QUAT9, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Quat6:
  {
    result = inv_icm20948_write_mems(pdev, ODR_QUAT6, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_QUAT6, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_ALS:
  {
    result = inv_icm20948_write_mems(pdev, ODR_ALS, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_ALS, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Cpass:
  {
    result = inv_icm20948_write_mems(pdev, ODR_CPASS, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_CPASS, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Gyro:
  {
    result = inv_icm20948_write_mems(pdev, ODR_GYRO, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_GYRO, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Accel:
  {
    result = inv_icm20948_write_mems(pdev, ODR_ACCEL, 2, (const unsigned char *)&odr_reg_val);
    result2 = inv_icm20948_write_mems(pdev, ODR_CNTR_ACCEL, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  default:
    result = ICM_20948_STAT_INVALID_DMP_REGISTER;
    break;
  }

  const icm20948_status_e low_power_result = icm20948_low_power(pdev, true); // Put chip into low power state
  if (low_power_result != ICM_20948_STAT_OK)
    return low_power_result;

  if (result2 > result)
    result = result2; // Return the highest error

  return result;
}

icm20948_status_e inv_icm20948_enable_dmp_sensor(icm20948_device_t *pdev, enum inv_icm20948_sensor sensor, int state)
{
  icm20948_status_e result = ICM_20948_STAT_OK;

  uint16_t inv_event_control = 0; // Use this to store the value for MOTION_EVENT_CTL
  uint16_t data_rdy_status = 0;   // Use this to store the value for DATA_RDY_STATUS

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED; // Bail if DMP is not supported

  uint8_t androidSensor = sensor_type_2_android_sensor(sensor); // Convert sensor from enum inv_icm20948_sensor to Android numbering

  if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED; // Bail if the sensor is not supported (TO DO: Support B2S etc)

  // Convert the Android sensor into a bit mask for DATA_OUT_CTL1
  uint16_t delta = inv_androidSensor_to_control_bits[androidSensor];
  if (delta == 0xFFFF)
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED; // Bail if the sensor is not supported

  // Convert the Android sensor number into a bitmask and set or clear that bit in _enabled_Android_0 / _enabled_Android_1
  unsigned long androidSensorAsBitMask;
  if (androidSensor < 32) // Sensors 0-31
  {
    androidSensorAsBitMask = 1L << androidSensor;
    if (state == 0) // Should we disable the sensor?
    {
      pdev->_enabled_Android_0 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
    }
    else
    {
      pdev->_enabled_Android_0 |= androidSensorAsBitMask; // Set the bit to enable the sensor
    }
  }
  else // Sensors 32-
  {
    androidSensorAsBitMask = 1L << (androidSensor - 32);
    if (state == 0) // Should we disable the sensor?
    {
      pdev->_enabled_Android_1 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
    }
    else
    {
      pdev->_enabled_Android_1 |= androidSensorAsBitMask; // Set the bit to enable the sensor
    }
  }

  // Now we know androidSensor is valid, reconstruct the value for DATA_OUT_CTL1 from _enabled_Android_0 and _enabled_Android_0
  delta = 0; // Clear delta
  for (int i = 0; i < 32; i++)
  {
    androidSensorAsBitMask = 1L << i;
    if ((pdev->_enabled_Android_0 & androidSensorAsBitMask) > 0) // Check if the Android sensor (0-31) is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i]; // If it is, or the required bits into delta
    }
    if ((pdev->_enabled_Android_1 & androidSensorAsBitMask) > 0) // Check if the Android sensor (32-) is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i + 32]; // If it is, or the required bits into delta
    }
    // Also check which bits need to be set in the Data Ready Status and Motion Event Control registers
    // Compare to INV_NEEDS_ACCEL_MASK, INV_NEEDS_GYRO_MASK and INV_NEEDS_COMPASS_MASK
    // See issue #150 - thank you @dobodu
    if (((pdev->_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK) > 0)
    || ((pdev->_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Accel;
      inv_event_control |= DMP_Motion_Event_Control_Accel_Calibr;
    }
    if (((pdev->_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK) > 0)
    || ((pdev->_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_GYRO_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Gyro;
      inv_event_control |= DMP_Motion_Event_Control_Gyro_Calibr;
    }
    if (((pdev->_enabled_Android_0 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK) > 0)
    || ((pdev->_enabled_Android_1 & androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Secondary_Compass;
      inv_event_control |= DMP_Motion_Event_Control_Compass_Calibr;
    }
  }

  result = icm20948_sleep(pdev, false); // Make sure chip is awake
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  result = icm20948_low_power(pdev, false); // Make sure chip is not in low power state
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  // Check if Accel, Gyro/Gyro_Calibr or Compass_Calibr/Quat9/GeoMag/Compass are to be enabled. If they are then we need to request the accuracy data via header2.
  uint16_t delta2 = 0;
  if ((delta & DMP_Data_Output_Control_1_Accel) > 0)
  {
    delta2 |= DMP_Data_Output_Control_2_Accel_Accuracy;
  }
  if (((delta & DMP_Data_Output_Control_1_Gyro_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Gyro) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Gyro_Accuracy;
  }
  if (((delta & DMP_Data_Output_Control_1_Compass_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Compass) > 0) || ((delta & DMP_Data_Output_Control_1_Quat9) > 0) || ((delta & DMP_Data_Output_Control_1_Geomag) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Compass_Accuracy;
  }
  // TO DO: Add DMP_Data_Output_Control_2_Pickup etc. if required

  // Write the sensor control bits into memory address DATA_OUT_CTL1
  unsigned char data_output_control_reg[2];
  data_output_control_reg[0] = (unsigned char)(delta >> 8);
  data_output_control_reg[1] = (unsigned char)(delta & 0xff);
  pdev->_dataOutCtl1 = delta; // Diagnostics
  result = inv_icm20948_write_mems(pdev, DATA_OUT_CTL1, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  // Write the 'header2' sensor control bits into memory address DATA_OUT_CTL2
  data_output_control_reg[0] = (unsigned char)(delta2 >> 8);
  data_output_control_reg[1] = (unsigned char)(delta2 & 0xff);
  pdev->_dataOutCtl2 = delta2; // Diagnostics
  result = inv_icm20948_write_mems(pdev, DATA_OUT_CTL2, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  // Set the DATA_RDY_STATUS register
  data_output_control_reg[0] = (unsigned char)(data_rdy_status >> 8);
  data_output_control_reg[1] = (unsigned char)(data_rdy_status & 0xff);
  pdev->_dataRdyStatus = data_rdy_status; // Diagnostics
  result = inv_icm20948_write_mems(pdev, DATA_RDY_STATUS, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  // Check which extra bits need to be set in the Motion Event Control register
  if ((delta & DMP_Data_Output_Control_1_Quat9) > 0)
  {
    inv_event_control |= DMP_Motion_Event_Control_9axis;
  }
  if (((delta & DMP_Data_Output_Control_1_Step_Detector) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_0) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_1) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_2) > 0))
  {
    inv_event_control |= DMP_Motion_Event_Control_Pedometer_Interrupt;
  }
  if ((delta & DMP_Data_Output_Control_1_Geomag) > 0)
  {
    inv_event_control |= DMP_Motion_Event_Control_Geomag;
  }

  // Set the MOTION_EVENT_CTL register
  data_output_control_reg[0] = (unsigned char)(inv_event_control >> 8);
  data_output_control_reg[1] = (unsigned char)(inv_event_control & 0xff);
  pdev->_motionEventCtl = inv_event_control; // Diagnostics
  result = inv_icm20948_write_mems(pdev, MOTION_EVENT_CTL, 2, (const unsigned char *)&data_output_control_reg);
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  result = icm20948_low_power(pdev, true); // Put chip into low power state
  if (result != ICM_20948_STAT_OK)
    return result;

  return result;
}

icm20948_status_e inv_icm20948_enable_dmp_sensor_int(icm20948_device_t *pdev, enum inv_icm20948_sensor sensor, int state)
{
  icm20948_status_e result = ICM_20948_STAT_OK;

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED; // Bail if DMP is not supported

  uint8_t androidSensor = sensor_type_2_android_sensor(sensor); // Convert sensor from enum inv_icm20948_sensor to Android numbering

  if (androidSensor > ANDROID_SENSOR_NUM_MAX)
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED; // Bail if the sensor is not supported

  // Convert the Android sensor into a bit mask for DATA_OUT_CTL1
  uint16_t delta = inv_androidSensor_to_control_bits[androidSensor];
  if (delta == 0xFFFF)
    return ICM_20948_STAT_SENSOR_NOT_SUPPORTED; // Bail if the sensor is not supported

  // Convert the Android sensor number into a bitmask and set or clear that bit in _enabled_Android_intr_0 / _enabled_Android_intr_1
  unsigned long androidSensorAsBitMask;
  if (androidSensor < 32) // Sensors 0-31
  {
    androidSensorAsBitMask = 1L << androidSensor;
    if (state == 0) // Should we disable the sensor interrupt?
    {
      pdev->_enabled_Android_intr_0 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor interrupt
    }
    else
    {
      pdev->_enabled_Android_intr_0 |= androidSensorAsBitMask; // Set the bit to enable the sensor interrupt
    }
  }
  else // Sensors 32-
  {
    androidSensorAsBitMask = 1L << (androidSensor - 32);
    if (state == 0) // Should we disable the sensor?
    {
      pdev->_enabled_Android_intr_1 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor interrupt
    }
    else
    {
      pdev->_enabled_Android_intr_1 |= androidSensorAsBitMask; // Set the bit to enable the sensor interrupt
    }
  }

  // Now we know androidSensor is valid, reconstruct the value for DATA_INTR_CTL from _enabled_Android_intr_0 and _enabled_Android_intr_0
  delta = 0; // Clear delta
  for (int i = 0; i < 32; i++)
  {
    androidSensorAsBitMask = 1L << i;
    if ((pdev->_enabled_Android_intr_0 & androidSensorAsBitMask) > 0) // Check if the Android sensor (0-31) interrupt is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i]; // If it is, or the required bits into delta
    }
    if ((pdev->_enabled_Android_intr_1 & androidSensorAsBitMask) > 0) // Check if the Android sensor (32-) interrupt is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i + 32]; // If it is, or the required bits into delta
    }
  }

  result = icm20948_sleep(pdev, false); // Make sure chip is awake
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  result = icm20948_low_power(pdev, false); // Make sure chip is not in low power state
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  unsigned char data_intr_ctl[2];

  data_intr_ctl[0] = (unsigned char)(delta >> 8);
  data_intr_ctl[1] = (unsigned char)(delta & 0xff);
  pdev->_dataIntrCtl = delta; // Diagnostics
  
  // Write the interrupt control bits into memory address DATA_INTR_CTL
  result = inv_icm20948_write_mems(pdev, DATA_INTR_CTL, 2, (const unsigned char *)&data_intr_ctl);

  result = icm20948_low_power(pdev, true); // Put chip into low power state
  if (result != ICM_20948_STAT_OK)
    return result;

  return result;
}

icm20948_status_e inv_icm20948_read_dmp_data(icm20948_device_t *pdev, icm_20948_DMP_data_t *data)
{
  icm20948_status_e result = ICM_20948_STAT_OK;
  uint8_t fifoBytes[ICM_20948_DMP_MAXIMUM_BYTES]; // Interim storage for the FIFO data

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED;

  // Check how much data is in the FIFO
  uint16_t fifo_count;
  result = icm20948_get_fifo_count(pdev, &fifo_count);
  if (result != ICM_20948_STAT_OK)
    return result;

  if (fifo_count < ICM_20948_DMP_HEADER_BYTES) // Has a 2-byte header arrived?
    return ICM_20948_STAT_FIFO_NO_DATA_AVAIL;     // Bail if no header is available

  // Read the header (2 bytes)
  data->header = 0; // Clear the existing header
  uint16_t aShort = 0;
  result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_HEADER_BYTES);
  if (result != ICM_20948_STAT_OK)
    return result;
  for (int i = 0; i < ICM_20948_DMP_HEADER_BYTES; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8)); // MSB first
  }
  data->header = aShort;                    // Store the header in data->header
  fifo_count -= ICM_20948_DMP_HEADER_BYTES; // Decrement the count

  // If the header indicates a header2 is present then read that now
  data->header2 = 0;                                  // Clear the existing header2
  if ((data->header & DMP_header_bitmap_Header2) > 0) // If the header2 bit is set
  {
    if (fifo_count < ICM_20948_DMP_HEADER_2_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_HEADER_2_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if no header2 is available
    // Read the header (2 bytes)
    aShort = 0;
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_HEADER_2_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_HEADER_2_BYTES; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->header2 = aShort;                    // Store the header2 in data->header2
    fifo_count -= ICM_20948_DMP_HEADER_2_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Accel) > 0) // case DMP_header_bitmap_Accel:
  {
    if (fifo_count < ICM_20948_DMP_RAW_ACCEL_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_RAW_ACCEL_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_RAW_ACCEL_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_RAW_ACCEL_BYTES; i++)
    {
      data->Raw_Accel.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_RAW_ACCEL_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro) > 0) // case DMP_header_bitmap_Gyro:
  {
    if (fifo_count < (ICM_20948_DMP_RAW_GYRO_BYTES + ICM_20948_DMP_GYRO_BIAS_BYTES)) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < (ICM_20948_DMP_RAW_GYRO_BYTES + ICM_20948_DMP_GYRO_BIAS_BYTES))
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], (ICM_20948_DMP_RAW_GYRO_BYTES + ICM_20948_DMP_GYRO_BIAS_BYTES));
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < (ICM_20948_DMP_RAW_GYRO_BYTES + ICM_20948_DMP_GYRO_BIAS_BYTES); i++)
    {
      data->Raw_Gyro.Bytes[DMP_Raw_Gyro_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= (ICM_20948_DMP_RAW_GYRO_BYTES + ICM_20948_DMP_GYRO_BIAS_BYTES); // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Compass) > 0) // case DMP_header_bitmap_Compass:
  {
    if (fifo_count < ICM_20948_DMP_COMPASS_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_COMPASS_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_COMPASS_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_COMPASS_BYTES; i++)
    {
      data->Compass.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_COMPASS_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_ALS) > 0) // case DMP_header_bitmap_ALS:
  {
    if (fifo_count < ICM_20948_DMP_ALS_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_ALS_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_ALS_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_ALS_BYTES; i++)
    {
      data->ALS[i] = fifoBytes[i];
    }
    fifo_count -= ICM_20948_DMP_ALS_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat6) > 0) // case DMP_header_bitmap_Quat6:
  {
    if (fifo_count < ICM_20948_DMP_QUAT6_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_QUAT6_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_QUAT6_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_QUAT6_BYTES; i++)
    {
      data->Quat6.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_QUAT6_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat9) > 0) // case DMP_header_bitmap_Quat9:
  {
    if (fifo_count < ICM_20948_DMP_QUAT9_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_QUAT9_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_QUAT9_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_QUAT9_BYTES; i++)
    {
      data->Quat9.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_QUAT9_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_PQuat6) > 0) // case DMP_header_bitmap_PQuat6:
  {
    if (fifo_count < ICM_20948_DMP_PQUAT6_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_PQUAT6_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_PQUAT6_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_PQUAT6_BYTES; i++)
    {
      data->PQuat6.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_PQUAT6_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Geomag) > 0) // case DMP_header_bitmap_Geomag:
  {
    if (fifo_count < ICM_20948_DMP_GEOMAG_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_GEOMAG_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_GEOMAG_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_GEOMAG_BYTES; i++)
    {
      data->Geomag.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_GEOMAG_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Pressure) > 0) // case DMP_header_bitmap_Pressure:
  {
    if (fifo_count < ICM_20948_DMP_PRESSURE_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_PRESSURE_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_PRESSURE_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_PRESSURE_BYTES; i++)
    {
      data->Pressure[i] = fifoBytes[i];
    }
    fifo_count -= ICM_20948_DMP_PRESSURE_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro_Calibr) > 0) // case DMP_header_bitmap_Gyro_Calibr:
  {
    // lcm20948MPUFifoControl.c suggests ICM_20948_DMP_GYRO_CALIBR_BYTES is not supported
    // and looking at DMP frames which have the Gyro_Calibr bit set, that certainly seems to be true.
    // So, we'll skip this...:
    /*
			if (fifo_count < ICM_20948_DMP_GYRO_CALIBR_BYTES) // Check if we need to read the FIFO count again
			{
					result = ICM_20948_get_FIFO_count(pdev, &fifo_count);
					if (result != ICM_20948_STAT_OK)
							return result;
			}
			if (fifo_count < ICM_20948_DMP_GYRO_CALIBR_BYTES)
					return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
			result = ICM_20948_read_FIFO(pdev, &fifoBytes[0], ICM_20948_DMP_GYRO_CALIBR_BYTES);
			if (result != ICM_20948_STAT_OK)
					return result;
			for (int i = 0; i < ICM_20948_DMP_GYRO_CALIBR_BYTES; i++)
			{
					data->Gyro_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
			}
			fifo_count -= ICM_20948_DMP_GYRO_CALIBR_BYTES; // Decrement the count
			*/
  }

  if ((data->header & DMP_header_bitmap_Compass_Calibr) > 0) // case DMP_header_bitmap_Compass_Calibr:
  {
    if (fifo_count < ICM_20948_DMP_COMPASS_CALIBR_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_COMPASS_CALIBR_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_COMPASS_CALIBR_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_COMPASS_CALIBR_BYTES; i++)
    {
      data->Compass_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= ICM_20948_DMP_COMPASS_CALIBR_BYTES; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Step_Detector) > 0) // case DMP_header_bitmap_Step_Detector:
  {
    if (fifo_count < ICM_20948_DMP_STEP_DETECTOR_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_STEP_DETECTOR_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_STEP_DETECTOR_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    uint32_t aWord = 0;
    for (int i = 0; i < ICM_20948_DMP_STEP_DETECTOR_BYTES; i++)
    {
      aWord |= ((uint32_t)fifoBytes[i]) << (24 - (i * 8)); // MSB first
    }
    data->Pedometer_Timestamp = aWord;
    fifo_count -= ICM_20948_DMP_STEP_DETECTOR_BYTES; // Decrement the count
  }

  // Now check for header2 features

  if ((data->header2 & DMP_header2_bitmap_Accel_Accuracy) > 0) // case DMP_header2_bitmap_Accel_Accuracy:
  {
    if (fifo_count < ICM_20948_DMP_ACCEL_ACCURACY_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_ACCEL_ACCURACY_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    aShort = 0;
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_ACCEL_ACCURACY_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_ACCEL_ACCURACY_BYTES; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Accel_Accuracy = aShort;
    fifo_count -= ICM_20948_DMP_ACCEL_ACCURACY_BYTES; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Gyro_Accuracy) > 0) // case DMP_header2_bitmap_Gyro_Accuracy:
  {
    if (fifo_count < ICM_20948_DMP_GYRO_ACCURACY_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_GYRO_ACCURACY_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    aShort = 0;
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_GYRO_ACCURACY_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_GYRO_ACCURACY_BYTES; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Gyro_Accuracy = aShort;
    fifo_count -= ICM_20948_DMP_GYRO_ACCURACY_BYTES; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Compass_Accuracy) > 0) // case DMP_header2_bitmap_Compass_Accuracy:
  {
    if (fifo_count < ICM_20948_DMP_COMPASS_ACCURACY_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_COMPASS_ACCURACY_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    aShort = 0;
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_COMPASS_ACCURACY_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_COMPASS_ACCURACY_BYTES; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Compass_Accuracy = aShort;
    fifo_count -= ICM_20948_DMP_COMPASS_ACCURACY_BYTES; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Fsync) > 0) // case DMP_header2_bitmap_Fsync:
  {
    // lcm20948MPUFifoControl.c suggests ICM_20948_DMP_FSYNC_DETECTION_BYTES is not supported.
    // So, we'll skip this just in case...:
    /*
			if (fifo_count < ICM_20948_DMP_FSYNC_DETECTION_BYTES) // Check if we need to read the FIFO count again
			{
					result = icm20948_get_fifo_count(pdev, &fifo_count);
					if (result != ICM_20948_STAT_OK)
							return result;
			}
			if (fifo_count < ICM_20948_DMP_FSYNC_DETECTION_BYTES)
					return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
			aShort = 0;
			result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_FSYNC_DETECTION_BYTES);
			if (result != ICM_20948_STAT_OK)
					return result;
			for (int i = 0; i < ICM_20948_DMP_FSYNC_DETECTION_BYTES; i++)
			{
					aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
			}
			data->Fsync_Delay_Time = aShort;
			fifo_count -= ICM_20948_DMP_FSYNC_DETECTION_BYTES; // Decrement the count
			*/
  }

  if ((data->header2 & DMP_header2_bitmap_Pickup) > 0) // case DMP_header2_bitmap_Pickup:
  {
    if (fifo_count < ICM_20948_DMP_PICKUP_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_PICKUP_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    aShort = 0;
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_PICKUP_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_PICKUP_BYTES; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Pickup = aShort;
    fifo_count -= ICM_20948_DMP_PICKUP_BYTES; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Activity_Recog) > 0) // case DMP_header2_bitmap_Activity_Recog:
  {
    if (fifo_count < ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES; i++)
    {
      data->Activity_Recognition.Bytes[DMP_Activity_Recognition_Byte_Ordering[i]] = fifoBytes[i];
    }
    fifo_count -= ICM_20948_DMP_ACTIVITY_RECOGNITION_BYTES; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Secondary_On_Off) > 0) // case DMP_header2_bitmap_Secondary_On_Off:
  {
    if (fifo_count < ICM_20948_DMP_SECONDARY_ON_OFF_BYTES) // Check if we need to read the FIFO count again
    {
      result = icm20948_get_fifo_count(pdev, &fifo_count);
      if (result != ICM_20948_STAT_OK)
        return result;
    }
    if (fifo_count < ICM_20948_DMP_SECONDARY_ON_OFF_BYTES)
      return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
    result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_SECONDARY_ON_OFF_BYTES);
    if (result != ICM_20948_STAT_OK)
      return result;
    for (int i = 0; i < ICM_20948_DMP_SECONDARY_ON_OFF_BYTES; i++)
    {
      data->Secondary_On_Off.Bytes[DMP_Secondary_On_Off_Byte_Ordering[i]] = fifoBytes[i];
    }
    fifo_count -= ICM_20948_DMP_SECONDARY_ON_OFF_BYTES; // Decrement the count
  }

  // Finally, extract the footer (gyro count)
  if (fifo_count < ICM_20948_DMP_FOOTER_BYTES) // Check if we need to read the FIFO count again
  {
    result = icm20948_get_fifo_count(pdev, &fifo_count);
    if (result != ICM_20948_STAT_OK)
      return result;
  }
  if (fifo_count < ICM_20948_DMP_FOOTER_BYTES)
    return ICM_20948_STAT_FIFO_INCOMPLETE_DATA; // Bail if not enough data is available
  aShort = 0;
  result = icm20948_read_fifo(pdev, &fifoBytes[0], ICM_20948_DMP_FOOTER_BYTES);
  if (result != ICM_20948_STAT_OK)
    return result;
  for (int i = 0; i < ICM_20948_DMP_FOOTER_BYTES; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
  }
  data->Footer = aShort;
  fifo_count -= ICM_20948_DMP_FOOTER_BYTES; // Decrement the count

  if (fifo_count > 0) // Check if there is still data waiting to be read
    return ICM_20948_STAT_FIFO_MORE_DATA_AVAIL;

  return result;
}

uint8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor)
{
  switch (sensor)
  {
  case INV_ICM20948_SENSOR_ACCELEROMETER:
    return ANDROID_SENSOR_ACCELEROMETER; // 1
  case INV_ICM20948_SENSOR_GYROSCOPE:
    return ANDROID_SENSOR_GYROSCOPE; // 4
  case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
    return ANDROID_SENSOR_RAW_ACCELEROMETER; // 42
  case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
    return ANDROID_SENSOR_RAW_GYROSCOPE; // 43
  case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
    return ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED; // 14
  case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
    return ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED; // 16
  case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
    return ANDROID_SENSOR_ACTIVITY_CLASSIFICATON; // 47
  case INV_ICM20948_SENSOR_STEP_DETECTOR:
    return ANDROID_SENSOR_STEP_DETECTOR; // 18
  case INV_ICM20948_SENSOR_STEP_COUNTER:
    return ANDROID_SENSOR_STEP_COUNTER; // 19
  case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
    return ANDROID_SENSOR_GAME_ROTATION_VECTOR; // 15
  case INV_ICM20948_SENSOR_ROTATION_VECTOR:
    return ANDROID_SENSOR_ROTATION_VECTOR; // 11
  case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
    return ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR; // 20
  case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
    return ANDROID_SENSOR_GEOMAGNETIC_FIELD; // 2
  case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
    return ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION; // 17
  case INV_ICM20948_SENSOR_FLIP_PICKUP:
    return ANDROID_SENSOR_FLIP_PICKUP; // 46
  case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
    return ANDROID_SENSOR_WAKEUP_TILT_DETECTOR; // 41
  case INV_ICM20948_SENSOR_GRAVITY:
    return ANDROID_SENSOR_GRAVITY; // 9
  case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
    return ANDROID_SENSOR_LINEAR_ACCELERATION; // 10
  case INV_ICM20948_SENSOR_ORIENTATION:
    return ANDROID_SENSOR_ORIENTATION; // 3
  case INV_ICM20948_SENSOR_B2S:
    return ANDROID_SENSOR_B2S; // 45
  default:
    return ANDROID_SENSOR_NUM_MAX;
  }
}

enum inv_icm20948_sensor inv_icm20948_sensor_android_2_sensor_type(int sensor)
{
  switch (sensor)
  {
  case ANDROID_SENSOR_ACCELEROMETER:
    return INV_ICM20948_SENSOR_ACCELEROMETER;
  case ANDROID_SENSOR_GYROSCOPE:
    return INV_ICM20948_SENSOR_GYROSCOPE;
  case ANDROID_SENSOR_RAW_ACCELEROMETER:
    return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
  case ANDROID_SENSOR_RAW_GYROSCOPE:
    return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
  case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
    return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
  case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
    return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
  case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
    return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
  case ANDROID_SENSOR_STEP_DETECTOR:
    return INV_ICM20948_SENSOR_STEP_DETECTOR;
  case ANDROID_SENSOR_STEP_COUNTER:
    return INV_ICM20948_SENSOR_STEP_COUNTER;
  case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
  case ANDROID_SENSOR_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_ROTATION_VECTOR;
  case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
  case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
    return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
  case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
    return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
  case ANDROID_SENSOR_FLIP_PICKUP:
    return INV_ICM20948_SENSOR_FLIP_PICKUP;
  case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
    return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
  case ANDROID_SENSOR_GRAVITY:
    return INV_ICM20948_SENSOR_GRAVITY;
  case ANDROID_SENSOR_LINEAR_ACCELERATION:
    return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
  case ANDROID_SENSOR_ORIENTATION:
    return INV_ICM20948_SENSOR_ORIENTATION;
  case ANDROID_SENSOR_B2S:
    return INV_ICM20948_SENSOR_B2S;
  default:
    return INV_ICM20948_SENSOR_MAX;
  }
}

icm20948_status_e inv_icm20948_set_gyro_sf(icm20948_device_t *pdev, unsigned char div, int gyro_level)
{
  icm20948_status_e result = ICM_20948_STAT_OK;

  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_STAT_DMP_NOT_SUPPORTED;

  // gyro_level should be set to 4 regardless of fullscale, due to the addition of API dmp_icm20648_set_gyro_fsr()
  gyro_level = 4;

  // First read the TIMEBASE_CORRECTION_PLL register from Bank 1
  int8_t pll; // Signed. Typical value is 0x18
  result = icm20948_set_bank(pdev, 1);
  result = icm20948_execute_r(pdev, AGB1_REG_TIMEBASE_CORRECTION_PLL, (uint8_t *)&pll, 1);
  if (result != ICM_20948_STAT_OK)
  {
    return result;
  }

  pdev->_gyroSFpll = pll; // Record the PLL value so we can debug print it

  // Now calculate the Gyro SF using code taken from the InvenSense example (inv_icm20948_set_gyro_sf)

  long gyro_sf;

  unsigned long long const MagicConstant = 264446880937391LL;
  unsigned long long const MagicConstantScale = 100000LL;
  unsigned long long ResultLL;

  if (pll & 0x80)
  {
    ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 - (pll & 0x7F)) / MagicConstantScale);
  }
  else
  {
    ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 + pll) / MagicConstantScale);
  }
  /*
	    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
	    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
	    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
	*/
  if (ResultLL > 0x7FFFFFFF)
    gyro_sf = 0x7FFFFFFF;
  else
    gyro_sf = (long)ResultLL;

  pdev->_gyroSF = gyro_sf; // Record value so we can debug print it

  // Finally, write the value to the DMP GYRO_SF register
  unsigned char gyro_sf_reg[4];
  gyro_sf_reg[0] = (unsigned char)(gyro_sf >> 24);
  gyro_sf_reg[1] = (unsigned char)(gyro_sf >> 16);
  gyro_sf_reg[2] = (unsigned char)(gyro_sf >> 8);
  gyro_sf_reg[3] = (unsigned char)(gyro_sf & 0xff);
  result = inv_icm20948_write_mems(pdev, GYRO_SF, 4, (const unsigned char*)&gyro_sf_reg);

  return result;
}
