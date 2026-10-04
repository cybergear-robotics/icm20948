#ifndef _ICM_20948_C_H_
#define _ICM_20948_C_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sdkconfig.h"

#include "icm20948_registers.h"
#include "icm20948_enumerations.h"
#include "ak09916_enumerations.h"
#include "icm20948_dmp.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int memcmp(const void *, const void *, size_t); // Avoid compiler warnings

// There are two versions of the InvenSense DMP firmware for the ICM20948 - with slightly different sizes
#define DMP_CODE_SIZE 14301 /* eMD-SmartMotion-ICM20948-1.1.0-MP */
//#define DMP_CODE_SIZE 14290 /* ICM20948_eMD_nucleo_1.0 */


#define ICM_20948_I2C_ADDR_AD0 0x68 // Or 0x69 when AD0 is high
#define ICM_20948_I2C_ADDR_AD1 0x69 //
#define ICM_20948_WHOAMI 0xEA

#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_REG_WHO_AM_I 0x00

/** @brief Max size that can be read across I2C or SPI data lines */
#define INV_MAX_SERIAL_READ 16
/** @brief Max size that can be written across I2C or SPI data lines */
#define INV_MAX_SERIAL_WRITE 16

  typedef enum
  {
    ICM_20948_STAT_OK = 0x00, // The only return code that means all is well
    ICM_20948_STAT_ERR,       // A general error
    ICM_20948_STAT_NOT_IMPL,   // Returned by virtual functions that are not implemented
    ICM_20948_STAT_PARAM_ERR,
    ICM_20948_STAT_WRONG_ID,
    ICM_20948_STAT_INVALID_SENSOR, // Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
    ICM_20948_STAT_NO_DATA,
    ICM_20948_STAT_SENSOR_NOT_SUPPORTED,
    ICM_20948_STAT_DMP_NOT_SUPPORTED,
    ICM_20948_STAT_DMP_VERIFY_FAIL,      // DMP was written but did not verify correctly
    ICM_20948_STAT_FIFO_NO_DATA_AVAIL,
    ICM_20948_STAT_FIFO_INCOMPLETE_DATA,
    ICM_20948_STAT_FIFO_MORE_DATA_AVAIL,
    ICM_20948_STAT_UNRECOGNISED_DMP_HEADER,
    ICM_20948_STAT_UNRECOGNISED_DMP_HEADER_2,
    ICM_20948_STAT_INVALID_DMP_REGISTER,

    ICM_20948_STAT_NUM,
    ICM_20948_STAT_UNKNOWN,
  } icm20948_status_e;

  typedef enum
  {
    ICM_20948_INTERNAL_ACC = (1 << 0),
    ICM_20948_INTERNAL_GYR = (1 << 1),
    ICM_20948_INTERNAL_MAG = (1 << 2),
    ICM_20948_INTERNAL_TMP = (1 << 3),
    ICM_20948_INTERNAL_MST = (1 << 4), // I2C Master Ineternal
  } icm20948_internal_sensor_id_bm;     // A bitmask of internal sensor IDs

  typedef union
  {
    int16_t i16bit[3];
    uint8_t u8bit[6];
  } icm20948_axis3bit16_t;

  typedef union
  {
    int16_t i16bit;
    uint8_t u8bit[2];
  } icm20948_axis1bit16_t;

  typedef struct
  {
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
  } icm20948_fss_t; // Holds full-scale settings to be able to extract measurements with units

  typedef struct
  {
    uint8_t a;
    uint8_t g;
  } icm20948_dlpcfg_t; // Holds digital low pass filter settings. Members are type icm20948_accel_config_dlpcfg_e

  typedef struct
  {
    uint16_t a;
    uint8_t g;
  } icm20948_smplrt_t;

  typedef struct
  {
    uint8_t I2C_MST_INT_EN : 1;
    uint8_t DMP_INT1_EN : 1;
    uint8_t PLL_RDY_EN : 1;
    uint8_t WOM_INT_EN : 1;
    uint8_t REG_WOF_EN : 1;
    uint8_t RAW_DATA_0_RDY_EN : 1;
    uint8_t FIFO_OVERFLOW_EN_4 : 1;
    uint8_t FIFO_OVERFLOW_EN_3 : 1;
    uint8_t FIFO_OVERFLOW_EN_2 : 1;
    uint8_t FIFO_OVERFLOW_EN_1 : 1;
    uint8_t FIFO_OVERFLOW_EN_0 : 1;
    uint8_t FIFO_WM_EN_4 : 1;
    uint8_t FIFO_WM_EN_3 : 1;
    uint8_t FIFO_WM_EN_2 : 1;
    uint8_t FIFO_WM_EN_1 : 1;
    uint8_t FIFO_WM_EN_0 : 1;
  } icm20948_int_enable_t;

  typedef union
  {
    icm20948_axis3bit16_t raw;
    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } axes;
  } icm20948_axis3named_t;

  typedef struct
  {
    icm20948_axis3named_t acc;
    icm20948_axis3named_t gyr;
    icm20948_axis3named_t mag;
    union
    {
      icm20948_axis1bit16_t raw;
      int16_t val;
    } tmp;
    icm20948_fss_t fss; // Full-scale range settings for this measurement
    uint8_t magStat1;
    uint8_t magStat2;
  } icm20948_agmt_t;

  typedef struct
  {
    icm20948_status_e (*write)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    icm20948_status_e (*read)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    // void				(*delay)(uint32_t ms);
    void *user;
  } icm20948_serif_t;                      // This is the vtable of serial interface functions
  extern const icm20948_serif_t NullSerif; // Here is a default for initialization (NULL)

  typedef struct
  {
    const icm20948_serif_t *_serif; // Pointer to the assigned Serif (Serial Interface) vtable
    bool _dmp_firmware_available;    // Indicates if the DMP firmware has been included. It
    bool _firmware_loaded;           // Indicates if DMP has been loaded
    uint8_t _last_bank;              // Keep track of which bank was selected last - to avoid unnecessary writes
    uint8_t _last_mems_bank;         // Keep track of which bank was selected last - to avoid unnecessary writes
    int32_t _gyroSF;                 // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    int8_t _gyroSFpll;
    uint32_t _enabled_Android_0;      // Keep track of which Android sensors are enabled: 0-31
    uint32_t _enabled_Android_1;      // Keep track of which Android sensors are enabled: 32-
    uint32_t _enabled_Android_intr_0; // Keep track of which Android sensor interrupts are enabled: 0-31
    uint32_t _enabled_Android_intr_1; // Keep track of which Android sensor interrupts are enabled: 32-
    uint16_t _dataOutCtl1;            // Diagnostics: record the setting of DATA_OUT_CTL1
    uint16_t _dataOutCtl2;            // Diagnostics: record the setting of DATA_OUT_CTL2
    uint16_t _dataRdyStatus;          // Diagnostics: record the setting of DATA_RDY_STATUS
    uint16_t _motionEventCtl;         // Diagnostics: record the setting of MOTION_EVENT_CTL
    uint16_t _dataIntrCtl;            // Diagnostics: record the setting of DATA_INTR_CTL
  } icm20948_device_t;               // Definition of device struct type

  icm20948_status_e icm20948_init_struct(icm20948_device_t *pdev); // Initialize icm20948_device_t

  // icm20948_status_e ICM_20948_Startup( icm20948_device_t* pdev ); // For the time being this performs a standardized startup routine

  icm20948_status_e icm20948_link_serif(icm20948_device_t *pdev, const icm20948_serif_t *s); // Links a SERIF structure to the device

  // use the device's serif to perform a read or write
  icm20948_status_e icm20948_execute_r(icm20948_device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len); // Executes a R or W witht he serif vt as long as the pointers are not null
  icm20948_status_e icm20948_execute_w(icm20948_device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len);

  // Single-shot I2C on Master IF
  icm20948_status_e icm20948_i2c_controller_periph4_txn(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
  icm20948_status_e icm20948_i2c_master_single_w(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);
  icm20948_status_e icm20948_i2c_master_single_r(icm20948_device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);

  // Device Level
  icm20948_status_e icm20948_set_bank(icm20948_device_t *pdev, uint8_t bank);                                 // Sets the bank
  icm20948_status_e icm20948_sw_reset(icm20948_device_t *pdev);                                               // Performs a SW reset
  icm20948_status_e icm20948_sleep(icm20948_device_t *pdev, bool on);                                         // Set sleep mode for the chip
  icm20948_status_e icm20948_low_power(icm20948_device_t *pdev, bool on);                                     // Set low power mode for the chip
  icm20948_status_e icm20948_set_clock_source(icm20948_device_t *pdev, icm20948_pwr_mgmt_1_clksel_e source); // Choose clock source
  icm20948_status_e icm20948_get_who_am_i(icm20948_device_t *pdev, uint8_t *whoami);                          // Return whoami in out prarmeter
  icm20948_status_e icm20948_check_id(icm20948_device_t *pdev);                                               // Return 'ICM_20948_STAT_OK' if whoami matches ICM_20948_WHOAMI
  icm20948_status_e icm20948_data_ready(icm20948_device_t *pdev);                                             // Returns 'Ok' if data is ready

  // Interrupt Configuration
  icm20948_status_e icm20948_int_pin_cfg(icm20948_device_t *pdev, icm20948_int_pin_cfg_t *write, icm20948_int_pin_cfg_t *read); // Set the INT pin configuration
  icm20948_status_e icm20948_int_enable(icm20948_device_t *pdev, icm20948_int_enable_t *write, icm20948_int_enable_t *read);    // Write and or read the interrupt enable information. If non-null the write operation occurs before the read, so as to verify that the write was successful

  // WoM Enable Logic configuration
  icm20948_status_e icm20948_wom_logic(icm20948_device_t *pdev, icm20948_accel_intel_ctrl_t *write, icm20948_accel_intel_ctrl_t *read); //Enable or disable WoM Logic

  // WoM Threshold Level Configuration
  icm20948_status_e icm20948_wom_threshold(icm20948_device_t *pdev, icm20948_accel_wom_thr_t *write, icm20948_accel_wom_thr_t *read); // Write and or read the Wake on Motion threshold. If non-null the write operation occurs before the read, so as to verify that the write was successful

  // Internal Sensor Options
  icm20948_status_e icm20948_set_sample_mode(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_lp_config_cycle_e mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
  icm20948_status_e icm20948_set_full_scale(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_fss_t fss);
  icm20948_status_e icm20948_set_dlpf_cfg(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_dlpcfg_t cfg);
  icm20948_status_e icm20948_enable_dlpf(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, bool enable);
  icm20948_status_e icm20948_set_sample_rate(icm20948_device_t *pdev, icm20948_internal_sensor_id_bm sensors, icm20948_smplrt_t smplrt);

  // Interface Things
  icm20948_status_e icm20948_i2c_master_passthrough(icm20948_device_t *pdev, bool passthrough);
  icm20948_status_e icm20948_i2c_master_enable(icm20948_device_t *pdev, bool enable);
  icm20948_status_e icm20948_i2c_master_reset(icm20948_device_t *pdev);
  icm20948_status_e icm20948_i2c_controller_configure_peripheral(icm20948_device_t *pdev, uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);

  // Higher Level
  icm20948_status_e icm20948_get_agmt(icm20948_device_t *pdev, icm20948_agmt_t *p);

  // FIFO

  icm20948_status_e icm20948_enable_fifo(icm20948_device_t *pdev, bool enable);
  icm20948_status_e icm20948_reset_fifo(icm20948_device_t *pdev);
  icm20948_status_e icm20948_set_fifo_mode(icm20948_device_t *pdev, bool snapshot);
  icm20948_status_e icm20948_get_fifo_count(icm20948_device_t *pdev, uint16_t *count);
  icm20948_status_e icm20948_read_fifo(icm20948_device_t *pdev, uint8_t *data, uint8_t len);

  // DMP

  icm20948_status_e icm20948_enable_dmp(icm20948_device_t *pdev, bool enable);
  icm20948_status_e icm20948_reset_dmp(icm20948_device_t *pdev);
  icm20948_status_e icm20948_firmware_load(icm20948_device_t *pdev);
  icm20948_status_e icm20948_set_dmp_start_address(icm20948_device_t *pdev, unsigned short address);

  /** @brief Loads the DMP firmware from SRAM
	* @param[in] data  pointer where the image
	* @param[in] size  size if the image
	* @param[in] load_addr  address to loading the image
	* @return 0 in case of success, -1 for any error
	*/
  icm20948_status_e inv_icm20948_firmware_load(icm20948_device_t *pdev, const unsigned char *data, unsigned short size, unsigned short load_addr);
  /**
	*  @brief       Write data to a register in DMP memory
	*  @param[in]   DMP memory address
	*  @param[in]   number of byte to be written
	*  @param[out]  output data from the register
	*  @return     0 if successful.
	*/
  icm20948_status_e inv_icm20948_write_mems(icm20948_device_t *pdev, unsigned short reg, unsigned int length, const unsigned char *data);
  /**
	*  @brief      Read data from a register in DMP memory
	*  @param[in]  DMP memory address
	*  @param[in]  number of byte to be read
	*  @param[in]  input data from the register
	*  @return     0 if successful.
	*/
  icm20948_status_e inv_icm20948_read_mems(icm20948_device_t *pdev, unsigned short reg, unsigned int length, unsigned char *data);

  icm20948_status_e inv_icm20948_set_dmp_sensor_period(icm20948_device_t *pdev, enum DMP_ODR_Registers odr_reg, uint16_t interval);
  icm20948_status_e inv_icm20948_enable_dmp_sensor(icm20948_device_t *pdev, enum inv_icm20948_sensor sensor, int state);     // State is actually boolean
  icm20948_status_e inv_icm20948_enable_dmp_sensor_int(icm20948_device_t *pdev, enum inv_icm20948_sensor sensor, int state); // State is actually boolean
  uint8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor);
  enum inv_icm20948_sensor inv_icm20948_sensor_android_2_sensor_type(int sensor);

  icm20948_status_e inv_icm20948_read_dmp_data(icm20948_device_t *pdev, icm_20948_DMP_data_t *data);
  icm20948_status_e inv_icm20948_set_gyro_sf(icm20948_device_t *pdev, unsigned char div, int gyro_level);

  // ToDo:

  /*
	Want to access magnetometer throught the I2C master interface...

  // If using the I2C master to read from the magnetometer
  // Enable the I2C master to talk to the magnetometer through the ICM 20948
  myICM.i2cMasterEnable( true );
  SERIAL_PORT.print(F("Enabling the I2C master returned ")); SERIAL_PORT.println(myICM.statusString());
  myICM.i2cControllerConfigurePeripheral ( 0, MAG_AK09916_I2C_ADDR, REG_ST1, 9, true, true, false, false, false );
  SERIAL_PORT.print(F("Configuring the magnetometer peripheral returned ")); SERIAL_PORT.println(myICM.statusString());

  // Operate the I2C master in duty-cycled mode
  myICM.setSampleMode( (ICM_20948_INTERNAL_MST | ICM_20948_INTERNAL_GYR), SAMPLE_MODE_CYCLED ); // options: SAMPLE_MODE_CONTINUOUS or SAMPLE_MODE_CYCLED
*/


  icm20948_status_e icm20948_init_dmp_sensor_with_defaults(icm20948_device_t *pdev);

#ifdef __cplusplus
}
#endif

#endif /* _ICM_20948_C_H_ */
