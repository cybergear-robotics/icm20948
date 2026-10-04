#include "icm20948.h"

#include "ak09916_registers.h"

// Combine all of the DMP start-up code from the earlier DMP examples
// This function is defined as __attribute__((weak)) so you can overwrite it if you want to,
//   e.g. to modify the sample rate
icm20948_status_e icm20948_init_dmp_sensor_with_defaults(icm20948_device_t *pdev)
{
    icm20948_status_e  worst_result = ICM_20948_STAT_OK;
    // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
    // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
    icm20948_status_e  result = ICM_20948_STAT_OK; // Use result and worst_result to show if the configuration was successful

    // The AK09916 is connected through the ICM-20948 auxiliary I2C master.
    result = icm20948_i2c_master_enable(pdev, true);
    if (result > worst_result) worst_result = result;

    // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
    // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
    // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
    // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
    //
    // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
    // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
    // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
    // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
    // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
    //
    // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
    // 0: use I2C_SLV0
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
    // 10: we read 10 bytes each cycle
    // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
    result = icm20948_i2c_controller_configure_peripheral(pdev, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, 0); 
    if (result > worst_result) worst_result = result;

    // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
    // 1: use I2C_SLV1
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_CNTL2: we start writing here (0x31)
    // 1: not sure why, but the write does not happen if this is set to zero
    // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
    // AK09916_MODE_SINGLE: tell I2C_SLV1 to write the Single Measurement command each sample
    result = icm20948_i2c_controller_configure_peripheral(pdev, 1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_MODE_SINGLE); 
    if (result > worst_result) worst_result = result;

    // Set the I2C Master ODR configuration
    // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
    // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
    //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
    //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
    //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
    // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
    // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
    result = icm20948_set_bank(pdev, 3); 
    if (result > worst_result) worst_result = result; // Select Bank 3
    uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
    result = icm20948_execute_w(pdev, AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); 
    if (result > worst_result) worst_result = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

    // Configure clock source through PWR_MGMT_1
    // CLOCK_AUTO selects the best available clock source – PLL if ready, else use the Internal oscillator
    result = icm20948_set_clock_source(pdev, CLOCK_AUTO); 
    if (result > worst_result) worst_result = result; // This is shorthand: success will be set to false if setClockSource fails

    // Enable accel and gyro sensors through PWR_MGMT_2
    // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
    result = icm20948_set_bank(pdev, 0); 
    if (result > worst_result) worst_result = result;                               // Select Bank 0
    uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
    result = icm20948_execute_w(pdev, AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); 
    if (result > worst_result) worst_result = result; // Write one byte to the PWR_MGMT_2 register

    // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
    // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
    result = icm20948_set_sample_mode(pdev, ICM_20948_INTERNAL_MST, SAMPLE_MODE_CYCLED); 
    if (result > worst_result) worst_result = result;

    // Disable the FIFO
    result = icm20948_enable_fifo(pdev, false); 
    if (result > worst_result) worst_result = result;

    // Disable the DMP
    result = icm20948_enable_dmp(pdev, 0); 
    if (result > worst_result) worst_result = result;

    // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
    // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
    icm20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = GPM_4;        // (icm20948_accel_config_fs_sel_e)
                            // gpm2
                            // GPM_4
                            // GPM_8
                            // GPM_16
    myFSS.g = DPS_2000;     // (icm20948_gyro_config_1_fs_sel_e)
                            // DPS_250
                            // DPS_500
                            // DPS_1000
                            // DPS_2000
    result = icm20948_set_full_scale(pdev, (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR), myFSS); 
    if (result > worst_result) worst_result = result;

    // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
    // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
    // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
    result = icm20948_enable_dlpf(pdev, ICM_20948_INTERNAL_GYR, true); 
    if (result > worst_result) worst_result = result;

    // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
    // If we see this interrupt, we'll need to reset the FIFO
    //result = intEnableOverflowFIFO( 0x1F ); if (result > worst_result) worst_result = result; // Enable the interrupt on all FIFOs

    // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
    // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
    result = icm20948_set_bank(pdev, 0); 
    if (result > worst_result) worst_result = result; // Select Bank 0
    uint8_t zero = 0;
    result = icm20948_execute_w(pdev, AGB0_REG_FIFO_EN_1, &zero, 1); 
    if (result > worst_result) worst_result = result;
    // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
    result = icm20948_execute_w(pdev, AGB0_REG_FIFO_EN_2, &zero, 1); 
    if (result > worst_result) worst_result = result;

    // Turn off data ready interrupt through INT_ENABLE_1
    icm20948_int_enable_t en;                          // storage
    result = icm20948_int_enable(pdev, NULL, &en); // read phase
    if (result > worst_result) worst_result = result;

    en.RAW_DATA_0_RDY_EN = false;                     // change the setting
    result = icm20948_int_enable(pdev, &en, &en); // write phase w/ readback
    if (result > worst_result) worst_result = result;
    if (en.RAW_DATA_0_RDY_EN != false)
    {
        result = ICM_20948_STAT_ERR;
        if (result > worst_result) worst_result = result;
    }  

    // Reset FIFO through FIFO_RST
    result = icm20948_reset_fifo(pdev); 
    if (result > worst_result) worst_result = result;

    // Set gyro sample rate divider with GYRO_SMPLRT_DIV
    // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
    icm20948_smplrt_t mySmplrt;
    mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
    mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
    //mySmplrt.g = 4; // 225Hz
    //mySmplrt.a = 4; // 225Hz
    //mySmplrt.g = 8; // 112Hz
    //mySmplrt.a = 8; // 112Hz
    result = icm20948_set_sample_rate(pdev, (ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR), mySmplrt); 
    if (result > worst_result) worst_result = result;

    // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = icm20948_set_dmp_start_address(pdev, DMP_START_ADDRESS); 
    if (result > worst_result) worst_result = result; // Defaults to DMP_START_ADDRESS

    // Now load the DMP firmware
    result = icm20948_firmware_load(pdev); 
    if (result > worst_result) worst_result = result;

    // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = icm20948_set_dmp_start_address(pdev, DMP_START_ADDRESS); 
    if (result > worst_result) worst_result = result; // Defaults to DMP_START_ADDRESS

    // Set the Hardware Fix Disable register to 0x48
    result = icm20948_set_bank(pdev, 0);
    if (result > worst_result) worst_result = result; // Select Bank 0
    uint8_t fix = 0x48;
    result = icm20948_execute_w(pdev, AGB0_REG_HW_FIX_DISABLE, &fix, 1); 
    if (result > worst_result) worst_result = result;

    // Set the Single FIFO Priority Select register to 0xE4
    result = icm20948_set_bank(pdev, 0); 
    if (result > worst_result) worst_result = result; // Select Bank 0
    uint8_t fifoPrio = 0xE4;
    result = icm20948_execute_w(pdev, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); 
    if (result > worst_result) worst_result = result;

    // Configure Accel scaling to DMP
    // The DMP scales accel raw data internally to align 1g as 2^25
    // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
    const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
    result = inv_icm20948_write_mems(pdev, ACC_SCALE, 4, &accScale[0]); 
    if (result > worst_result) worst_result = result; // Write accScale to ACC_SCALE DMP register
    // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
    const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
    result = inv_icm20948_write_mems(pdev, ACC_SCALE2, 4, &accScale2[0]); 
    if (result > worst_result) worst_result = result; // Write accScale2 to ACC_SCALE2 DMP register

    // Configure Compass mount matrix and scale to DMP
    // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
    // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
    // Each compass axis will be converted as below:
    // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
    // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
    // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
    // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
    // 2^30 / 6.66666 = 161061273 = 0x9999999
    const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
    const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_00, 4, &mountMultiplierPlus[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_01, 4, &mountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_02, 4, &mountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_10, 4, &mountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_11, 4, &mountMultiplierMinus[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_12, 4, &mountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_20, 4, &mountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_21, 4, &mountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, CPASS_MTX_22, 4, &mountMultiplierMinus[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the B2S Mounting Matrix
    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = inv_icm20948_write_mems(pdev, B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); 
    if (result > worst_result) worst_result = result;
    result = inv_icm20948_write_mems(pdev, B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the DMP Gyro Scaling Factor
    // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
    //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
    //            10=102.2727Hz sample rate, ... etc.
    // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
    result = inv_icm20948_set_gyro_sf(pdev, 19, 3); // 19 = 55Hz (see above), 3 = 2000dps (see above)
    if (result > worst_result) worst_result = result; 

    // Configure the Gyro full scale
    // 2000dps : 2^28
    // 1000dps : 2^27
    //  500dps : 2^26
    //  250dps : 2^25
    const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
    result = inv_icm20948_write_mems(pdev, GYRO_FULLSCALE, 4, &gyroFullScale[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
    const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
    //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
    //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
    result = inv_icm20948_write_mems(pdev, ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
    const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
    //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
    //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
    result = inv_icm20948_write_mems(pdev, ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
    const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
    //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
    //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
    result = inv_icm20948_write_mems(pdev, ACCEL_A_VAR, 4, &accelAVar[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the Accel Cal Rate
    const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = inv_icm20948_write_mems(pdev, ACCEL_CAL_RATE, 2, &accelCalRate[0]); 
    if (result > worst_result) worst_result = result;

    // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
    // Let's set the Compass Time Buffer to 69 (Hz).
    const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
    result = inv_icm20948_write_mems(pdev, CPASS_TIME_BUFFER, 2, &compassRate[0]); 
    if (result > worst_result) worst_result = result;

    // Enable DMP interrupt
    // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
    //result = intEnableDMP(true); if (result > worst_result) worst_result = result;

    return worst_result;
}
