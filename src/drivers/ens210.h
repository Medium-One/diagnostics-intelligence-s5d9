/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */


#ifndef __ENS210_H_
#define __ENS210_H_

#include <stdbool.h>
#include <stdint.h>

/** @defgroup ENS210   ENS210 Driver API
 * This module provides the API to operate an ENS210 relative humidity and temperature sensor with I2C interface.
 *
 * Basic steps to operate the sensor are as follows:
 * -# Set Run mode (#ENS210_SensRun_Set)
 * -# Start measurement (#ENS210_SensStart_Set)
 * -# Wait for measurement to complete
 * -# Read measurement (#ENS210_TVal_Get, #ENS210_HVal_Get, #ENS210_THVal_Get)
 *
 * Please refer to ENS210 Reference Driver and Porting Guide for more details on platform porting. In this module, names
 * T and H have been used to refer to temperature and relative humidity respectively to comply with ENS210 datasheet
 * naming convention.
 *
 * Example 1 - Sample application code to measure temperature and relative humidity without error checking
 * -------------------------------------------------------------------------------------------------------
 * @code
 * uint32_t T_Raw, H_Raw;
 * int32_t T_mCelsius, T_mFahrenheit, T_mKelvin, H_Percent;
 *
 * //Set runmode, start measurement, wait, read measurement (for both T and H)
 * ENS210_SensRun_Set(ENS210_SENSRUN_T_MODE_SINGLE_SHOT | ENS210_SENSRUN_H_MODE_SINGLE_SHOT);
 * ENS210_SensStart_Set(ENS210_SENSSTART_T_START | ENS210_SENSSTART_H_START);
 * WaitMsec(ENS210_T_H_CONVERSION_TIME_MS);
 * ENS210_THVal_Get(&T_Raw,&H_Raw);
 *
 * //Convert the raw temperature to milli Kelvin
 * T_mKelvin = ENS210_ConvertRawToKelvin(T_Raw, 1000);
 * //Convert the raw temperature to milli Celsius
 * T_mCelsius = ENS210_ConvertRawToCelsius(T_Raw, 1000);
 * //Convert the raw temperature to milli Fahrenheit
 * T_mFahrenheit = ENS210_ConvertRawToFahrenheit(T_Raw, 1000);
 * printf("T crc ok = %s\n", ENS210_IsCrcOk(T_Raw)  ? "yes" : "no");
 * printf("T valid = %s \n", ENS210_IsDataValid(T_Raw) ? "yes" : "no");
 * //Update the int32_t format specifier (%ld) based on platform word-size 
 * printf("T = %ld mK %ld mC %ld mF \n", T_mKelvin, T_mCelsius, T_mFahrenheit);
 *
 * //Convert the raw relative humidity to milli %
 * H_Percent = ENS210_ConvertRawToPercentageH(H_Raw, 1000);
 * printf("H crc ok = %s\n", ENS210_IsCrcOk(H_Raw)  ? "yes" : "no");
 * printf("H valid = %s \n", ENS210_IsDataValid(H_Raw) ? "yes" : "no");
 * //Update the int32_t format specifier (%ld) based on platform word-size 
 * printf("H = %ld m%%\n", H_Percent);
 *
 * @endcode
 *
 *
 * Example 2 - Sample application code to measure relative humidity with error checking
 * ------------------------------------------------------------------------------------
 * @code
 * uint32_t H_Raw;
 * int32_t H_Percent;
 * int status;
 * bool i2cOk;
 *
 * i2cOk = true; //Start accumulating I2C transaction errors
 *
 * status = ENS210_SensRun_Set(ENS210_SENSRUN_H_MODE_SINGLE_SHOT);
 * i2cOk &= status==I2C_RESULT_OK;
 *
 * status = ENS210_SensStart_Set(ENS210_SENSSTART_H_START);
 * i2cOk &= status==I2C_RESULT_OK;
 *
 * WaitMsec(ENS210_T_H_CONVERSION_TIME_MS);
 *
 * status = ENS210_HVal_Get(&H_Raw);
 * i2cOk &= status==I2C_RESULT_OK;
 *
 * if( !i2cOk ) {
 *     printf("H i2c error\n")
 * } else if( !ENS210_IsCrcOk(H_Raw) ) {
 *     printf("H crc error\n")
 * } else if( !ENS210_IsDataValid(H_Raw) ) {
 *     printf("H data invalid\n")
 * } else {
 *     //Convert the raw relative humidity to milli %
 *     H_Percent = ENS210_ConvertRawToPercentageH(H_Raw,1000);
 *     //Update the int32_t format specifier (%ld) based on platform word-size 
 *     printf("H = %ld m%%\n", H_Percent);
 * }
 *
 * @endcode
 *
 * @{
 */
/*****************************************************************************
 * Types/enumerations/variables
 ****************************************************************************/
 /** ENS210 os-free driver version info */
#define ENS210_OSFREE_DRIVER_VERSION			2

/** ENS210 T and H conversion time in milliseconds. Refer to ENS210 data sheet for timing information. */
#define ENS210_T_H_CONVERSION_TIME_MS           130

/** ENS210 T conversion time in milliseconds */
#define ENS210_T_CONVERSION_TIME_MS             110

/** ENS210 Booting time in milliseconds. */
#define ENS210_BOOTING_TIME_MS                  2


/** ENS210 SysCtrl register: Low power enable */
#define ENS210_SYSCTRL_LOWPOWER_ENABLE          (1 << 0)
/** ENS210 SysCtrl register: Low power disable */
#define ENS210_SYSCTRL_LOWPOWER_DISABLE         (0 << 0)
/** ENS210 SysCtrl register: Reset enable */
#define ENS210_SYSCTRL_RESET_ENABLE             (1 << 7)
/** ENS210 SysCtrl register: Reset disable */
#define ENS210_SYSCTRL_RESET_DISABLE            (0 << 7)

/** ENS210 SysStat register: Standby or Booting mode */
#define ENS210_SYSSTAT_MODE_STANDBY             (0 << 0)
/** ENS210 SysStat register: Active mode */
#define ENS210_SYSSTAT_MODE_ACTIVE              (1 << 0)


/** ENS210 SensRun register: temperature single shot mode */
#define ENS210_SENSRUN_T_MODE_SINGLE_SHOT       (0 << 0)
/** ENS210 SensRun register: temperature continuous mode */
#define ENS210_SENSRUN_T_MODE_CONTINUOUS        (1 << 0)
/** ENS210 SensRun register: relative humidity single shot mode */
#define ENS210_SENSRUN_H_MODE_SINGLE_SHOT       (0 << 1)
/** ENS210 SensRun register: relative humidity continuous mode */
#define ENS210_SENSRUN_H_MODE_CONTINUOUS        (1 << 1)

/** ENS210  SensStart register: T sensor start */
#define ENS210_SENSSTART_T_START                (1 << 0)
/** ENS210  SensStart register: H sensor start */
#define ENS210_SENSSTART_H_START                (1 << 1)

/** ENS210  SensStop register: T sensor stop */
#define ENS210_SENSSTOP_T_STOP                  (1 << 0)
/** ENS210  SensStop register: H sensor stop */
#define ENS210_SENSSTOP_H_STOP                  (1 << 1)

/** ENS210  SensStat register: T sensor idle */
#define ENS210_SENSSTAT_T_STAT_IDLE             (0 << 0)
/** ENS210  SensStat register: T sensor active */
#define ENS210_SENSSTAT_T_STAT_ACTIVE           (1 << 0)
/** ENS210  SensStat register: H sensor idle */
#define ENS210_SENSSTAT_H_STAT_IDLE             (0 << 1)
/** ENS210  SensStat register: H sensor active */
#define ENS210_SENSSTAT_H_STAT_ACTIVE           (1 << 1)


/**
 * @brief    ENS210 ID block structure
 */
typedef struct ENS210_Ids_s 
{
    uint16_t    partId;             /*!< Part ID */
    uint8_t     uId[8];             /*!< Unique Identifier 8 bytes */
} ENS210_Ids_t;


/****************************************************************************
 * Function Prototypes
 ****************************************************************************/
/**
 * @brief   Set ENS210 SysCtrl register; enabling reset and/or low power.
 * @param   sysCtrl     :   Mask composed of  ENS210_SYSCTRL_xxx macros.
 * @return  The result code of the I2C transaction (untranslated).
 */
int ENS210_SysCtrl_Set(uint8_t sysCtrl);

/**
 * @brief   Get ENS210 SysCtrl register.
 * @param   sysCtrl     :   Pointer to receive value of the register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
int ENS210_SysCtrl_Get(uint8_t *sysCtrl);

/**
 * @brief   Get ENS210 SysStat register.
 * @param   sysStat     :   Pointer to receive value of the register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
int ENS210_SysStat_Get(uint8_t *sysStat);

/**
 * @brief   Set ENS210 SensRun register; set the run mode single shot/continuous for T and H sensors.
 * @param   sensRun     :   Mask composed of ENS210_SENSRUN_xxx macros.
 * @return  The result code of the I2C transaction (untranslated).
 */
int ENS210_SensRun_Set(uint8_t sensRun);

/**
 * @brief   Get ENS210 SensRun register.
 * @param   sensRun     :   Pointer to receive value of the register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
int ENS210_SensRun_Get(uint8_t *sensRun);

/**
 * @brief   Set ENS210 SensStart register; starts the measurement for T and/or H sensors.
 * @param   sensStart   :  Mask composed of ENS210_SENSSTART_xxx macros.
 * @return  The result code of the I2C transaction (untranslated).
 */
int ENS210_SensStart_Set(uint8_t sensStart);

/**
 * @brief   Set ENS210 SensStop register; stops the measurement for T and/or H sensors.
 * @param   sensStop    :   Mask composed of ENS210_SENSSTOP_xxx macros.
 * @return  The result code of the I2C transaction (untranslated).
 */
int ENS210_SensStop_Set(uint8_t sensStop);

/**
 * @brief   Get ENS210 SensStat register.
 * @param   sensStat    :   Pointer to receive value of the register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 */
int ENS210_SensStat_Get(uint8_t *sensStat);

/**
 * @brief   Get ENS210 TVal register; raw measurement data as well as CRC and valid indication.
 * @param   traw         :   Pointer to receive value of the register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    Use #ENS210_IsCrcOk and #ENS210_IsDataValid before using the measurement data.
 * @note    Use ENS210_ConvertRawToXXX to convert raw data to standard units.
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
int ENS210_TVal_Get(uint32_t *traw);

/**
 * @brief   Get ENS210 HVal register; raw measurement data as well as CRC and valid indication.
 * @param   hraw         :   Pointer to receive value of register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    Use #ENS210_IsCrcOk and #ENS210_IsDataValid before using the measurement data.
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
int ENS210_HVal_Get(uint32_t *hraw);

/**
 * @brief   Get ENS210 TVal and HVal registers; raw measurement data as well as CRC and valid indication.
 * @param   traw         :   Pointer to receive value of TVal register. Must not be null.
 * @param   hraw         :   Pointer to receive value of HVal register. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    Use #ENS210_IsCrcOk and #ENS210_IsDataValid before using the measurement data.
 * @note    If the return indicates I2C failure, the value of the out parameter is undefined.
 */
int ENS210_THVal_Get(uint32_t *traw, uint32_t *hraw);

/**
 * @brief   Verify the CRC of the raw temperature or relative humidity.
 * @param   raw         :  Raw temperature or relative humidity value to be verified (#ENS210_TVal_Get, #ENS210_HVal_Get, #ENS210_THVal_Get).
 * @return  True - Success,  False - Failure.
 * @note    This function can be used on raw T as well as raw H data (since they use the same format and CRC).
*/
bool ENS210_IsCrcOk(uint32_t raw);

/**
 * @brief   Verify data validity of the raw temperature or relative humidity.
 * @param   raw         :  Raw  temperature or relative humidity value to be verified (#ENS210_TVal_Get, #ENS210_HVal_Get, #ENS210_THVal_Get).
 * @return  True - Valid,  False - Invalid.
*/
bool ENS210_IsDataValid(uint32_t raw);

/**
 * @brief   Converts a raw temperature value into Kelvin.
 *          The output value is in Kelvin multiplied by parameter "multiplier".
 * @param   traw         :   The temperature value in the raw format (#ENS210_TVal_Get, #ENS210_THVal_Get).
 * @param   multiplier  :   The multiplication factor.
 * @return  The temperature value in 1/multiplier Kelvin.
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, 1000, or powers of 2.
 */
int32_t ENS210_ConvertRawToKelvin(uint32_t traw, int multiplier);

/**
 * @brief   Converts a raw temperature value into Celsius.
 *          The output value is in Celsius multiplied by parameter "multiplier".
 * @param   traw         :   The temperature value in the raw format (#ENS210_TVal_Get, #ENS210_THVal_Get).
 * @param   multiplier   :   The multiplication factor.
 * @return  The temperature value in 1/multiplier Celsius.
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, 1000, or powers of 2.
 */
int32_t ENS210_ConvertRawToCelsius(uint32_t traw, int multiplier);

/**
 * @brief   Converts a raw temperature value into Fahrenheit.
 *          The output value is in Fahrenheit multiplied by parameter "multiplier".
 * @param   traw         :   The temperature value in the raw format (#ENS210_TVal_Get, #ENS210_THVal_Get).
 * @param   multiplier   :   The multiplication factor of the converted temperature
 * @return  The temperature value in 1/multiplier Fahrenheit.
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, or powers of 2.
 */
int32_t ENS210_ConvertRawToFahrenheit(uint32_t traw, int multiplier);

/**
 * @brief   Converts a raw relative humidity value into human readable format.
 * @param   hraw         :   The relative humidity value in the raw format (#ENS210_HVal_Get, #ENS210_THVal_Get).
 * @param   multiplier   :   The multiplication factor of the converted relative humidity.
 * @return  The converted relative humidity value
 * @note    The multiplier should be between 1 and 1024 (inclusive) to avoid overflows.
 * @note    Typical values for multiplier are 1, 10, 100, 1000, or powers of 2.
 */
int32_t ENS210_ConvertRawToPercentageH(uint32_t hraw, int multiplier);

/**
 * @brief   Get ENS210 Part ID and UID.
 * @param   ids         :   Pointer to receive ids. Must not be null.
 * @return  The result code of the I2C transaction (untranslated).
 * @note    If this function retuns an error, it is suggested to reset the device to bring it to a known state.
 */
int ENS210_Ids_Get(ENS210_Ids_t *ids);


/**
 * @}
 */

#endif /* __ENS210_H_ */

