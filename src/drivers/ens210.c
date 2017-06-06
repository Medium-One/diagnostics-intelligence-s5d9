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


#include "ens210_platform.h"
#include "ens210.h"
#include <string.h>
#include <assert.h>

/** @brief ENS210 driver implementation
 */

/*****************************************************************************
 * Private macros and functions
 ****************************************************************************/
/* Register address */
#define ENS210_REG_PART_ID                      0x00
#define ENS210_REG_UID                          0x04
#define ENS210_REG_SYS_CTRL                     0x10
#define ENS210_REG_SYS_STAT                     0x11
#define ENS210_REG_SENS_RUN                     0x21
#define ENS210_REG_SENS_START                   0x22
#define ENS210_REG_SENS_STOP                    0x23
#define ENS210_REG_SENS_STAT                    0x24
#define ENS210_REG_T_VAL                        0x30
#define ENS210_REG_H_VAL                        0x33

/** I2C slave address */
#define ENS210_I2C_SLAVE_ADDRESS                0x43

/** Mask to extract 16-bit data from raw T and H values */
#define ENS210_T_H_MASK                         0xFFFFU


/** Simplification macro, implementing integer division with simple rounding to closest number
 *  It supports both positive and negative numbers, but ONLY positive divisors */
#define IDIV(n,d)                               ((n)>0 ? ((n)+(d)/2)/(d) : ((n)-(d)/2)/(d))

#define CRC7WIDTH                               7       //7 bits CRC has polynomial of 7th order (has 8 terms)
#define CRC7POLY                                0x89    //The 8 coefficients of the polynomial
#define CRC7IVEC                                0x7F    //Initial vector has all 7 bits high

#define DATA7WIDTH                              17
#define DATA7MASK                               ((1UL << DATA7WIDTH) - 1)     //0b 1 1111 1111 1111 1111
#define DATA7MSB                                (1UL << (DATA7WIDTH - 1))     //0b 1 0000 0000 0000 0000

/** When the ENS210 is soldered a correction on T needs to be applied (see application note). 
 *  Typically the correction is 50mK. Units for raw T is 1/64K. */
#define ENS210_TRAW_SOLDERCORRECTION            (50*64/1000)

//Compute the CRC-7 of 'val' (which should only have 17 bits)
static uint32_t ENS210_ComputeCRC7(uint32_t val)
{
    //Setup polynomial
    uint32_t pol= CRC7POLY;
    
    //Align polynomial with data
    pol = pol << (DATA7WIDTH-CRC7WIDTH-1);
    
    //Loop variable (indicates which bit to test, start with highest)
    uint32_t bit = DATA7MSB;
    
    //Make room for CRC value
    val = val << CRC7WIDTH;
    bit = bit << CRC7WIDTH;
    pol = pol << CRC7WIDTH;
    
    //Insert initial vector
    val |= CRC7IVEC;
    
    //Apply division until all bits done
    while( bit & (DATA7MASK<<CRC7WIDTH) )
    {
        if( bit & val )
        {
            val ^= pol;
        }
        bit >>= 1;
        pol >>= 1;
    }
    return val;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
//Set ENS210 SysCtrl register; enabling reset and/or low power
int ENS210_SysCtrl_Set(uint8_t sysCtrl)
{
    uint8_t wBuf[] = {ENS210_REG_SYS_CTRL, sysCtrl};
	
    assert((sysCtrl & ~(ENS210_SYSCTRL_LOWPOWER_ENABLE | ENS210_SYSCTRL_RESET_ENABLE )) == 0);

    return I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}


//Get ENS210 SysCtrl register
int ENS210_SysCtrl_Get(uint8_t *sysCtrl)
{
    uint8_t wBuf[] = {ENS210_REG_SYS_CTRL};

    assert(sysCtrl != NULL);

    return I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, sysCtrl, sizeof *sysCtrl);
}

//Get ENS210 SysStat register.
int ENS210_SysStat_Get(uint8_t *sysStat)
{
    uint8_t wBuf[] = {ENS210_REG_SYS_STAT};

    assert(sysStat != NULL);

    return I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, sysStat, sizeof *sysStat);
}

//Set ENS210 SensRun register; set the run mode single shot/continuous for T and H sensors.
int ENS210_SensRun_Set(uint8_t sensRun)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_RUN, sensRun};

    assert((sensRun & ~(ENS210_SENSRUN_T_MODE_CONTINUOUS | ENS210_SENSRUN_H_MODE_CONTINUOUS)) == 0);

    return I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}

//Get ENS210 SensRun register
int ENS210_SensRun_Get(uint8_t *sensRun)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_RUN};
    
    assert(sensRun != NULL);

    return I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, sensRun, sizeof *sensRun);
}

//Set ENS210 SensStart register; starts the measurement for T and/or H sensors.
int ENS210_SensStart_Set(uint8_t sensStart)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_START, sensStart};

    assert((sensStart & ~(ENS210_SENSSTART_T_START | ENS210_SENSSTART_H_START)) == 0);

    return I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}

//Set ENS210 SensStop register; stops the measurement for T and/or H sensors.
int ENS210_SensStop_Set(uint8_t sensStop)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_STOP, sensStop};

    assert((sensStop & ~(ENS210_SENSSTOP_T_STOP | ENS210_SENSSTOP_H_STOP)) == 0);

    return I2C_Write(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf);
}


//Get ENS210 SensStat register.
int ENS210_SensStat_Get(uint8_t *sensStat)
{
    uint8_t wBuf[] = {ENS210_REG_SENS_STAT};

    assert(sensStat != NULL);

    return I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, sensStat, sizeof *sensStat);
}


//Get ENS210 TVal register; raw measurement data as well as CRC and valid indication
int ENS210_TVal_Get(uint32_t *traw)
{
    uint8_t rBuf[3];
    uint8_t wBuf[] = {ENS210_REG_T_VAL};
    int status;

    assert(traw != NULL);

    status = I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);

    *traw = ((uint32_t)rBuf[2]) << 16 | ((uint32_t)rBuf[1]) << 8 | (uint32_t)rBuf[0];

    return status;
}

//Get ENS210 HVal register; raw measurement data as well as CRC and valid indication
int ENS210_HVal_Get(uint32_t *hraw)
{
    uint8_t rBuf[3];
    uint8_t wBuf[] = {ENS210_REG_H_VAL};
    int status;

    assert(hraw != NULL);

    status = I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);

    *hraw = ((uint32_t)rBuf[2]) << 16 | ((uint32_t)rBuf[1]) << 8 | ((uint32_t)rBuf[0]) << 0;

    return status;
}

//Get ENS210 TVal and Hval registers; raw measurement data as well as CRC and valid indication
int ENS210_THVal_Get(uint32_t *traw, uint32_t *hraw)
{
    uint8_t rBuf[6];
    uint8_t wBuf[] = {ENS210_REG_T_VAL};
    int status;

    assert((traw != NULL) && (hraw != NULL));
    
    // Read 6 bytes starting from ENS210_REG_T_VAL 
    status = I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);

    *traw = ((uint32_t)rBuf[2]) << 16 | ((uint32_t)rBuf[1]) << 8 | (uint32_t)rBuf[0];
    *hraw = ((uint32_t)rBuf[5]) << 16 | ((uint32_t)rBuf[4]) << 8 | (uint32_t)rBuf[3];

    return status;    
}

//Verify the CRC
bool ENS210_IsCrcOk(uint32_t raw)
{
    uint32_t crc, data;

	assert(raw <= 0xffffffUL);

    //Extract 7-bit CRC(Bit-17 to Bit-23)
    crc =  (raw >> 17) & 0x7F;

    //Get the raw T/H and data valid indication.
    data =  raw & 0x1ffff;

    return  ENS210_ComputeCRC7(data) == crc;
}

//Check the Data Valid Bit
bool ENS210_IsDataValid(uint32_t raw)
{
	assert(raw <= 0xffffffUL);

    //Bit-16 is data valid bit. It will be set if data is valid
    return (raw & (1UL << 16)) != 0;
}

//Convert raw temperature to Kelvin
//The output value is in Kelvin multiplied by parameter "multiplier"
int32_t ENS210_ConvertRawToKelvin(uint32_t traw, int multiplier)
{
    int32_t t;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw temperature
    t = traw & ENS210_T_H_MASK;
    
    //Compensate for soldering effect
    t-= ENS210_TRAW_SOLDERCORRECTION;

    //We must compute and return m*K
    //where m is the multiplier, R the raw value and K is temperature in Kelvin.
    //K=R/64 (since raw has format 10.6).
    //m*K =  m*R/64
    return IDIV(t*multiplier, 64);
}

//Convert raw temperature to Celsius
//The output value is in Celsius multiplied by parameter "multiplier"
int32_t ENS210_ConvertRawToCelsius(uint32_t traw, int multiplier)
{
    int32_t t;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw temperature
    t = traw & ENS210_T_H_MASK;
    
    //Compensate for soldering effect
    t-= ENS210_TRAW_SOLDERCORRECTION;

    //We must compute and return m*C
    //where m is the multiplier, R the raw value and K, C, F temperature in various units.
    //We use C=K-273.15 and K=R/64 (since raw has format 10.6).
    //m*C = m*(K-273.15) = m*K - 27315*m/100 = m*R/64 - 27315*m/100

    return IDIV(t*multiplier, 64) - IDIV(27315L*multiplier, 100);
}

//Convert raw temperature to Fahrenheit
//The output value is in Fahrenheit multiplied by parameter "multiplier"
int32_t ENS210_ConvertRawToFahrenheit(uint32_t traw, int multiplier)
{
    int32_t t;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw temperature
    t = traw & ENS210_T_H_MASK;
    
    //Compensate for soldering effect
    t-= ENS210_TRAW_SOLDERCORRECTION;

    //We must compute and return m*F
    //where m is the multiplier, R the raw value and K, C, F temperature in various units.
    //We use F=1.8*(K-273.15)+32 and K=R/64 (since raw has format 10.6).

    //m*F = m*(1.8*(K-273.15)+32) = m*(1.8*K-273.15*1.8+32) = 1.8*m*K-459.67*m = 9*m*K/5 - 45967*m/100 = 9*m*R/320 - 45967*m/100
    return IDIV(9*multiplier*t, 320) - IDIV(45967L*multiplier, 100);
    
    //The first multiplication stays below 32 bits (tRaw:16, multiplier:11, 9:4)
    //The second  multiplication stays below 32 bits (multiplier:10, 45967:16)
}

//Convert raw relative humidity to readable format
//The output value is in % multiplied by parameter "multiplier"
int32_t ENS210_ConvertRawToPercentageH(uint32_t hraw, int multiplier)
{
    int32_t h;

    assert((1 <= multiplier) && (multiplier <= 1024));

    //Get the raw relative humidity
    h = hraw & ENS210_T_H_MASK;

    //As raw format is 7.9, to obtain the relative humidity, it must be divided by 2^9
    return IDIV(h*multiplier, 512);
}

// Get ENS210 Part ID and UID.
int ENS210_Ids_Get(ENS210_Ids_t *ids)
{
    uint8_t rBuf[12];
    uint8_t wBuf[] = {ENS210_REG_PART_ID};
    int status;

    assert(ids != NULL);

    // Special procedure needed to read ID's: put device in high power (see datasheet)
    // Set the system in Active mode
    status = ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_DISABLE);
    if (status != I2C_RESULT_OK) goto return_status;

    // Wait for sensor to go to active mode
    WaitMsec(ENS210_BOOTING_TIME_MS);

    // Get the id's
    status = I2C_Read(ENS210_I2C_SLAVE_ADDRESS, wBuf, sizeof wBuf, rBuf, sizeof rBuf);
    if (status != I2C_RESULT_OK) goto return_status;

    // Copy id's (hw gives partid in little-endian format)
    ids->partId = (uint16_t)(((uint32_t)rBuf[1]) << 8 | ((uint32_t)rBuf[0]) << 0);
    memcpy(&ids->uId[0], &rBuf[4], 8);

    // Go back to low power mode
    status = ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_ENABLE);
    if (status != I2C_RESULT_OK) goto return_status;

    // Signal success
    return I2C_RESULT_OK;

return_status:
    // Make an attempt to restore low-power
    ENS210_SysCtrl_Set(ENS210_SYSCTRL_LOWPOWER_ENABLE);
    // Return original I2C error
    return status;
}

