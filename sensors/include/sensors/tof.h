#ifndef _TOFLIB_H_
#define _TOFLIB_H_
//
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

//
// Read the model and revision of the
// tof sensor
//
typedef enum vcselperiodtype { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
#define ADDRESS_DEFAULT 0b0101001

class VL53L0X
{
public:
    VL53L0X(void)
    : address(ADDRESS_DEFAULT)
    , io_timeout(0) // no timeout
    , did_timeout(false)
    {
    }

    int tofGetModel(int *model, int *revision);

    //
    // Opens a file system handle to the I2C device
    // sets the device continous capture mode
    //
    int tofInit(int iChan, int iAddr, int bLongRange);

    //
    // Read the current distance in mm
    //
    int tofReadDistance(void);

    void setAddress(uint8_t new_addr);
    inline uint8_t getAddress(void) { return address; }

    int initSensor(int bLongRangeMode);

    void writeReg(unsigned char ucAddr, unsigned char ucValue);
    void writeReg16(unsigned char ucAddr, unsigned short usValue);
    void writeMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount);
    void writeRegList(unsigned char *ucList);
    unsigned char readReg(unsigned char ucAddr);
    unsigned short readReg16(unsigned char ucAddr);
    void readMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount);

    bool setSignalRateLimit(float limit_Mcps);
    float getSignalRateLimit(void);

    int setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget(void);

    int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
    uint8_t getVcselPulsePeriod(vcselPeriodType type);

    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous(void);
    uint16_t readRangeContinuousMillimeters(void);
    uint16_t readRangeSingleMillimeters(void);

    inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    inline uint16_t getTimeout(void) { return io_timeout; }
    bool timeoutOccurred(void);

private:
    struct SequenceStepEnables
    {
      bool tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts
    {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    uint8_t address;
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;

    int getSpadInfo(unsigned char *pCount, unsigned char *pTypeIsAperture);

    void getSequenceStepEnables(SequenceStepEnables * enables);
    void getSequenceStepTimeouts(uint8_t enables, SequenceStepTimeouts * timeouts);

    int performSingleRefCalibration(uint8_t vhv_init_byte);

    static uint16_t decodeTimeout(uint16_t reg_val);
    static uint16_t encodeTimeout(uint16_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
};


#endif // _TOFLIB_H
