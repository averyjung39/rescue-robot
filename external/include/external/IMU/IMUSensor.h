#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "external/IMU/i2c-dev.h"
#include "external/IMU/L3G.h"
#include "external/IMU/LSM303.h"

namespace IMU {
    void readBlock(uint8_t command, uint8_t size, uint8_t *data);
    void selectDevice(int file, int addr);
    void readACC(int *a);
    void readMAG(int *m);
    void readGYR(int *g);
    void writeAccReg(uint8_t reg, uint8_t value);
    void writeMagReg(uint8_t reg, uint8_t value);
    void writeGyrReg(uint8_t reg, uint8_t value);
    void enableIMU();
}