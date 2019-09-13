#pragma once

/***************************************************************************
        Distributed with a free-will license.
        Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
        MCP3428

****************************************************************************/

#include <Wire.h>
#include <math.h>

class CASARIA_MCP3428
{
    protected:
        long raw_adc;
        uint8_t SPS;
        bool MODE;
        uint8_t i;
        uint8_t testvar;
        uint8_t config;
        uint8_t GAIN;
        uint8_t no_of_bytes;
        uint8_t data[3];
    
    public:
        uint8_t devAddr;
        CASARIA_MCP3428(uint8_t i2cAddress);
        ~CASARIA_MCP3428();
        bool testConnection(void);
        void SetConfiguration(uint8_t channel, uint8_t resolution, bool mode, uint8_t PGA);
        bool CheckConversion();
        long readADC();


    private:
};
