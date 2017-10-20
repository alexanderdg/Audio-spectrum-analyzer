#ifndef BARGRAPH_H
#define BARGRAPH_H

#include "mbed.h"

class BarGraph {
    
    public:
        BarGraph(PinName Data, PinName Clk, PinName Cs);
        
        //methods to control each decade
        bool setDecade(uint8_t decade, uint8_t level);
        
        bool selfTestOn(void);
        
        bool selfTestOff(void);
        
        bool refresh(void);
    
        bool intensity(uint8_t level);
            
    private:
        SPI spi;
        DigitalOut cs;
        Serial pc;
        uint8_t levels[];
        uint8_t v_intensity;
        uint16_t static  lut[];
        unsigned char static lookup[];
        
        bool init(void);
        
        uint8_t reverse(uint8_t n);
        
        uint16_t decode(uint8_t value);
        
        bool sendCommand(int command);
        
        bool sendCommand(int command1, int command2, int command3,int command4);
        
    
};

#endif