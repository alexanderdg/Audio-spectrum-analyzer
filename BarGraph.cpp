#include "BarGraph.h"
#include "mbed.h"

uint16_t BarGraph::lut[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
unsigned char BarGraph::lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

BarGraph::BarGraph(PinName Data, PinName Clk, PinName Cs): spi(Data, NC, Clk), cs(Cs), pc(USBTX, USBRX) {
    pc.printf("Making a BarGraph\n");
    cs = 1;
    spi.format(16,3);
    spi.frequency(1000000);
    levels[16];
    init();
}

bool BarGraph::init(void) {
    sendCommand(0x0900);
    sendCommand(0x0A0F);
    sendCommand(0x0B07);
    sendCommand(0x0C01);
    return true;
}

bool BarGraph::selfTestOn(void) {
    sendCommand(0x0F01);
    return true;   
}

bool BarGraph::selfTestOff(void) {
    sendCommand(0x0F00);
    return true;   
} 

bool BarGraph::setDecade(uint8_t decade, uint8_t level) {
    levels[decade] = level;
    return true;   
}

uint16_t BarGraph::decode(uint8_t value) {
    uint16_t temp = 0;
    for(int i = 1; i <= value; i++) {
        temp = temp + lut[i-1];   
    }
    return temp;   
}

bool BarGraph::refresh(void) {
   for(int i = 0; i < 8; i++) {
        uint8_t u1 = decode(levels[i]) & 0xFF;
        uint8_t u3 = decode(levels[7-i]) >> 8;
        uint8_t u2 = decode(levels[i+8]) & 0xFF;
        uint8_t u4 = decode(levels[15-i]) >> 8;
        uint8_t ru1 = reverse(u1);
        uint8_t ru2 = reverse(u2);
        uint8_t ru3 = reverse(u3);
        uint8_t ru4 = reverse(u4);
        //pc.printf("%d, U1:%u  U2:%u  U3:%u  U4:%u \n", i+1 << 8, u1, u2, ru3, ru4);
        sendCommand(((i+1) << 8) + ru1, ((i+1) << 8) + ru2, ((i+1) << 8) + u4, ((i+1) << 8) + u3);
   }
   return true;     
}

bool BarGraph::intensity(uint8_t level) {
    if(level <= 15) {
        pc.printf("setIntensity %d\n", level);
        sendCommand(((0x0A) << 8) + ((level*2)+1));
        return true;
    }
    else {
        return false;
    }  
}

uint8_t BarGraph::reverse(uint8_t b) {
   // Reverse the top and bottom nibble then swap them.
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

bool BarGraph::sendCommand(int command) {
    sendCommand(command, command, command, command);   
}

bool BarGraph::sendCommand(int command1,int command2,int command3,int command4){
    //pc.printf("sendCommand\n");
    cs = 0;
    wait_us(1);
    spi.write(command4);
    spi.write(command3);
    spi.write(command2);
    spi.write(command1);
    //wait_us(10);
    cs = 1;
    //wait_us(10);
    return true;   
}