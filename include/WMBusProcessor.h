#ifndef WMBusProcessor_h
#define WMBusProcessor_h

#include "Arduino.h"

class WMBusProcessor {
public:
    WMBusProcessor(int new_channel, bool diagnostics_enabled);
    void processByte(char new_byte);
    bool isPacketReady();
    char * getPacket();
private:
    enum packet_state {AWAITING_NEW_PACKET, ASSEMBLING_PACKET, PACKET_READY};
    int byte_counter = 0;
    int byte_index = 0;
    char packet[128]= ""; // maximum packet size 128 bytes - we're only expect 19 bytes
    bool diagnostics = false;
    int channel = 1;  // channel number

    enum packet_state processor_state = AWAITING_NEW_PACKET;
};

#endif