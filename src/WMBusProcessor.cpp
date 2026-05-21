#include "Arduino.h"
#include "WMBusProcessor.h"

WMBusProcessor::WMBusProcessor(int new_channel, bool diagnostics_enabled) {

    byte_counter = 0;
    byte_index = 0;

    processor_state = AWAITING_NEW_PACKET;
    diagnostics = diagnostics_enabled;
    channel = new_channel;

}

bool WMBusProcessor::isPacketReady() {
  if (processor_state == PACKET_READY) {
    return true;
  } else {
    return false;
  }
}

char * WMBusProcessor::getPacket() {
  processor_state = AWAITING_NEW_PACKET;
  return packet;
}

void WMBusProcessor::processByte(char new_byte) {
  // assembles Wireless M-Bus packets from individual bytes
  // once we have a complete packet, we give it a timestamp and store it

  if (processor_state == AWAITING_NEW_PACKET) {
    // we have received a byte whilst expecting a new packet
    // therefore this byte is the first byte of the packet, which tells us the number of bytes to follow

  if(diagnostics == true) {
    Serial.print("#New packet - size: ");
    char packet_size[8];

    sprintf(packet_size, "%d", new_byte);

    Serial.print(packet_size);
    Serial.println(" bytes");
  } 

    // Only accept a new packet if we have a 22 byte packet, as that's what Cryoegg-2023 produces
    // EXCEPT! sometimes the Radiocrafts receiver gets into a funny mode and gives us 25 byte packets instead, so we need to cope with these as well
    if (new_byte != 22 && new_byte != 25) {
      Serial.println("#Not a valid Cryoegg packet");
 
      return;
    }

    byte_counter = new_byte;

    packet[0] = new_byte;

    byte_index  = 1; // byte_index is the array position that we will write into for the next byte

    processor_state = ASSEMBLING_PACKET; // change state      
  } 
  else if (processor_state == ASSEMBLING_PACKET) {
    // add the new byte to the end of the existing packet

    if (byte_index == 1) {
      Serial.print("#Second byte: ");
      Serial.println(new_byte, HEX);
      if (new_byte != 0x44) { // 68) { // second byte of packet should be 0x44 which is 68 - and this is true regardless of whether we got a 19-byte or 25-byte packet
        processor_state = AWAITING_NEW_PACKET; // reset state machine and try again
         Serial.print("#Not a valid Cryoegg packet - second byte wrong - second byte is ");
         Serial.println(new_byte);
 
        return;
      }
    }

    packet[byte_index] = new_byte;

    byte_index++;
    byte_counter--; 

    if (byte_counter == 0) {
      // we've reached the end of the packet


      // flag that we have a packet ready

      processor_state = PACKET_READY; 
      
    }
    
  }
  
}

