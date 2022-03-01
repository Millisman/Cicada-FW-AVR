#pragma once

#ifdef __AVR__

#include "bufferedserial.h"

namespace Cicada {

class Serial_AvrUart0 : public BufferedSerial {
  public:
      Serial_AvrUart0(char* readBuffer, char* writeBuffer, Size bufferSize);
      Serial_AvrUart0(char* readBuffer, char* writeBuffer, Size readBufferSize, Size writeBufferSize);
      ~Serial_AvrUart0();
    
    virtual bool open() override;
    virtual bool isOpen() override;
    virtual bool setSerialConfig(uint32_t baudRate, uint8_t dataBits) override;
    virtual void close() override;
    virtual const char* portName() const override;
    virtual bool rawRead(uint8_t& data) override;
    virtual bool rawWrite(uint8_t data) override;
    virtual void startTransmit() override;
    virtual bool writeBufferProcessed() const override;
    bool    RxInt;
    uint8_t RxChar;
private:
    // Private constructors to avoid copying
    Serial_AvrUart0(const Serial_AvrUart0&);
    Serial_AvrUart0& operator=(const Serial_AvrUart0&);
};

}

#endif
