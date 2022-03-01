#ifdef __AVR__

#include "serial_avr0.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "irq.h"

namespace {

Cicada::Serial_AvrUart0 *H_AvrUart0 = 0;

ISR(USART0_TX_vect) {
    if (H_AvrUart0) H_AvrUart0->transferToAndFromBuffer();
}


ISR(USART0_RX_vect) {  
    if (bit_is_set(UCSR0A, UPE0)) {
        UDR0; // parity error!
    } else {
        if (H_AvrUart0) {
            H_AvrUart0->RxChar = UDR0;
            H_AvrUart0->RxInt = true;
            H_AvrUart0->transferToAndFromBuffer();
        }
    }
}

}

using namespace Cicada;

Serial_AvrUart0::Serial_AvrUart0(char* readBuffer, char* writeBuffer, Size readBufferSize, Size writeBufferSize):
BufferedSerial(readBuffer, writeBuffer, readBufferSize, writeBufferSize) { }

Serial_AvrUart0::Serial_AvrUart0(char* readBuffer, char* writeBuffer, Size bufferSize):
BufferedSerial(readBuffer, writeBuffer, bufferSize) { }

Serial_AvrUart0::~Serial_AvrUart0() {
    close();
}

bool Serial_AvrUart0::setSerialConfig(uint32_t baudRate, uint8_t dataBits) {
    UCSR0C = 0x06; //dataBits;
    UCSR0A = 1 << U2X0;
    UBRR0H = ((F_CPU / 4 / baudRate - 1) / 2) >> 8;
    UBRR0L = ((F_CPU / 4 / baudRate - 1) / 2) & 0xFF;
    return true;
}

bool Serial_AvrUart0::open() {
    H_AvrUart0 = this;
    RxInt = false;
    UCSR0B = ((1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0) | (1<<TXCIE0));
    // UCSR1B |= (1 << RXEN1);
    // UCSR1B |= (1 << TXEN1);
    // UCSR1B |= (1 << RXCIE1);
    // UCSR1B |= (1 << TXCIE1);
    eEnableInterrupts();
    return true;
}

void Serial_AvrUart0::close() {
    UCSR0B = 0;
}

bool Serial_AvrUart0::isOpen() {
    return UCSR0B != 0;
}

const char* Serial_AvrUart0::portName() const {
    return 0;
}

bool Serial_AvrUart0::rawRead(uint8_t &data) {
    if (RxInt) {
        RxInt = false;
        data = RxChar;
        return true;
    }
    return false;
}

bool Serial_AvrUart0::rawWrite(uint8_t data) {
    UDR0 = data;
    return true;
}

void Serial_AvrUart0::startTransmit() {
    while (!(UCSR0A & (1<<UDRE0)));
    transferToAndFromBuffer();
}

bool Serial_AvrUart0::writeBufferProcessed() const {
    return _writeBuffer.bytesAvailable() == 0;
}



#endif
