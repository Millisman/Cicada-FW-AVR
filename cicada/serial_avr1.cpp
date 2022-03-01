#ifdef __AVR__

#include "serial_avr1.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "irq.h"

#define FLAG_ISOPEN (1 << 0)

namespace {

Cicada::Serial_AvrUart1 *H_AvrUart1 = 0;

ISR(USART1_TX_vect) {
    if (H_AvrUart1) H_AvrUart1->transferToAndFromBuffer();
}


ISR(USART1_RX_vect) {  
    if (bit_is_set(UCSR1A, UPE1)) {
        UDR1; // parity error!
    } else {
        if (H_AvrUart1) {
            H_AvrUart1->RxChar = UDR1;
            H_AvrUart1->RxInt = true;
            H_AvrUart1->transferToAndFromBuffer();
        }
    }
}

}

using namespace Cicada;

Serial_AvrUart1::Serial_AvrUart1(char* readBuffer, char* writeBuffer, Size readBufferSize, Size writeBufferSize):
BufferedSerial(readBuffer, writeBuffer, readBufferSize, writeBufferSize) { }

Serial_AvrUart1::Serial_AvrUart1(char* readBuffer, char* writeBuffer, Size bufferSize):
BufferedSerial(readBuffer, writeBuffer, bufferSize) { }

Serial_AvrUart1::~Serial_AvrUart1() {
    close();
}

bool Serial_AvrUart1::setSerialConfig(uint32_t baudRate, uint8_t dataBits) {
    UCSR1C = 0x06; //dataBits;
    UCSR1A = 1 << U2X1;
    UBRR1H = ((F_CPU / 4 / baudRate - 1) / 2) >> 8;
    UBRR1L = ((F_CPU / 4 / baudRate - 1) / 2) & 0xFF;
    return true;
}

bool Serial_AvrUart1::open() {
    H_AvrUart1 = this;
    RxInt = false;
    UCSR1B = ((1<<TXEN1) | (1<<RXEN1) | (1<<RXCIE1) | (1<<TXCIE1));
    // UCSR1B |= (1 << RXEN1);
    // UCSR1B |= (1 << TXEN1);
    // UCSR1B |= (1 << RXCIE1);
    // UCSR1B |= (1 << TXCIE1);
    eEnableInterrupts();
    return true;
}

void Serial_AvrUart1::close() {
    UCSR1B = 0;
}

bool Serial_AvrUart1::isOpen() {
    return UCSR1B != 0;
}

const char* Serial_AvrUart1::portName() const {
    return 0;
}

bool Serial_AvrUart1::rawRead(uint8_t &data) {
    if (RxInt) {
        RxInt = false;
        data = RxChar;
        return true;
    }
    return false;
}

bool Serial_AvrUart1::rawWrite(uint8_t data) {
    UDR1 = data;
    return true;
}

void Serial_AvrUart1::startTransmit() {
    while (!(UCSR1A & (1<<UDRE1)));
    transferToAndFromBuffer();
}

bool Serial_AvrUart1::writeBufferProcessed() const {
    return _writeBuffer.bytesAvailable() == 0;
}



#endif
