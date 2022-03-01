#include "cicada/serial_avr0.h"
#include "cicada/serial_avr1.h"
#include "cicada/scheduler.h"
#include "cicada/tick.h"
#include "printf/printf.h"
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>


#define MAKEPIN(a, n, d) Pin(&(PIN##a), (1 << n), Pin::PIN_##d)
#define _AVR_REG(addr) (*reinterpret_cast<volatile uint8_t *>(addr))
#define _PIN _AVR_REG(reg_pin)
#define _DDR _AVR_REG(reg_pin + 1)
#define _PORT _AVR_REG(reg_pin + 2)


class Pin {
public:
    enum PinDirection { PIN_IN, PIN_IN_PULLUP, PIN_OUT };
    Pin(volatile uint8_t *reg_pin, const uint8_t b, const PinDirection d):
        reg_pin(reg_pin), b(b) {
        _DDR &= ~b;
        
        if (d == PIN_OUT) _DDR |= b;
        else if (d == PIN_IN_PULLUP)
            _PORT |= b;
    }
    Pin &operator=(const uint8_t rhs) {
        if (rhs)
            _PORT |= b;
        else
            _PORT &= ~b;
        
        return *this;
    }
    bool operator!() const { return (_PIN & b) ? false : true; }   
private:
    volatile uint8_t *const reg_pin;
    const uint8_t b;
};



using namespace Cicada;

void yieldFunction(void* sched)
{
    ((Scheduler*)sched)->runTask();
}


class PPPpow : public Task {
    Pin p;
public:
    PPPpow(): p(MAKEPIN(D, 5, OUT)) { }
    virtual void run() {
        E_BEGIN_TASK
        p = 0;
        E_REENTER_DELAY(1);
        p = 1;
        E_REENTER_DELAY(1500);
        E_END_TASK
    }
};


class SerialTask : public Task {
public:
    SerialTask(IBufferedSerial& serial) : m_serial(serial) {}
    
    virtual void run() {        
        E_BEGIN_TASK
        
        if (!m_serial.setSerialConfig(115200, 8)) {
            // TODO: Error 
        }
        
        if (!m_serial.open()) {
            // TODO: Error
        }
        
        for (;;) {
            {
                const char* send_str = "AT\r\n";
                printf("write AT...\n");
                m_serial.write((const uint8_t*)send_str, strlen(send_str));
            }
            
            //E_REENTER_COND_DELAY(m_serial.bytesAvailable(), 100);
            if (m_serial.bytesAvailable())
            {
                E_REENTER_DELAY(100);
                char buf[32] = {0};
                m_serial.read((uint8_t*)buf, 31);
                printf("%s", buf);
            }
            
            E_REENTER_DELAY(500);
            printf(".");
        }
        
        m_serial.close();
        
        E_END_TASK
    }
    
private:
    IBufferedSerial& m_serial;
};

Serial_AvrUart0 *cout_ser = 0;


void _putchar(char c) {
    cout_ser->write(c);
}

int main(int argc, char* argv[]) {
    
    const uint16_t bs = 1504;
    char rb0[bs];
    char tb0[bs];
    Serial_AvrUart0 ser_out(rb0, tb0, bs);
    ser_out.setSerialConfig(115200, 0);
    ser_out.open();
    cout_ser = &ser_out;
    
    const uint16_t bufferSize = 1504;
    char readBuffer[bufferSize];
    char writeBuffer[bufferSize];
    
    Serial_AvrUart1 serial(readBuffer, writeBuffer, bufferSize);
    SerialTask task(serial);
    PPPpow pp;
    
    
    Task* taskList[] = { &pp, &task, NULL };
    printf("shed\n");
    Scheduler s(&eTickFunction, taskList);
    s.start();
}

