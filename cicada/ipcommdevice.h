/*
 * E-Lib
 * Copyright (C) 2019 EnAccess
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#pragma once

#include "bufferedserial.h"
#include "circularbuffer.h"
#include "iipcommdevice.h"
#include "task.h"

#define CONNECT_PENDING (1 << 0)
#define RESET_PENDING (1 << 1)
#define DATA_PENDING (1 << 2)
#define DISCONNECT_PENDING (1 << 3)
#define IP_CONNECTED (1 << 4)
#define LINE_READ (1 << 5)
#define SERIAL_LOCKED (1 << 6)

namespace Cicada {

class IPCommDevice : public IIPCommDevice, public Task
{
  public:
    /*
     * \param readBuffer user supplied buffer for data arriving from the device
     * \param writeBuffer user supplied buffer to store data before being sent
     * to the device
     * \param bufferSize size of each buffer. Both buffers have the same size
     */
    IPCommDevice(uint8_t* readBuffer, uint8_t* writeBuffer, Size bufferSize);
    IPCommDevice(
        uint8_t* readBuffer, uint8_t* writeBuffer, Size readBufferSize, Size writeBufferSize);
    virtual ~IPCommDevice() {}

    virtual void setHostPort(const char* host, uint16_t port, ConnectionType type = TCP);
    virtual bool connect();
    virtual void disconnect();
    virtual bool isConnected();
    virtual bool isIdle();
    virtual Size bytesAvailable() const;
    virtual Size spaceAvailable() const;
    virtual Size read(uint8_t* data, Size maxSize);
    virtual Size write(const uint8_t* data, Size size);
    virtual bool writeBufferProcessed() const;

  protected:
    enum ConnectState {
        notConnected,
        intermediate,
        connected,
        transmitting,
        receiving,
        generalError,
        dnsError,
    };

    CircularBuffer<uint8_t> _readBuffer;
    CircularBuffer<uint8_t> _writeBuffer;
    ConnectionType _type;
    const char* _host;
    uint16_t _port;
    uint8_t _stateBooleans;
    ConnectState _connectState;
    const char* _waitForReply;
};
}

