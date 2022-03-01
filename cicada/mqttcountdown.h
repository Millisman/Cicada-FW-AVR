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

#include "defines.h"
#include <stdint.h>

namespace Cicada {

/*!
 * \class MQTTCountdown
 *
 * Implements the countdown ad required by the Paho MQTT client.
 *
 * *NOTE:* Due to limitations of the MQTTClient API, it's not possible
 * to pass any arguments to the constructor. So, this class needs access
 * to the global timer tick function.
 */

class MQTTCountdown
{
  public:
    MQTTCountdown();
    MQTTCountdown(int ms);

    bool expired();
    void countdown_ms(int ms);
    void countdown(int seconds);
    int left_ms();

  private:
    E_TICK_TYPE _endTime;
};
}


