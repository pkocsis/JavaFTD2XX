/*
 * The MIT License
 *
 * Copyright 2011 Peter Kocsis <p. kocsis. 2. 7182 at gmail.com>.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.ftdi;

/**
 * Bit Modes (see FT_SetBitMode)
 * @author Peter Kocsis <p. kocsis. 2. 7182 at gmail.com>
 */
public enum BitModes {

    FT_BITMODE_RESET(0x00),
    FT_BITMODE_ASYNC_BITBANG(0x01),
    FT_BITMODE_MPSSE(0x02),
    FT_BITMODE_SYNC_BITBANG(0x04),
    FT_BITMODE_MCU_HOST(0x08),
    FT_BITMODE_FAST_SERIAL(0x10),
    FT_BITMODE_CBUS_BITBANG(0x20),
    FT_BITMODE_SYNC_FIFO(0x40);
    private final int constant;

    private BitModes(int constant) {
        this.constant = constant;
    }

    int constant() {
        return this.constant;
    }
}
