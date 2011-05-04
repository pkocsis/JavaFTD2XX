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

import com.sun.jna.Memory;
import com.sun.jna.ptr.IntByReference;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Java class to communicate easily to a FTDI device.
 * @author Peter Kocsis <p. kocsis. 2. 7182 at gmail.com>
 */
public class FTDevice {

    static private final FTD2XX ftd2xx = FTD2XX.INSTANCE;
    private final int devID, devLocationID;
    private final DeviceType devType;
    private int ftHandle;
    private final String devSerialNumber, devDescription;

    private FTDevice(DeviceType devType, int devID, int devLocationID,
            String devSerialNumber, String devDescription, int ftHandle) {
        this.devType = devType;
        this.devID = devID;
        this.devLocationID = devLocationID;
        this.devSerialNumber = devSerialNumber;
        this.devDescription = devDescription;
        this.ftHandle = ftHandle;
    }

    /**
     * Get device description.
     * @return device description
     */
    public String getDevDescription() {
        return devDescription;
    }

    /**
     * Get device ID.
     * @return device ID
     */
    public int getDevID() {
        return devID;
    }

    /**
     * Get device serial number.
     * @return device serial number
     */
    public String getDevSerialNumber() {
        return devSerialNumber;
    }

    /**
     * Get device type.
     * @return device type.
     */
    public DeviceType getDevType() {
        return devType;
    }

    /**
     * Get device location.
     * @return device location.
     */
    public int getDevLocationID() {
        return devLocationID;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof FTDevice)) {
            return false;
        }
        FTDevice eq = (FTDevice) obj;
        return eq.ftHandle == this.ftHandle;
    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 97 * hash + this.ftHandle;
        return hash;
    }

    @Override
    public String toString() {
        return "FTDevice{" + "devDescription=" + devDescription
                + ", devSerialNumber=" + devSerialNumber + '}';
    }

    private static void ensureFTStatus(int ftstatus) throws FTD2XXException {
        if (!(ftstatus == FT_STATUS.FT_OK.constant())) {
            throw new FTD2XXException(ftstatus);
        }
    }

    /**
     * Get the connected FTDI devices.
     * @return List contain available FTDI devices.
     * @throws FTD2XXException If something goes wrong.
     */
    public static List<FTDevice> getDevices() throws FTD2XXException {
        IntByReference devNum = new IntByReference();

        ensureFTStatus(ftd2xx.FT_CreateDeviceInfoList(devNum));

        Logger.getLogger(FTDevice.class.getName()).log(Level.INFO,
                "Found devs:{0}", devNum.getValue());

        ArrayList<FTDevice> devs = new ArrayList<FTDevice>(devNum.getValue());

        IntByReference flag = new IntByReference();
        IntByReference devType = new IntByReference();
        IntByReference devID = new IntByReference();
        IntByReference locID = new IntByReference();
        IntByReference ftHandle = new IntByReference();
        Memory devSerNum = new Memory(16);
        Memory devDesc = new Memory(64);
        for (int i = 0; i < devNum.getValue(); i++) {
            ensureFTStatus(ftd2xx.FT_GetDeviceInfoDetail(i, flag, devType, devID,
                    locID, devSerNum, devDesc, ftHandle));

            devs.add(new FTDevice(DeviceType.values()[devType.getValue()],
                    devID.getValue(), locID.getValue(), devSerNum.getString(0),
                    devDesc.getString(0), ftHandle.getValue()));

        }
        return devs;
    }

    /**
     * Open connection with device.
     * @throws FTD2XXException If something goes wrong.
     */
    public void open() throws FTD2XXException {
        Memory memory = new Memory(16);
        memory.setString(0, devSerialNumber);
        IntByReference handle = new IntByReference();
        ensureFTStatus(ftd2xx.FT_OpenEx(memory, FTD2XX.FT_OPEN_BY_SERIAL_NUMBER,
                handle));
        this.ftHandle = handle.getValue();
    }

    /**
     * Close connection with device.
     * @throws FTD2XXException If something goes wrong.
     */
    public void close() throws FTD2XXException {
        ensureFTStatus(ftd2xx.FT_Close(ftHandle));
    }

    /**
     * Set desired baud rate.
     * @param baudRate The baud rate.
     * @throws FTD2XXException If something goes wrong.
     */
    public void setBaudRate(long baudRate) throws FTD2XXException {
        ensureFTStatus(ftd2xx.FT_SetBaudRate(ftHandle, (int) baudRate));
    }

    /**
     * This function sets the data characteristics for the device
     * @param wordLength Number of bits per word 
     * @param stopBits Number of stop bits
     * @param parity Parity
     * @throws FTD2XXException If something goes wrong.
     */
    public void setDataCharacteristics(WordLength wordLength, StopBits stopBits,
            Parity parity) throws FTD2XXException {
        ensureFTStatus(ftd2xx.FT_SetDataCharacteristics(ftHandle,
                (byte) wordLength.constant(), (byte) stopBits.constant(),
                (byte) parity.constant()));
    }

    /**
     * Set the read and write timeouts for the device.
     * @param readTimeout Read timeout in milliseconds.
     * @param writeTimeout Write timeout in milliseconds.
     * @throws FTD2XXException If something goes wrong.
     */
    public void setTimeouts(long readTimeout, long writeTimeout)
            throws FTD2XXException {
        ensureFTStatus(ftd2xx.FT_SetTimeouts(ftHandle, (int) readTimeout,
                (int) writeTimeout));
    }
    
    /**
     * Sets the flow control for the device.
     * @param flowControl Flow control type.
     * @throws FTD2XXException If something goes wrong.
     */
    public void setFlowControl(FlowControl flowControl) throws FTD2XXException{
        ensureFTStatus(ftd2xx.FT_SetFlowControl(ftHandle, 
                (short)flowControl.constant(), (byte)0, (byte)0));
    }
    
    /**
     * Sets the flow control for the device.
     * @param flowControl Flow control type.
     * @param uXon Character used to signal Xon. Only used if flow control is 
     * FT_FLOW_XON_XOFF
     * @param uXoff Character used to signal Xoff.  Only used if flow control is 
     * FT_FLOW_XON_XOFF
     * @throws FTD2XXException If something goes wrong.
     */
    public void setFlowControl(FlowControl flowControl, byte uXon, byte uXoff) 
            throws FTD2XXException{
        ensureFTStatus(ftd2xx.FT_SetFlowControl(ftHandle, 
                (short)flowControl.constant(), uXon, uXoff));
    }

    /**
     * Write bytes to device.
     * @param bytes Byte array to send
     * @param offset Start index
     * @param length Amount of bytes to write
     * @return Number of bytes actually written
     * @throws FTD2XXException If something goes wrong.
     */
    public int write(byte[] bytes, int offset, int length)
            throws FTD2XXException {
        Memory memory = new Memory(0);
        memory.write(0, bytes, offset, length);
        IntByReference wrote = new IntByReference();

        ensureFTStatus(ftd2xx.FT_Write(ftHandle, memory, length, wrote));

        return wrote.getValue();
    }

    /**
     * Write bytes to device.
     * @param bytes Byte array to send
     * @return Number of bytes actually written
     * @throws FTD2XXException If something goes wrong.
     */
    public int write(byte[] bytes) throws FTD2XXException {
        return write(bytes, 0, bytes.length);
    }

    /**
     * Write byte to device.
     * @param b Byte to send (0..255)
     * @return It was success?
     */
    public boolean write(int b) throws FTD2XXException {
        byte[] c = new byte[1];
        c[0] = (byte) b;
        return (write(c) == 1) ? true : false;
    }

    /**
     * Read bytes from device.
     * @param bytes Bytes array to store read bytes
     * @param offset Start index.
     * @param lenght Amount of bytes to read
     * @return Number of bytes actually read
     * @throws FTD2XXException If something goes wrong.
     */
    public int read(byte[] bytes, int offset, int lenght)
            throws FTD2XXException {
        Memory memory = new Memory(lenght);
        IntByReference read = new IntByReference();

        ensureFTStatus(ftd2xx.FT_Read(ftHandle, memory, lenght, read));

        memory.read(0, bytes, offset, lenght);

        return read.getValue();
    }

    /**
     * Read bytes from device.
     * @param bytes Bytes array to store read bytes
     * @return Number of bytes actually read
     * @throws FTD2XXException If something goes wrong.
     */
    public int read(byte[] bytes) throws FTD2XXException {
        return read(bytes, 0, bytes.length);
    }

    /**
     * Read a byte from device.
     * @return The byte what read or -1;
     * @throws FTD2XXException 
     */
    public int read() throws FTD2XXException {
        byte[] c = new byte[1];
        int ret = read(c);
        return (ret == 1) ? ((int) c[0] & 0xFF) : -1;
    }
}
