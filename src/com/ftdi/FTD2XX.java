/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.ftdi;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.ByteByReference;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.LongByReference;
import com.sun.jna.ptr.ShortByReference;

/**
 *
 * @author kocsis
 */
public interface FTD2XX extends Library {

    final FTD2XX INSTANCE = (FTD2XX) Native.synchronizedLibrary(
            (FTD2XX) Native.loadLibrary(
            /*(Platform.isWindows() ? */"ftd2xx" /*: "ftd2xx""ftdi")*/, FTD2XX.class));

    public static class FT_STATUS {

        public final static int FT_OK = 0,
                FT_INVALID_HANDLE = 1,
                FT_DEVICE_NOT_FOUND = 2,
                FT_DEVICE_NOT_OPENED = 3,
                FT_IO_ERROR = 4,
                FT_INSUFFICIENT_RESOURCES = 5,
                FT_INVALID_PARAMETER = 6,
                FT_INVALID_BAUD_RATE = 7,
                FT_DEVICE_NOT_OPENED_FOR_ERASE = 8,
                FT_DEVICE_NOT_OPENED_FOR_WRITE = 9,
                FT_FAILED_TO_WRITE_DEVICE = 10,
                FT_EEPROM_READ_FAILED = 11,
                FT_EEPROM_WRITE_FAILED = 12,
                FT_EEPROM_ERASE_FAILED = 13,
                FT_EEPROM_NOT_PRESENT = 14,
                FT_EEPROM_NOT_PROGRAMMED = 15,
                FT_INVALID_ARGS = 16,
                FT_NOT_SUPPORTED = 17,
                FT_OTHER_ERROR = 18;
    }

    public static class FT_DEVICE {

        public final static int FT_DEVICE_232BM = 0,
                FT_DEVICE_232AM = 1,
                FT_DEVICE_100AX = 2,
                FT_DEVICE_UNKNOWN = 3,
                FT_DEVICE_2232C = 4,
                FT_DEVICE_232R = 5,
                FT_DEVICE_2232H = 6,
                FT_DEVICE_4232H = 7,
                FT_DEVICE_232H = 8;
    }

    public static class Flags {

        public final static int FT_LIST_NUMBER_ONLY = 0x80000,
                FT_LIST_BY_INDEX = 0x40000000,
                FT_LIST_ALL = 0x20000000;
        public final static int FT_OPEN_BY_SERIAL_NUMBER = 1,
                FT_OPEN_BY_DESCRIPTION = 2,
                FT_OPEN_BY_LOCATION = 4;
    }

    public static class WordLenght {

        public final static int FT_BITS_8 = 8,
                FT_BITS_7 = 7;
    }

    public static class StopBits {

        public final static int FT_STOP_BITS_1 = 0,
                FT_STOP_BITS_2 = 2;
    }

    public static class Parity {

        public final static int FT_PARITY_NONE = 0,
                FT_PARITY_ODD = 1,
                FT_PARITY_EVEN = 2,
                FT_PARITY_MARK = 3,
                FT_PARITY_SPACE = 4;
    }

    public static class FlowControl {

        public final static int FT_FLOW_NONE = 0x0000,
                FT_FLOW_RTS_CTS = 0x0100,
                FT_FLOW_DTR_DSR = 0x0200,
                FT_FLOW_XON_XOFF = 0x0400;
    }

    public static class Purge {

        public final static int FT_PURGE_RX = 1,
                FT_PURGE_TX = 2;
    }

    public static class NotificationEvents {

        public final static int FT_EVENT_RXCHAR = 1,
                FT_EVENT_MODEM_STATUS = 2,
                FT_EVENT_LINE_STATUS = 4;
    }

    public static class ModemStatus {

        public final static int CTS = 0x10,
                DSR = 0x20,
                RI = 0x40,
                DCD = 0x80;
    }

    public static class LineStatus {

        public final static int OE = 0x02,
                PE = 0x04,
                FE = 0x08,
                BI = 0x10;
    }

    public static class BitModes {

        public final static int FT_BITMODE_RESET = 0x00,
                FT_BITMODE_ASYNC_BITBANG = 0x01,
                FT_BITMODE_MPSSE = 0x02,
                FT_BITMODE_SYNC_BITBANG = 0x04,
                FT_BITMODE_MCU_HOST = 0x08,
                FT_BITMODE_FAST_SERIAL = 0x10,
                FT_BITMODE_CBUS_BITBANG = 0x20,
                FT_BITMODE_SYNC_FIFO = 0x40;
    }

    public static class FT_DEVICE_LIST_INFO_NODE extends Structure {

        public int Flags;
        public int Type;
        public int ID;
        public int LocId;
        public byte[] SerialNumber = new byte[16];
        public byte[] Description = new byte[64];
        public int ftHandle;
    }

    public static class FT_FLAGS {

        public final static int FT_FLAGS_OPENED = 0x00000001;
    }

    public static class FT_PROGRAM_DATA extends Structure {

        public static class ByReference extends FT_PROGRAM_DATA
                implements Structure.ByReference {
        }
        /**
         * Header - must be 0x0000000
         */
        public int Signature1;
        /**
         * Header - must be 0xffffffff
         */
        public int Signature2;
        /**
         * // Header - FT_PROGRAM_DATA version
         * 0 = original (FT232B)
         * 1 = FT2232 extensions
         * 2 = FT232R extensions
         * 3 = FT2232H extensions
         * 4 = FT4232H extensions
         * 5 = FT232H extensions
         */
        public int Version;
        /**
         * 0x0403
         */
        public short VendorId;
        /**
         * 0x6001
         */
        public short ProductId;
        /**
         * "FTDI"
         */
        public String Manufacturer;
        /**
         * "FT"
         */
        public String ManufacturerId;
        /**
         * "USB HS Serial Converter"
         */
        public String Description;
        /**
         * "FT000001" if fixed, or NULL
         */
        public String SerialNumber;
        /**
         * 0 < MaxPower <= 500
         */
        public short MaxPower;
        /**
         * 0 = disabled, 1 = enabled
         */
        public short PnP;
        /**
         * 0 = bus powered, 1 = self powered
         */
        public short SelfPowered;
        /**
         * 0 = not capable, 1 = capable
         */
        public short RemoteWakeup;
//
// Rev4 (FT232B) extensions
//
        /**
         * non-zero if Rev4 chip, zero otherwise
         */
        public byte Rev4;
        /**
         * non-zero if in endpoint is isochronous
         */
        public byte IsoIn;
        /**
         * non-zero if out endpoint is isochronous
         */
        public byte IsoOut;
        /**
         * non-zero if pull down enabled
         */
        public byte PullDownEnable;
        /**
         * non-zero if serial number to be used
         */
        public byte SerNumEnable;
        /**
         * non-zero if chip uses USBVersion
         */
        public byte USBVersionEnable;
        /**
         * BCD (0x0200 => USB2)
         */
        public short USBVersion;
//
// Rev 5 (FT2232) extensions
//
        /**
         * non-zero if Rev5 chip, zero otherwise
         */
        public byte Rev5;
        /**
         * non-zero if in endpoint is isochronous
         */
        public byte IsoInA;
        /**
         * non-zero if in endpoint is isochronous
         */
        public byte IsoInB;
        /**
         * non-zero if out endpoint is isochronous
         */
        public byte IsoOutA;
        /**
         * non-zero if out endpoint is isochronous
         */
        public byte IsoOutB;
        /**
         * non-zero if pull down enabled
         */
        public byte PullDownEnable5;
        /**
         * non-zero if serial number to be used
         */
        public byte SerNumEnable5;
        /**
         * non-zero if chip uses USBVersion
         */
        public byte USBVersionEnable5;
        /**
         * BCD (0x0200 => USB2)
         */
        public short USBVersion5;
        /**
         * non-zero if interface is high current
         */
        public byte AIsHighCurrent;
        /**
         * non-zero if interface is high current
         */
        public byte BIsHighCurrent;
        /**
         * non-zero if interface is 245 FIFO
         */
        public byte IFAIsFifo;
        /**
         * non-zero if interface is 245 FIFO CPU target
         */
        public byte IFAIsFifoTar;
        /**
         * non-zero if interface is Fast serial
         */
        public byte IFAIsFastSer;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte AIsVCP;
        /**
         * non-zero if interface is 245 FIFO
         */
        public byte IFBIsFifo;
        /**
         * non-zero if interface is 245 FIFO CPU target
         */
        public byte IFBIsFifoTar;
        /**
         * non-zero if interface is Fast serial
         */
        public byte IFBIsFastSer;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte BIsVCP;
//
// Rev 6 (FT232R) extensions
//
        /**
         * Use External Oscillator
         */
        public byte UseExtOsc;
        /**
         * High Drive I/Os
         */
        public byte HighDriveIOs;
        /**
         * Endpoint size
         */
        public byte EndpointSize;
        /**
         * non-zero if pull down enabled
         */
        public byte PullDownEnableR;
        /**
         * non-zero if serial number to be used
         */
        public byte SerNumEnableR;
        /**
         * non-zero if invert TXD
         */
        public byte InvertTXD;
        /**
         * non-zero if invert RXD
         */
        public byte InvertRXD;
        /**
         * non-zero if invert RTS
         */
        public byte InvertRTS;
        /**
         * non-zero if invert CTS
         */
        public byte InvertCTS;
        /**
         * non-zero if invert DTR
         */
        public byte InvertDTR;
        /**
         * non-zero if invert DSR
         */
        public byte InvertDSR;
        /**
         * non-zero if invert DCD
         */
        public byte InvertDCD;
        /**
         * non-zero if invert RI
         */
        public byte InvertRI;
        /**
         * Cbus Mux control
         */
        public byte Cbus0;
        /**
         * Cbus Mux control
         */
        public byte Cbus1;
        /**
         * Cbus Mux control
         */
        public byte Cbus2;
        /**
         * Cbus Mux control
         */
        public byte Cbus3;
        /**
         * Cbus Mux control
         */
        public byte Cbus4;
        /**
         * non-zero if using D2XX driver
         */
        public byte RIsD2XX;
        //
// Rev 7 (FT2232H) Extensions
//
        /**
         * non-zero if pull down enabled
         */
        public byte PullDownEnable7;
        /**
         * non-zero if serial number to be used
         */
        public byte SerNumEnable7;
        /**
         * non-zero if AL pins have slow slew
         */
        public byte ALSlowSlew;
        /**
         * non-zero if AL pins are Schmitt input
         */
        public byte ALSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte ALDriveCurrent;
        /**
         * non-zero if AH pins have slow slew
         */
        public byte AHSlowSlew;
        /**
         * non-zero if AH pins are Schmitt input
         */
        public byte AHSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte AHDriveCurrent;
        /**
         * non-zero if BL pins have slow slew
         */
        public byte BLSlowSlew;
        /**
         * non-zero if BL pins are Schmitt input
         */
        public byte BLSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte BLDriveCurrent;
        /**
         * non-zero if BH pins have slow slew
         */
        public byte BHSlowSlew;
        /**
         * non-zero if BH pins are Schmitt input
         */
        public byte BHSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte BHDriveCurrent;
        /**
         * non-zero if interface is 245 FIFO
         */
        public byte IFAIsFifo7;
        /**
         * non-zero if interface is 245 FIFO CPU target
         */
        public byte IFAIsFifoTar7;
        /**
         * non-zero if interface is Fast serial
         */
        public byte IFAIsFastSer7;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte AIsVCP7;
        /**
         * non-zero if interface is 245 FIFO
         */
        public byte IFBIsFifo7;
        /**
         * non-zero if interface is 245 FIFO CPU target
         */
        public byte IFBIsFifoTar7;
        /**
         * non-zero if interface is Fast serial
         */
        public byte IFBIsFastSer7;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte BIsVCP7;
        /**
         * non-zero if using BCBUS7 to save power for self-powered designs
         */
        public byte PowerSaveEnable;
//
// Rev 8 (FT4232H) Extensions
//
        /**
         * non-zero if pull down enabled
         */
        public byte PullDownEnable8;
        /**
         * non-zero if serial number to be used
         */
        public byte SerNumEnable8;
        /**
         * non-zero if AL pins have slow slew
         */
        public byte ASlowSlew;
        /**
         * non-zero if AL pins are Schmitt input
         */
        public byte ASchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte ADriveCurrent;
        /**
         * non-zero if AH pins have slow slew
         */
        public byte BSlowSlew;
        /**
         * non-zero if AH pins are Schmitt input
         */
        public byte BSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte BDriveCurrent;
        /**
         * non-zero if BL pins have slow slew
         */
        public byte CSlowSlew;
        /**
         * non-zero if BL pins are Schmitt input
         */
        public byte CSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte CDriveCurrent;
        /**
         * non-zero if BH pins have slow slew
         */
        public byte DSlowSlew;
        /**
         * non-zero if BH pins are Schmitt input
         */
        public byte DSchmittInput;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte DDriveCurrent;
        /**
         * non-zero if port A uses RI as RS485 TXDEN
         */
        public byte ARIIsTXDEN;
        /**
         * non-zero if port B uses RI as RS485 TXDEN
         */
        public byte BRIIsTXDEN;
        /**
         * non-zero if port C uses RI as RS485 TXDEN
         */
        public byte CRIIsTXDEN;
        /**
         * non-zero if port D uses RI as RS485 TXDEN
         */
        public byte DRIIsTXDEN;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte AIsVCP8;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte BIsVCP8;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte CIsVCP8;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte DIsVCP8;
//
// Rev 9 (FT232H) Extensions
//
        /**
         * non-zero if pull down enabled
         */
        public byte PullDownEnableH;
        /**
         * non-zero if serial number to be used
         */
        public byte SerNumEnableH;
        /**
         * non-zero if AC pins have slow slew
         */
        public byte ACSlowSlewH;
        /**
         * non-zero if AC pins are Schmitt input
         */
        public byte ACSchmittInputH;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte ACDriveCurrentH;
        /**
         * non-zero if AD pins have slow slew
         */
        public byte ADSlowSlewH;
        /**
         * non-zero if AD pins are Schmitt input
         */
        public byte ADSchmittInputH;
        /**
         * valid values are 4mA, 8mA, 12mA, 16mA
         */
        public byte ADDriveCurrentH;
        /**
         * Cbus Mux control
         */
        public byte Cbus0H;
        /**
         * Cbus Mux control
         */
        public byte Cbus1H;
        /**
         * Cbus Mux control
         */
        public byte Cbus2H;
        /**
         * Cbus Mux control
         */
        public byte Cbus3H;
        /**
         * Cbus Mux control
         */
        public byte Cbus4H;
        /**
         * Cbus Mux control
         */
        public byte Cbus5H;
        /**
         * Cbus Mux control
         */
        public byte Cbus6H;
        /**
         * Cbus Mux control
         */
        public byte Cbus7H;
        /**
         * Cbus Mux control
         */
        public byte Cbus8H;
        /**
         * Cbus Mux control
         */
        public byte Cbus9H;
        /**
         * non-zero if interface is 245 FIFO
         */
        public byte IsFifoH;
        /**
         * non-zero if interface is 245 FIFO CPU target
         */
        public byte IsFifoTarH;
        /**
         * non-zero if interface is Fast serial
         */
        public byte IsFastSerH;
        /**
         * non-zero if interface is FT1248
         */
        public byte IsFT1248H;
        /**
         * FT1248 clock polarity - clock idle high (1) or clock idle low (0)
         */
        public byte FT1248CpolH;
        /**
         * FT1248 data is LSB (1) or MSB (0)
         */
        public byte FT1248LsbH;
        /**
         * FT1248 flow control enable
         */
        public byte FT1248FlowControlH;
        /**
         * non-zero if interface is to use VCP drivers
         */
        public byte IsVCPH;
        /**
         * non-zero if using ACBUS7 to save power for self-powered designs
         */
        public byte PowerSaveEnableH;
    }

    /**
     * A command to include a custom VID and PID combination within the internal
     * device list table.  This will allow the driver to load for the specified 
     * VID and PID combination.
     * @param dwVID Device Vendor ID (VID)
     * @param dwPID Device Product ID (PID)
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetVIDPID(int dwVID, int dwPID);

    /**
     * A command to retrieve the current VID and PID combination from within the
     * internal device list table.
     * @param pdwVID Pointer to DWORD(int) that will contain the internal VID
     * @param pdwPID Pointer to DWORD(int) that will contain the internal PID
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetVIDPID(IntByReference pdwVID, IntByReference pdwPID);

    /**
     * This function builds a device information list and returns the number of 
     * D2XX devices connected to the system.  The list contains information 
     * about both unopen and open devices.
     * @param lpdwNumDevs Pointer to unsigned long(long) to store the number of 
     * devices connected.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_CreateDeviceInfoList(LongByReference lpdwNumDevs);

    /**
     * This function returns an entry from the device information list. 
     * @param pDest Pointer to an array of FT_DEVICE_LIST_INFO_NODE structures.
     * @param lpdwNumDevs Pointer to the number of elements in the array
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetDeviceInfoList(FT_DEVICE_LIST_INFO_NODE[] pDest,
            LongByReference lpdwNumDevs);

    /**
     * This function returns an entry from the device information list.
     * @param dwIndex Index of the entry in the device info list.
     * @param lpdwFlags Pointer to unsigned long to store the flag value.
     * @param lpdwType Pointer to unsigned long to store device type.
     * @param lpdwID Pointer to unsigned long to store device ID.
     * @param lpdwLocId Pointer to unsigned long to store the device location ID.
     * @param pcSerialNumber Pointer to buffer to store device serial number as 
     * a nullterminated string.
     * @param pcDescription Pointer to buffer to store device description as a 
     * null-terminated string.
     * @param ftHandle  Pointer to a variable of type FT_HANDLE where the handle
     * will be stored.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetDeviceInfoDetail(int dwIndex, LongByReference lpdwFlags,
            LongByReference lpdwType, LongByReference lpdwID,
            LongByReference lpdwLocId, String pcSerialNumber,
            String pcDescription, IntByReference ftHandle);

    /**
     * Gets information concerning the devices currently connected.  This 
     * function can return information such as the number of devices connected, 
     * the device serial number and device description strings, and the 
     * location IDs of connected devices.
     * @param pvArg1 Meaning depends on dwFlags
     * @param pvArg2 Meaning depends on dwFlags
     * @param dwFlags Determines format of returned information.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_ListDevices(Pointer pvArg1, Pointer pvArg2, int dwFlags);

    /**
     * Open the device and return a handle which will be used for subsequent 
     * accesses.
     * @param iDevice Index of the device to open.  Indices are 0 based.
     * @param ftHandle Pointer to a variable of type FT_HANDLE where the handle
     * will be stored. This handle must be used to access the device.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Open(int iDevice, IntByReference ftHandle);

    /**
     * Open the specified device and return a handle that will be used for
     * subsequent accesses.  The device can be specified by its serial number,
     * device description or location. This function can also be used to open
     * multiple devices simultaneously.  Multiple devices can be specified by 
     * serial number, device description or location ID (location information 
     * derived from the physical location of a device on USB).  Location IDs for
     * specific USB ports can be obtained using the utility USBView and are 
     * given in hexadecimal format.  Location IDs for devices connected to a 
     * system can be obtained by calling FT_GetDeviceInfoList or FT_ListDevices 
     * with the appropriate flags. 
     * @param pvArg1 Pointer to an argument whose type depends on the value of 
     * dwFlags.  It is normally be interpreted as a pointer to a null 
     * terminated string. 
     * @param dwFlags FT_OPEN_BY_SERIAL_NUMBER, FT_OPEN_BY_DESCRIPTION or 
     * FT_OPEN_BY_LOCATION.
     * @param ftHandle Pointer to a variable of type FT_HANDLE where the handle 
     * will be stored.  This handle must be used to access the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_OpenEx(Pointer pvArg1, int dwFlags, IntByReference ftHandle);

    /**
     * Close an open device.
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Close(int ftHandle);

    /**
     * Read data from the device. 
     * @param ftHandle Handle of the device.
     * @param lpBuffer Pointer to the buffer that receives the data from the 
     * device. 
     * @param dwBytesToRead Number of bytes to be read from the device. 
     * @param lpdwBytesReturned Pointer to a variable of type DWORD which 
     * receives the number of bytes read from the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Read(int ftHandle, byte[] lpBuffer, int dwBytesToRead,
            IntByReference lpdwBytesReturned);

    /**
     * Write data to the device. 
     * @param ftHandle Handle of the device.
     * @param lpBuffer Pointer to the buffer that contains the data to be 
     * written to the device. 
     * @param dwBytesToWrite Number of bytes to write to the device. 
     * @param lpdwBytesWritten Pointer to a variable of type DWORD which 
     * receives the number of bytes written to the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Write(int ftHandle, byte[] lpBuffer, int dwBytesToWrite,
            IntByReference lpdwBytesWritten);

    /**
     * This function sets the baud rate for the device. 
     * @param ftHandle Handle of the device.
     * @param dwBaudRate Baud rate. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetBaudRate(int ftHandle, int dwBaudRate);

    /**
     * This function sets the baud rate for the device.  It is used to set 
     * non-standard baud rates. 
     * This function is no longer required as FT_SetBaudRate will now 
     * automatically calculate the required divisor for a requested baud rate.
     * @param ftHandle Handle of the device. 
     * @param usDivisor Divisor. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetDivisor(int ftHandle, short usDivisor);

    /**
     * This function sets the data characteristics for the device. 
     * @param ftHandle Handle of the device. 
     * @param uWordLength Number of bits per word - must be FT_BITS_8 or 
     * FT_BITS_7. 
     * @param uStopBits Number of stop bits - must be FT_STOP_BITS_1 or
     * FT_STOP_BITS_2. 
     * @param uParity Parity - must be FT_PARITY_NONE, FT_PARITY_ODD, 
     * FT_PARITY_EVEN, FT_PARITY_MARK or FT_PARITY SPACE. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetDataCharacteristics(int ftHandle, byte uWordLength,
            byte uStopBits, byte uParity);

    /**
     * This function sets the read and write timeouts for the device. 
     * @param ftHandle Handle of the device.
     * @param dwReadTimeout Read timeout in milliseconds.
     * @param dwWriteTimeout Write timeout in milliseconds.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetTimeouts(int ftHandle, int dwReadTimeout, int dwWriteTimeout);

    /**
     * This function sets the flow control for the device. 
     * @param ftHandle Handle of the device.
     * @param usFlowControl Must be one of FT_FLOW_NONE, FT_FLOW_RTS_CTS,
     * FT_FLOW_DTR_DSR or FT_FLOW_XON_XOFF.
     * @param uXon Character used to signal Xon.  Only used if flow control is
     * FT_FLOW_XON_XOFF. 
     * @param uXoff Character used to signal Xoff.  Only used if flow control is
     * FT_FLOW_XON_XOFF. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetFlowControl(int ftHandle, short usFlowControl, byte uXon,
            byte uXoff);

    /**
     * This function sets the Data Terminal Ready (DTR) control signal. 
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetDtr(int ftHandle);

    /**
     * This function clears the Data Terminal Ready (DTR) control signal. 
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_ClrDtr(int ftHandle);

    /**
     * This function sets the Request To Send (RTS) control signal. 
     * @param ftHandle Handle of the device.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetRts(int ftHandle);

    /**
     * This function clears the Request To Send (RTS) control signal. 
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_ClrRts(int ftHandle);

    /**
     * Gets the modem status and line status from the device. 
     * @param ftHandle Handle of the device.
     * @param lpdwModemStatus Pointer to a variable of type DWORD which receives
     * the modem status and line status from the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetModemStatus(int ftHandle, IntByReference lpdwModemStatus);

    /**
     * Gets the number of bytes in the receive queue. 
     * @param ftHandle Handle of the device. 
     * @param lpdwAmountInRxQueue Pointer to a variable of type DWORD which 
     * receives the number of bytes in the receive queue. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetQueueStatus(int ftHandle, IntByReference lpdwAmountInRxQueue);

    /**
     * Get device information for an open device. 
     * @param ftHandle Handle of the device. 
     * @param pftType Pointer to unsigned long to store device type. 
     * @param lpdwID Pointer to unsigned long to store device ID. 
     * @param pcSerialNumber Pointer to buffer to store device serial number as
     * a null-terminated string. 
     * @param pcDescription Pointer to buffer to store device description as a 
     * null-terminated string.
     * @param pvDummy Reserved for future use - should be set to NULL. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetDeviceInfo(int ftHandle, IntByReference pftType,
            IntByReference lpdwID, String pcSerialNumber, String pcDescription,
            Pointer pvDummy);

    /**
     * This function returns the D2XX driver version number. 
     * @param ftHandle Handle of the device. 
     * @param lpdwDriverVersion Pointer to the driver version number. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetDriverVersion(int ftHandle, IntByReference lpdwDriverVersion);

    /**
     * This function returns D2XX DLL version number. 
     * @param lpdwDLLVersion Pointer to the DLL version number. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetLibraryVersion(IntByReference lpdwDLLVersion);

    /**
     * Retrieves the COM port associated with a device. 
     * @param ftHandle Handle of the device. 
     * @param lplComPortNumber Pointer to a variable of type LONG which receives
     * the COM port number associated with the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetComPortNumber(int ftHandle, IntByReference lplComPortNumber);

    /**
     * Gets the device status including number of characters in the receive
     * queue, number of characters in the transmit queue, and the current event
     * status. 
     * @param ftHandle Handle of the device. 
     * @param lpdwAmountInRxQueue Pointer to a variable of type DWORD which 
     * receives the number of characters in the receive queue. 
     * @param lpdwAmountInTxQueue Pointer to a variable of type DWORD which
     * receives the number of characters in the transmit queue. 
     * @param lpdwEventStatus Pointer to a variable of type DWORD which receives
     * the current state of the event status. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetStatus(int ftHandle, IntByReference lpdwAmountInRxQueue,
            IntByReference lpdwAmountInTxQueue, IntByReference lpdwEventStatus);

    /**
     * Sets conditions for event notification. 
     * @param ftHandle Handle of the device.
     * @param dwEventMask Conditions that cause the event to be set.
     * @param pvArg Interpreted as the handle of an event.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetEventNotification(int ftHandle, int dwEventMask, Pointer pvArg);

    /**
     * This function sets the special characters for the device. 
     * @param ftHandle Handle of the device.
     * @param uEventCh Event character.
     * @param uEventChEn 0 if event character disabled, non-zero otherwise.
     * @param uErrorCh Error character.
     * @param uErrorChEn 0 if error character disabled, non-zero otherwise.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetChars(int ftHandle, byte uEventCh, byte uEventChEn,
            byte uErrorCh, byte uErrorChEn);

    /**
     * Sets the BREAK condition for the device. 
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetBreakOn(int ftHandle);

    /**
     * Resets the BREAK condition for the device.
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetBreakOff(int ftHandle);

    /**
     * This function purges receive and transmit buffers in the device. 
     * @param ftHandle Handle of the device.
     * @param dwMask Combination of FT_PURGE_RX and FT_PURGE_TX.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Purge(int ftHandle, int dwMask);

    /**
     * This function sends a reset command to the device. 
     * @param ftHandle Handle of the device.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_ResetDevice(int ftHandle);

    /**
     * Send a reset command to the port. 
     * @param ftHandle Handle of the device.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_ResetPort(int ftHandle);

    /**
     * Send a cycle command to the USB port. 
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_CyclePort(int ftHandle);

    /**
     * This function can be of use when trying to recover devices
     * programatically. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Rescan();

    /**
     * This function forces a reload of the driver for devices with a specific 
     * VID and PID combination. 
     * @param wVID Vendor ID of the devices to reload the driver for. 
     * @param wPID Product ID of the devices to reload the driver for. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_Reload(short wVID, short wPID);

    /**
     * Set the ResetPipeRetryCount value. 
     * @param ftHandle Handle of the device. 
     * @param dwCount Unsigned long containing required ResetPipeRetryCount. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetResetPipeRetryCount(int ftHandle, int dwCount);

    /**
     * Stops the driver's IN task.
     * @param ftHandle Handle of the device.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_StopInTask(int ftHandle);

    /**
     * Restart the driver's IN task. 
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_RestartInTask(int ftHandle);

    /**
     * This function allows the maximum time in milliseconds that a USB request
     * can remain outstanding to be set. 
     * @param ftHandle Handle of the device. 
     * @param dwDeadmanTimeout Deadman timeout value in milliseconds. 
     * Default value is 5000. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetDeadmanTimeout(int ftHandle, int dwDeadmanTimeout);

    /**
     * Read a value from an EEPROM location. 
     * @param ftHandle Handle of the device. 
     * @param dwWordOffset EEPROM location to read from. 
     * @param lpwValue Pointer to the WORD value read from the EEPROM. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_ReadEE(int ftHandle, int dwWordOffset, ShortByReference lpwValue);

    /**
     * Write a value to an EEPROM location. 
     * @param ftHandle Handle of the device.
     * @param dwWordOffset EEPROM location to read from.
     * @param wValue The WORD value write to the EEPROM. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_WriteEE(int ftHandle, int dwWordOffset, short wValue);

    /**
     * Erases the device EEPROM.
     * @param ftHandle Handle of the device. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EraseEE(int ftHandle);

    /**
     * Read the contents of the EEPROM. 
     * @param ftHandle Handle of the device.
     * @param pData Pointer to structure of type FT_PROGRAM_DATA.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_Read(int ftHandle, FT_PROGRAM_DATA.ByReference pData);

    /**
     * Read the contents of the EEPROM and pass strings separately. 
     * @param ftHandle Handle of the device. 
     * @param pData Pointer to structure of type FT_PROGRAM_DATA.
     * @param Manufacturer Pointer to a null-terminated string containing the
     * manufacturer name. 
     * @param ManufacturerId Pointer to a null-terminated string containing the
     * manufacturer ID. 
     * @param Description Pointer to a null-terminated string containing the
     * device description. 
     * @param SerialNumber Pointer to a null-terminated string containing the
     * device serial number.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_ReadEx(int ftHandle, FT_PROGRAM_DATA.ByReference pData,
            String Manufacturer, String ManufacturerId, String Description,
            String SerialNumber);

    /**
     * Program the EEPROM.
     * @param ftHandle Handle of the device. 
     * @param pData Pointer to structure of type FT_PROGRAM_DATA. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_Program(int ftHandle, FT_PROGRAM_DATA.ByReference pData);

    /**
     * Program the EEPROM and pass strings separately. 
     * @param ftHandle Handle of the device.
     * @param pData Pointer to structure of type FT_PROGRAM_DATA. 
     * @param Manufacturer Pointer to a null-terminated string containing the
     * manufacturer name. 
     * @param ManufacturerId Pointer to a null-terminated string containing the
     * manufacturer ID. 
     * @param Description Pointer to a null-terminated string containing the
     * device description. 
     * @param SerialNumber Pointer to a null-terminated string containing the
     * device serial number.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_ProgramEx(int ftHandle, FT_PROGRAM_DATA.ByReference pData,
            String Manufacturer, String ManufacturerId,
            String Description, String SerialNumber);

    /**
     * Get the available size of the EEPROM user area.
     * @param ftHandle Handle of the device.
     * @param lpdwSize Pointer to a DWORD that receives the available size, in 
     * bytes, of the EEPROM user area. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_UASizeWrite(int ftHandle, IntByReference lpdwSize);

    /**
     * Read the contents of the EEPROM user area. 
     * @param ftHandle Handle of the device.
     * @param pucData Pointer to a buffer that contains storage for data to be
     * read. 
     * @param dwDataLen Size, in bytes, of buffer that contains storage for the
     * data to be read. 
     * @param lpdwBytesRead Pointer to a DWORD that receives the number of bytes
     * read. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_UARead(int ftHandle, byte[] pucData, int dwDataLen,
            IntByReference lpdwBytesRead);

    /**
     * Write data into the EEPROM user area. 
     * @param ftHandle Handle of the device.
     * @param pucData Pointer to a buffer that contains the data to be written.
     * @param dwDataLen Size, in bytes, of buffer that contains storage for the
     * data to be read. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_EE_UAWrite(int ftHandle, byte[] pucData, int dwDataLen);

    /**
     * Set the latency timer value.  
     * @param ftHandle Handle of the device. 
     * @param ucTimer Required value, in milliseconds, of latency timer. Valid
     * range is 2 – 255. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetLatencyTimer(int ftHandle, byte ucTimer);

    /**
     * Get the current value of the latency timer. 
     * @param ftHandle Handle of the device. 
     * @param pucTimer Pointer to unsigned char to store latency timer value.
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetLatencyTimer(int ftHandle, byte pucTimer);

    /**
     * Enables different chip modes. 
     * @param ftHandle Handle of the device. 
     * @param ucMask Required value for bit mode mask.  This sets up which bits
     * are  inputs and outputs.  A bit value of 0 sets the corresponding pin to
     * an input, a bit value of 1 sets the corresponding pin to an output. 
     * In the case of CBUS Bit Bang, the upper nibble of this value controls 
     * which pins are inputs and outputs, while the lower nibble controls which
     * of the outputs are high and low. 
     * @param ucMode Mode value.  Can be one of the following: 
     * 0x0 = Reset 
     * 0x1 = Asynchronous Bit Bang 
     * 0x2 = MPSSE (FT2232, FT2232H, FT4232H and FT232H devices only) 
     * 0x4 = Synchronous Bit Bang (FT232R, FT245R, FT2232, FT2232H, FT4232H and 
     * FT232H devices only) 
     * 0x8 = MCU Host Bus Emulation Mode (FT2232, FT2232H, FT4232H and FT232H 
     * devices only) 
     * 0x10 = Fast Opto-Isolated Serial Mode (FT2232, FT2232H, FT4232H and 
     * FT232H devices only) 
     * 0x20 = CBUS Bit Bang Mode (FT232R and FT232H devices only)  
     * 0x40 = Single Channel Synchronous 245 FIFO Mode (FT2232H and FT232H 
     * devices only) 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetBitmode(int ftHandle, byte ucMask, byte ucMode);

    /**
     * Gets the instantaneous value of the data bus. 
     * @param ftHandle Handle of the device. 
     * @param pucMode Pointer to unsigned char to store the instantaneous data
     * bus value. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_GetBitmode(int ftHandle, ByteByReference pucMode);

    /**
     * Set the USB request transfer size. 
     * @param ftHandle Handle of the device. 
     * @param dwInTransferSize Transfer size for USB IN request. 
     * @param dwOutTransferSize Transfer size for USB OUT request. 
     * @return FT_STATUS: FT_OK if successful, otherwise the return value is an 
     * FT error code.
     */
    int FT_SetUSBParameters(int ftHandle, int dwInTransferSize,
            int dwOutTransferSize);
}
