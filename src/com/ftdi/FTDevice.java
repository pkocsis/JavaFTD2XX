/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.ftdi;

import com.sun.jna.ptr.IntByReference;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Péter Kocsis
 */
public class FTDevice {

    static private FTD2XX ftd2xx = FTD2XX.INSTANCE;
    private final int devType, devID, locID, ftHandle;
    private final String devSerNum, devDesc;

    private FTDevice(int devType, int devID, int locID, String devSerNum,
            String devDesc, int ftHandle) {
        this.devType = devType;
        this.devID = devID;
        this.locID = locID;
        this.devSerNum = devSerNum;
        this.devDesc = devDesc;
        this.ftHandle = ftHandle;
    }

    public static List<FTDevice> getDevices() throws FTD2XXException {
        IntByReference devNum = new IntByReference();

        int ftStatus = ftd2xx.FT_CreateDeviceInfoList(devNum);
        if (!(ftStatus == FTD2XX.FT_STATUS.FT_OK)) {
            throw new FTD2XXException(ftStatus);
        }
        Logger.getLogger(FTDevice.class.getName()).log(Level.INFO,
                "Found devs:{0}", devNum.getValue());

        ArrayList<FTDevice> devs = new ArrayList<FTDevice>(devNum.getValue());

        IntByReference flag = new IntByReference();
        IntByReference devType = new IntByReference();
        IntByReference devID = new IntByReference();
        IntByReference locID = new IntByReference();
        IntByReference ftHandle = new IntByReference();
        String devSerNum = new String();
        String devDesc = new String();
        for (int i = 0; i < devNum.getValue(); i++) {
            ftStatus = ftd2xx.FT_GetDeviceInfoDetail(i, flag, devType, devID,
                    locID, devSerNum, devDesc, ftHandle);
            if (!(ftStatus == FTD2XX.FT_STATUS.FT_OK)) {
                throw new FTD2XXException(ftStatus);
            }
            
            devs.add(new FTDevice(devType.getValue(), devID.getValue(),
                    locID.getValue(), devSerNum, devDesc, ftHandle.getValue()));
        }
        return devs;
    }
}
