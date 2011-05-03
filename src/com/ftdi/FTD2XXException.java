/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.ftdi;

import java.io.IOException;

/**
 *
 * @author kocsis
 */
public class FTD2XXException extends IOException {

    public FTD2XXException(int ftStatus) {
        super("D2XX error, ftStatus:" + ftStatus);
    }

    public FTD2XXException(Throwable cause) {
        super(cause);
    }

    public FTD2XXException(String message, Throwable cause) {
        super(message, cause);
    }

    public FTD2XXException(String message) {
        super(message);
    }

    public FTD2XXException() {
    }
}
