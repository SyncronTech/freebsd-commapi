/*
 *
 * FreeBSD's JAVAComm API is a native interface to Serial/Parallel ports in Java.
 *
 * Copyright (c) 1999 Jean-Michel DRICOT <jdricot@ulb.ac.be>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS `AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */

package org.freebsd.io.comm;

import java.io.*;
import java.lang.*;
import java.io.IOException;
import java.io.OutputStream;
import java.io.InputStream;
import javax.comm.*;
import java.util.TooManyListenersException;

public class FreebsdSerial extends SerialPort
{
    private int                baud        = 9600;
    private int                databits    = SerialPort.DATABITS_8;
    private int                stopbits    = SerialPort.STOPBITS_1;
    private int                parity      = SerialPort.PARITY_NONE;
    private int                flowcontrol = SerialPort.FLOWCONTROL_NONE;
    private int                sd          = -1;
    private int                ibs         = 1024;
    private int                obs         = 1024;
    private int                framing     = -1;
    private int                timeout     = -1;
    private int                threshold   = -1;
    private SerialInputStream  input       = null;
    private SerialOutputStream output      = null;
    private SerialPortEventListener SPEventListener = null;
    private static boolean     debug       = false;
    private Thread           monitorThread = null;

    private boolean monitorData		   = true;
    private boolean monitorOutput          = false;
    private boolean monitorCTS             = false;
    private boolean monitorDSR             = false;
    private boolean monitorRI              = false;
    private boolean monitorOE              = false;
    private boolean monitorPE              = false;
    private boolean monitorCD              = false;
    private boolean monitorFE              = false;
    private boolean monitorBI              = false;


    /**
    * Loads up the native library and makes proper initializations.
    */
    static 
    {
        String debugFlag = System.getProperty("org.freebsd.io.comm.debug");

        if (debugFlag != null)
        	debug = Boolean.valueOf(debugFlag).booleanValue();

        try
        {
            System.loadLibrary("Serial");
            // Initialize ();
        }
        catch (SecurityException e1)
        {
            System.err.println("FreebsdSerial: SecurityException = " +
                               e1.getMessage());
            e1.printStackTrace ();
        }
        catch (UnsatisfiedLinkError e2)
        {
            System.err.println("FreebsdSerial: UnsatisfiedLinkError = " +
                               e2.getMessage());
            e2.printStackTrace ();
        }
        if (debug == true)
        {
            System.out.println ("loaded library successfully");
        }
    }

    /**
     * class contructor
     * @exception IOException if the port could not be opened.
     */
    public FreebsdSerial(String port) throws IOException
    {
        name = new String (port);

        // try and open the specified port via native library
        sd = deviceOpen (port);
        if (debug == true)
        {
            System.out.println ("deviceOpen: successful " + sd);
        }
        if (sd < 0)
        {
            throw new IOException("FreebsdSerial: port in use " + name);
        }
        // got the port, now set the default serial commincations parameters
        try
        {
            setSerialPortParams (baud, databits, stopbits, parity);
        }
        catch (UnsupportedCommOperationException e)
        {
            e.printStackTrace ();
        }
    }

    /**
     * <code>finalize</code>
     */
    protected void finalize()
    {
        close();
    }

    /**
     * <code>getBaudRate</code>
     */
    public int getBaudRate()
    {
        return (baud);
    }

    /**
     * <code>getDataBits</code>
     */
    public int getDataBits()
    {
        return (databits);
    }

    /**
     * <code>getStopBits</code>
     */
    public int getStopBits()
    {
        return (stopbits);
    }

    /**
     * <code>getParity</code>
     */
    public int getParity()
    {
        return (parity);
    }

    /**
     * <code>sendBreak</code>
     */
    public void sendBreak(int i)
    {
        if (sd < 0)
        {
            throw new IllegalStateException("FreebsdSerial.sendBreak: closed");
        }
        deviceSendBreak(sd, i);
    }

private native int  deviceOpen (String port) throws IOException;
private native void deviceSendBreak(int sd, int i);
private native void deviceSetReceiveThreshold(int sd, int i);

    protected boolean checkFlowControlModes (int modes)
    {
        boolean check = false;

        switch (modes)
        {
            case FLOWCONTROL_NONE:
                check = true;
                break;
            case FLOWCONTROL_RTSCTS_IN:
                check = true;
                break;
            case FLOWCONTROL_RTSCTS_OUT:
                check = true;
                break;
            case (FLOWCONTROL_RTSCTS_IN | FLOWCONTROL_RTSCTS_OUT):
                check = true;
                break;
            case FLOWCONTROL_XONXOFF_IN:
                check = true;
                break;
            case FLOWCONTROL_XONXOFF_OUT:
                check = true;
                break;
            case (FLOWCONTROL_XONXOFF_IN | FLOWCONTROL_XONXOFF_OUT):
                check = true;
                break;
            default:
                check = false;
                break;
        }
        return (check);
    }


    /** @exception UnsupportedCommOperationException if the comm port could
    * not be set to that mode.
    */
    public void setFlowControlMode(int i)
        throws UnsupportedCommOperationException
    {
        if (sd < 0)
        {
            throw new IllegalStateException(
                          "FreebsdSerial.setFlowControlMode: closed");
        }
        try
        {
            if (checkFlowControlModes (i) == true)
            {
                deviceSetFlowControl(sd, i);
                flowcontrol = i;
            }
            else
            {
                throw new UnsupportedCommOperationException (
                              "FreebsdSerial.setFlowControlMode: " + 
                              "can not mix hardware/software flow control");
            }
        }
        catch (UnsupportedCommOperationException e)
        {
            e.printStackTrace ();
        }
    }

private native void deviceSetFlowControl (int sd, int i);

    public int getFlowControlMode()
    {
        return (flowcontrol);
    }

    /** @exception UnsupportedCommOperationException if the comm port could 
    * not be set to that mode.
    */
    public void setSerialPortParams(int b, int d, int s, int p)
        throws UnsupportedCommOperationException
    {
        if (debug == true)
        {
            System.out.println (b + " " + d + "  " + s + " " + p);
        }
        if (sd < 0)
        {
            throw new IllegalStateException(
                          "FreebsdSerial.setSerialPortParams: closed");
        }
        deviceSetSerialPortParams(sd, b, d, s, p);
        baud = b;
        databits = d;
        stopbits = s;
        parity = p;
    }

private native void deviceSetSerialPortParams(int sd, int b, int d, int s,
                                              int p)
                        throws UnsupportedCommOperationException;

    public void setDTR(boolean flag)
    {
        if (sd < 0)
        {
            throw new IllegalStateException("FreebsdSerial.setDTR: closed");
        }
        deviceSetDTR(sd, flag);
    }

private native void deviceSetDTR(int sd, boolean flag);

    public void setRTS(boolean flag)
    {
        if (sd < 0)
        {
            throw new IllegalStateException("FreebsdSerial.setRTS: closed");
        }
        // check to see if HW flow control is on.  if so then RTS can not be
        // be changed period....
        if ((flowcontrol & FLOWCONTROL_RTSCTS_IN) == 1)
        {
            throw new IllegalStateException(
                         "RTS can not be modify when HW flowcontrol is on.");
        }
        deviceSetRTS(sd, flag);
    }

private native void deviceSetRTS(int sd, boolean flag);

    public boolean isDTR()
    {
        if (sd < 0)            
        {
            throw new IllegalStateException("FreebsdSerial.isRTS: closed");
        }
        return (deviceIsDTR (sd));
    }

private native boolean deviceIsDTR(int sd);

    public boolean isRTS()
    {
        if (sd < 0) 
        {
            throw new IllegalStateException("FreebsdSerial.isRTS: closed");
        }
        return (deviceIsRTS (sd));         
    }

private native boolean deviceIsRTS(int sd);

    public boolean isCTS()
    {
        if (sd < 0) 
        {
            throw new IllegalStateException("FreebsdSerial.isCTS: closed");
        }
        return (deviceIsCTS (sd));         
    }

private native boolean deviceIsCTS(int sd);

    public boolean isDSR()
    {
        if (sd < 0) 
        {
            throw new IllegalStateException("FreebsdSerial.isDSR: closed");
        }
        return (deviceIsDSR (sd));         
    }

private native boolean deviceIsDSR(int sd);

    public boolean isRI()
    {
        if (sd < 0) 
        {
            throw new IllegalStateException("FreebsdSerial.isRI: closed");
        }
        return (deviceIsRI (sd));
    }

private native boolean deviceIsRI(int sd);

    public boolean isCD()
    {
        if (sd < 0) 
        {
            throw new IllegalStateException("FreebsdSerial.isCD: closed");
        }
        return (deviceIsCD (sd));         
    }

private native boolean deviceIsCD(int sd);

private native void deviceEventLoop();

    private void sendEvent(int event, boolean state)
		{
                switch( event ) {
                        case SerialPortEvent.DATA_AVAILABLE:
                                if( monitorData ) break;
                                return;
                        case SerialPortEvent.OUTPUT_BUFFER_EMPTY:
                                if( monitorOutput ) break;
                                return;
                        case SerialPortEvent.CTS:
                                if( monitorCTS ) break;
                                return;
                        case SerialPortEvent.DSR:
                                if( monitorDSR ) break;
                                return;
                        case SerialPortEvent.RI:
                                if( monitorRI ) break;
                                return;
                        case SerialPortEvent.CD:
                                if( monitorCD ) break;
                                return;
                        case SerialPortEvent.OE:
                                if( monitorOE ) break;
                                return;
                        case SerialPortEvent.PE:
                                if( monitorPE ) break;
                                return;
                        case SerialPortEvent.FE:
                                if( monitorFE ) break;
                                return;
                        case SerialPortEvent.BI:
                                if( monitorBI ) break;
                                return;
                        default:
                                return;
                }
                SerialPortEvent e = new SerialPortEvent(this, event, !state,state );
                if( SPEventListener != null ) SPEventListener.serialEvent( e );
        }

    /** Adds an event listener to detect errors and data availability.
    * @exception TooManyListenersException if the port has already a listener 
    */
    public void addEventListener ( SerialPortEventListener serialPortEventListener )
        throws TooManyListenersException
    {
	if( SPEventListener != null ) throw new TooManyListenersException();
             SPEventListener = serialPortEventListener;
             monitorThread = new Thread() 
		{
               	public void run() 
		     {
                     deviceEventLoop();
                     }    
                 };

        monitorThread.setDaemon(true);
        monitorThread.start();
    }

    public void removeEventListener()
    {
 	SPEventListener = null;
       	if( monitorThread != null ) 
	   {
           monitorThread.interrupt();
           monitorThread = null;
           }
    }

	public void notifyOnDataAvailable( boolean enable ) { monitorData = enable; }

        public void notifyOnOutputEmpty( boolean enable ) { monitorOutput = enable; }

        public void notifyOnCTS( boolean enable ) { monitorCTS = enable; }

        public void notifyOnDSR( boolean enable ) { monitorDSR = enable; }

        public void notifyOnRingIndicator( boolean enable ) { monitorRI = enable; }

        public void notifyOnCarrierDetect( boolean enable ) { monitorCD = enable; }

        public void notifyOnOverrunError( boolean enable ) { monitorOE = enable; }

        public void notifyOnParityError( boolean enable ) { monitorPE = enable; }

        public void notifyOnFramingError( boolean enable ) { monitorFE = enable; }

        public void notifyOnBreakInterrupt( boolean enable ) { monitorBI = enable; }

//
// methods from CommPort interface
//
    /** Returns a stream descripting port's input.
    * @exception IOException if the stream could not be created 
    */
    public InputStream getInputStream() throws IOException
    {
        if (sd < 0)
        {   
            throw new IllegalStateException (
                          "FreebsdSerial.getInputStream: closed");
        }

        if (input == null)
        {
            input = new SerialInputStream(sd);
        }
        if (debug == true)
        {
            System.out.println ("FreebsdSerial.getInputStream: created");
        }
        return (input);
    }

    /** Returns a stream descripting port's output.
    * @exception IOException if the stream could not be created
    */
    public OutputStream getOutputStream() throws IOException
    {
        if (sd < 0)
        {
            throw new IllegalStateException (
                          "FreebsdSerial.getOutputStream: closed");
        }
    
        if (output == null)      
        {
            output = new SerialOutputStream(sd);                  
        }
        if (debug == true)
        {
            System.out.println ("FreebsdSerial.getOutputStream: created");
        }
        return (output);        
    }

    public void close()
    {
        removeEventListener();
        try
        {
            if (sd < 0)
            {
                throw new IllegalStateException ("FreebsdSerial.close: closed");
            }
            deviceClose (sd);
            sd = -1;
            if (input != null)
            {
                input.close ();
                input = null;
            }
            if (output != null)
            {
                output.close ();
                output = null;
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        super.close();
    }

private native void deviceClose (int sd) throws IOException;
private native int  deviceAvailable(int sd) throws IOException;
private native int  deviceRead(int sd, byte b[], int offset, int length, int timeout)
                        throws IOException;
private native int  deviceWrite(int sd, byte b[], int offset, int length)
                        throws IOException;
private native void deviceFlush(int sd);


    public void setInputBufferSize(int i)
    {
        if (sd < 0)            
        {    
            throw new IllegalStateException (
                          "FreebsdSerial.setInputBufferSize: closed");
        }
        ibs = i;
    }

    public int getInputBufferSize()
    {
        return ibs;
    }

    public void setOutputBufferSize(int i)
    {
        if (sd < 0)
        {
            throw new IllegalStateException (
                          "FreebsdSerial.setOutputBufferSize: closed");
        }
        obs = i;
    }

    public int getOutputBufferSize()
    {
        return obs;
    }

    /** Enables or disables receive framing 
    * @exception UnsupportedCommOperationException if the comm port could
    * not be set to that mode.
    */
    public void enableReceiveThreshold(int i)
        throws UnsupportedCommOperationException
    {
        threshold = i;
        deviceSetReceiveThreshold(sd, threshold);
    }
 
    public void disableReceiveThreshold()
    {
        threshold = -1;
        deviceSetReceiveThreshold(sd, threshold);
    }

    public boolean isReceiveThresholdEnabled()
    {
        boolean enabled = true;

        if (threshold == -1)
        {
            enabled = false;
        }
        return (enabled);
    }

    public int getReceiveThreshold()
    {
        return (threshold);
    }

    /** @exception UnsupportedCommOperationException if the comm port could
    * not be set to that mode.
    */
    public void enableReceiveTimeout(int i)
        throws UnsupportedCommOperationException
    {
        timeout = i;
    }

    public void disableReceiveTimeout()
    {
        timeout = -1;
    }

    public boolean isReceiveTimeoutEnabled()
    {
        boolean enabled = true;

        if (timeout == -1)
        {
            enabled = false;
        }
        return (enabled);
    }

    public int getReceiveTimeout()
    {
        return (timeout);
    }

    /** @exception UnsupportedCommOperationException if the comm port could
    * not be set to that mode.
    */
    public void enableReceiveFraming(int i)
        throws UnsupportedCommOperationException
    {
        framing = i;
    }

    public void disableReceiveFraming()
    {
        framing = -1;
    }

    public boolean isReceiveFramingEnabled()
    {
        boolean enabled = true;
        
        if (framing == -1)
        {   
            enabled = false;
        }
        return (enabled);
    }

    public int getReceiveFramingByte()
    {
        return (framing);
    }

//
// inner class that extends an InputStream by overriding the read methods 
// by making them call the native library for the serial ports
//
    class SerialInputStream extends InputStream
    {
        private int     sd   = -1;
        private boolean open = false;

        /**
         * class constructor
         */
        public SerialInputStream(int fd)
        {
            sd = fd;
            open = true;
        }

        /**
         * <code>available</code> is used to determine if data is available on
         * on the the underlying device or not.
         */
        public int available() throws IOException
        {
            int avail = 0;

            /*
             * first check to see if the input stream is open or not...
             */
            if (open == false)
            {
                throw new IllegalStateException("SerialInputStream.available:");
            }
            avail = deviceAvailable(sd);
            if (debug == true)
            {
                System.out.println ("SerialInputStream.available: " + avail);
            }
            return (avail);
        }

        /**
         * <code>read</code> is used to read a single byte of data from the
         * underlying input device
         */
        public int read() throws IOException
        {
            byte  b[] = new byte[1];
            int   i   = 0;
            int   v   = -1;
            
            i = deviceRead (sd, b, 0, 1, timeout);
            if (i == 1)
            {
                v = b[0] & 255;
            }
            if (debug == true)
            {
                System.out.println ("SerialInputStream.read: " + v);
            }
            return (v);
        }

        /**
         * <code>read</code> is used to read enough bytes to fill the specified 
         * array.
         */
        public int read(byte[] b) throws IOException
        {
            int   i   = 0;

            i = deviceRead (sd, b, 0, b.length, timeout);
            if (debug == true)
            {
                System.out.println ("SerialInputStream.read: length " + i + 
                                    " data: " + new String (b));
            }
            return (i);
        }

        /**
         * <code>read</code> is used to read length bytes starting at offset
         * and store them in the specified array
         */
        public int read(byte b[], int offset, int length) throws IOException
        {
            int i = 0;

            i = deviceRead (sd, b, offset, length, timeout);
            if (debug == true)
            {
                System.out.println ("SerialInputStream.read: length " + i + 
                                    " data: " + new String (b));
            }
            return (i);
        }
    }

//
// inner class that extends an OutputStream by overriding the write methods 
// by making them call the native library for the serial ports
//
    class SerialOutputStream extends OutputStream
    {
        private int      sd   = -1;
        private boolean  open = false;

        /**
         * class constructor
         */
        public SerialOutputStream(int fd)
        {
            sd = fd;		// save the file descriptor.
            open = true;
        }

        /**
         * <code>close</code> is used to signal that the output stream has been 
         * closed and is no longer available for writing....
         */
        public void close()
        {
            open = false;
            sd = -1;
        }

        /**
         * <code>write</code> is used to write a single value to the underlying
         * serial device by calling the native write method for the
         * OutputStream
         */
        public void write(int b) throws IOException
        {
            byte data[] = new byte[1];
            int  i      = 0;

            /*
             * check to see if someone has closed us....
             */
            if (open == false)
            {
                throw new IllegalStateException("SerialOutputStream.write:");
            }
            data[0] = (byte)b;
            i = deviceWrite(sd, data, 0, 1);
            if (debug == true)
            {
                System.out.println ("SerialInputStream.write: " + b);
            }
// GRS - should really have a return here to let the caller know how much data
//       was written to the serial device....
        }

        /**
         * <code>write</code> is used to write an array of data to the under-
         * lying serial devive by calling the native write method for the
         * OutputStream
         */
        public void write(byte[] b) throws IOException
        {
            /*
             * check to see if someone has closed us....
             */
            if (open == false)
            {
                throw new IllegalStateException("SerialOutputStream.write:");
            }
            write(b, 0, b.length); 
        }   

        /**
         * <code>write</code> is used to call the devices write method to actually
         * write the byte array to the device...
         */
        public void write(byte[] b, int offset, int length) throws IOException
        {
           int i = 0;

            /*
             * first check to see if the output stream is open or not...
             */
            if (open == false)
            {
                throw new IllegalStateException("SerialOutputStream.write: closed");
            }
            i = deviceWrite (sd, b, offset, length);
        }

        /**
         * <code>flush</code> is used to flush the contents of the output stream to
         * the device
         */
        public void flush() throws IOException
        {
            /*
             * first check to see if the output stream is open or not...
             */
            if (open == false)
            {
                throw new IllegalStateException("SerialOutputStream.flush: close");
            }
            deviceFlush(sd);
        }
    }
 
}
//  EOF
