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
import java.util.*;
import javax.comm.*;

/**
  * ParallelPort
  */
public final class FreebsdParallel extends ParallelPort {

	static {
		System.loadLibrary( "Parallel" );
		Initialize();
	}

	/** Initialize the native library */
	private native static void Initialize();

	/** Open the named port */
	public FreebsdParallel( String name ) throws IOException {
		fd = open( name );
	}
	private native int open( String name ) throws IOException;

	/** File descriptor */
	private int fd;

	/** Output stream */
	private final ParallelOutputStream out = new ParallelOutputStream();
	public OutputStream getOutputStream() { return out; }

	/** Input stream */
	private final ParallelInputStream in = new ParallelInputStream();
	public InputStream getInputStream() { return in; }

	/** return current mode LPT_MODE_SPP, LPT_MODE_PS2, LPT_MODE_EPP, 
	    or LPT_MODE_ECP */
	private int lprmode;
	public int getMode() { return lprmode; }
	public int setMode(int mode) {
		lprmode = mode;
		try {
			setLPRMode();
		} catch(UnsupportedCommOperationException e) {
			e.printStackTrace();
                        return -1;
		}
		return(0);
	}
	public void restart(){};
	public void suspend(){};
	
	public native boolean setLPRMode() throws UnsupportedCommOperationException;
	public native boolean isPaperOut();
	public native boolean isPrinterBusy();
	public native boolean isPrinterError();
	public native boolean isPrinterSelected();
	public native boolean isPrinterTimedOut();
	
	/** Close the port */
	public native void deviceClose();
	public void close()
        {
              removeEventListener();
              deviceClose();
        }

	/** Receive framing control 
        *@exception UnsupportedCommOperationException if the device does not
	*support this mode
	*/
	public void enableReceiveFraming( int f )
		throws UnsupportedCommOperationException
	{
		throw new UnsupportedCommOperationException( "Not supported yet" );
	}
	public void disableReceiveFraming() {}
	public boolean isReceiveFramingEnabled() { return false; }
	public int getReceiveFramingByte() { return 0; }

	/** Receive timeout control */
	private int timeout = 10;
	public void enableReceiveTimeout( int t ) {
		if( t > 0 ) timeout = t;
		else timeout = 0;
	}

	public void disableReceiveTimeout() { timeout = 0; }
	public boolean isReceiveTimeoutEnabled() { return (timeout > 0); }
	public int getReceiveTimeout() { return timeout; }

	/** Receive threshold control */
	private int threshold = 1;
	public void enableReceiveThreshold( int t ) {
		if( t > 1 ) threshold = t;
		else threshold = 1;
	}
	public void disableReceiveThreshold() { threshold = 1; }
	public int getReceiveThreshold() { return threshold; }
	public boolean isReceiveThresholdEnabled() { return threshold > 1; };

	/** Input/output buffers */
	int inputBufferSize=1;
	public void setInputBufferSize( int _size ) {
		inputBufferSize=_size;
	}
	public int getInputBufferSize() {
		return inputBufferSize;
	}
	int outputBufferSize=1;
	public void setOutputBufferSize( int _size ) {
		outputBufferSize=_size; 
	}
	public int getOutputBufferSize() {
		return outputBufferSize;
	}

	public native int getOutputBufferFree();

	/** Write to the port */
	private native void writeByte( int b ) throws IOException;
	private native void writeArray( byte b[], int off, int len )
		throws IOException;
	private native void drain() throws IOException;

	/** Read from the port */
	private native int nativeavailable() throws IOException;
	private native int getAvailableDataSize() throws IOException;
	private native int readByte() throws IOException;
	private native int readArray( byte b[], int off, int len ) 
		throws IOException;

	/** Parallel Port Event listener */
	private ParallelPortEventListener PPEventListener;

	/** Thread to monitor data */
	private Thread monitorThread;

	/** Process ParallelPortEvents */
	native void eventLoop();
	void sendEvent( int event, boolean state ) {
		switch( event ) {
			case ParallelPortEvent.PAR_EV_BUFFER:
				if( monitorBuffer ) break;
			case ParallelPortEvent.PAR_EV_ERROR:
				if( monitorError ) break;
			default:
				return;
		}
		ParallelPortEvent e = new ParallelPortEvent(this, event, !state, state );
		if( PPEventListener != null ) PPEventListener.parallelEvent( e );
	}

	/** Add an event listener 
	*@exception TooManyListenersException if a listener is already
	* present
	*/
	public void addEventListener( ParallelPortEventListener lsnr )
		throws TooManyListenersException
	{
		if( PPEventListener != null ) throw new TooManyListenersException();
		PPEventListener = lsnr;
		monitorThread = new Thread() {
			public void run() {
				eventLoop();
			}
		};
               
		monitorThread.setDaemon(true);
		monitorThread.start(); 
	}

	/** Remove the parallel port event listener */
	public void removeEventListener() {
		PPEventListener = null;
		if( monitorThread != null ) {
			monitorThread.interrupt();
			monitorThread = null;
		}
		/* FIXME: Should we reset all the notify flags here? */
	}

	/** Note: these have to be separate boolean flags because the
	   ParallelPortEvent constants are NOT bit-flags, they are just
	   defined as integers from 1 to 10  -DPL */
	private boolean monitorError = false;
	public void notifyOnError( boolean enable ) { monitorError = enable; }
	private boolean monitorBuffer = false;
	public void notifyOnBuffer( boolean enable ) { monitorBuffer = enable; }


	/** Finalize the port */
	protected void finalize() {
		close();
	}

        /** Inner class for ParallelOutputStream */
        class ParallelOutputStream extends OutputStream {
                public void write( int b ) throws IOException {
                        writeByte( b );
                }
                public void write( byte b[] ) throws IOException {
                        writeArray( b, 0, b.length );
                }
                public void write( byte b[], int off, int len ) throws IOException {
                        writeArray( b, off, len );
                }
                public void flush() throws IOException {
                        drain();
                }
        }

	/** Inner class for ParallelInputStream */
	class ParallelInputStream extends InputStream {
		public int read() throws IOException {
			return readByte();
		}
		public int read( byte b[] ) throws IOException {
			return readArray( b, 0, b.length );
		}
		public int read( byte b[], int off, int len ) throws IOException {
			return readArray( b, off, len );
		}
		public int available() throws IOException {
			return nativeavailable();
		}
	}
}
