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
import javax.comm.*;

/**
   This is the JAVA Comm Driver for FreeBSD.  
*/
public class FreebsdDriver implements CommDriver {


   /*
    * initialize() will be called by the CommPortIdentifier's static
    * initializer. The responsibility of this method is:
    * 1) Ensure that that the hardware is present.
    * 2) Load any required native libraries.
    * 3) Register the port names with the CommPortIdentifier.
	 * 
	 * <p>From the NullDriver.java CommAPI sample.
    */
	public void initialize() {
		File dev = new File( "/dev" );
		String[] devs = dev.list();

		String[] portPrefix = { "cuaa" };
		for( int i = 0; i < devs.length; i++ ) {
			for( int p = 0; p < portPrefix.length; p++ ) {
				if( devs[i].startsWith( portPrefix[p] ) ) {
					String portName = "/dev/" + devs[i];
					File port = new File( portName );
					if( port.canRead() && port.canWrite() ) 
						{
						CommPortIdentifier.addPortName( portName,
							CommPortIdentifier.PORT_SERIAL, this );
						}
				}
			}
		}

                for( int i = 0; i < devs.length; i++ ) {
                	if( devs[i].startsWith( "ppi" ) ) {
                       		String portName = "/dev/" + devs[i];
                               	File port = new File( portName );
                          	if( port.canRead() && port.canWrite() )
					{
					CommPortIdentifier.addPortName( portName,
						CommPortIdentifier.PORT_PARALLEL, this );
					}
                	}
                }
	}


	/*
	 * getCommPort() will be called by CommPortIdentifier from its openPort()
	 * method. portName is a string that was registered earlier using the
	 * CommPortIdentifier.addPortName() method. getCommPort() returns an
	 * object that extends either SerialPort or ParallelPort.
	 *
	 * <p>From the NullDriver.java CommAPI sample.
	 */
	public CommPort getCommPort( String portName, int portType ) {
		try {
			if (portType==CommPortIdentifier.PORT_SERIAL)
				return new FreebsdSerial( portName ); 
			else if (portType==CommPortIdentifier.PORT_PARALLEL)
				return new FreebsdParallel( portName ); 
		} catch( IOException e ) {
			e.printStackTrace();
		}
		return null;
	}
}
