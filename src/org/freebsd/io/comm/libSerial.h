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

/* javax.comm.SerialPort constants */
#define DATABITS_5		5
#define DATABITS_6		6
#define DATABITS_7		7
#define DATABITS_8		8
#define STOPBITS_1		1
#define STOPBITS_2		2
#define PARITY_NONE		0
#define PARITY_ODD		1
#define PARITY_EVEN		2
#define PARITY_MARK		3
#define PARITY_SPACE		4
#define FLOWCONTROL_NONE	0
#define FLOWCONTROL_RTSCTS_IN	1
#define FLOWCONTROL_RTSCTS_OUT	2
#define FLOWCONTROL_XONXOFF_IN	4
#define FLOWCONTROL_XONXOFF_OUT	8


/* javax.comm.SerialPortEvent constants */
#define SPE_DATA_AVAILABLE       1
#define SPE_OUTPUT_BUFFER_EMPTY  2
#define SPE_CTS                  3
#define SPE_DSR                  4
#define SPE_RI                   5
#define SPE_CD                   6
#define SPE_OE                   7
#define SPE_PE                   8
#define SPE_FE                   9
#define SPE_BI                  10

/* java exception class names */
#define UNSUPPORTED_COMM_OPERATION "javax/comm/UnsupportedCommOperationException"
#define ARRAY_INDEX_OUT_OF_BOUNDS "java/lang/ArrayIndexOutOfBoundsException"
#define OUT_OF_MEMORY "java/lang/OutOfMemoryError"
#define IO_EXCEPTION "java/io/IOException"

