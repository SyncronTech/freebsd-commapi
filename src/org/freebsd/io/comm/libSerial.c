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

#include <jni.h>
#include "org_freebsd_io_comm_FreebsdSerial.h"
#include "libSerial.h"

#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ttycom.h>

#define IOEXCEPTION	"java/io/IOException"
#define USCOEXCEPTION	"javax/comm/UnsupportedCommOperationException"

extern int errno;

void throw_exception (JNIEnv *env, char *exc, char *foo, char *msg)
{
    char    buf [128];
    jclass clazz = (*env)->FindClass (env, exc);
    
    if (!clazz)
    {
        (*env)->ExceptionDescribe (env);
        (*env)->ExceptionClear (env);
        return;
    }  
    snprintf (buf, 128, "%s in %s", msg, foo);
    (*env)->ThrowNew (env, clazz, buf);
} 

JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdSerial_Initialize
  (JNIEnv *env, jclass jclazz)
{
    /* do nothing.... */
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceOpen
 * Signature: (Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceOpen
  (JNIEnv *env, jobject jobj, jstring jstr)
{
    int             fd   = -1;
    struct termios  tty;
    const  char    *port = NULL;

    /*
     * Java uses UTF to store text type data so we must convert to good old
     * 8-bit ascii in order to read it...
     */
    port = (*env)->GetStringUTFChars (env, jstr, 0);

    /*
     * O_RDWR     = Read/Write access to the port
     * O_NOCTTY   = do not assign a controlling terminal
     * O_NONBLOCK = open in non blocking mode
     */
    fd = open (port, O_RDWR | O_NOCTTY | O_NONBLOCK);

    (*env)->ReleaseStringUTFChars (env, jstr, NULL);

    /*
     * if the file descriptor is still < 0 then an error occurred in the open
     */
    if (fd < 0)
    {
        return (-1);
    }

    /*
     * setup communications port for default of 9600, 8, 1, none
     */
    tty.c_iflag = INPCK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag = CREAD | CS8;
    tty.c_cc [VMIN] = 0;
    tty.c_cc [VTIME] = 1;
    if (cfsetspeed (&tty, B9600) <0)
    {
        throw_exception (env, IOEXCEPTION, "cfsetspeed ", strerror (errno));
        return (-1);
    }
    if (tcsetattr (fd, TCSAFLUSH, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcsetattr ", strerror (errno));
        return (-1);
    }
    fcntl (fd, F_SETOWN, getpid ());
    fcntl (fd, F_SETFL, FASYNC);
    return ((jint)fd);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceSendBreak
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceSendBreak
  (JNIEnv *env, jobject jobj, jint sd, jint i)
{
    /*
     * Freebsd ignores the len parameter according to the man pages...
     */
    tcsendbreak ((int)sd, 0);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceSetFlowControl
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceSetFlowControl
  (JNIEnv *env, jobject jobj, jint sd, jint i)
{
    struct termios tty;

    /* get termios structure for our serial port */
    if (tcgetattr ((int)sd, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcgetattr ", strerror (errno));
        return;
    }
    switch ((int)i)
    {
        case 0:		/* SerialPort.FLOWCONTROL_NONE */
            tty.c_cflag &= ~ (IXON | IXOFF | CRTSCTS);
            break;
        case 1:		/* SerialPort.FLOWCONTROL_RTSCTS_IN */
            tty.c_cflag |= CRTS_IFLOW;
            break;
        case 2:		/* SerialPort.FLOWCONTROL_RTSCTS_OUT */
            tty.c_cflag |= CCTS_OFLOW;
            break;
        case 3:		/* SerialPort.FLOWCONTROL_RTSCTS_IN/OUT */
            tty.c_cflag |= CRTSCTS;
            break;
        case 4:		/* SerialPort.FLOWCONTROL_XONXOFF_IN */
            tty.c_cflag |= IXOFF;
            break;
        case 8:		/* SerialPort.FLOWCONTROL_XONXOFF_OUT */
            tty.c_cflag |= IXON;
            break;
        case 12:	/* SerialPort.FLOWCONTROL_XONXOFF_IN/OUT */
            tty.c_cflag |= IXON | IXOFF;
            break;
    }
    if (tcsetattr ((int)sd, TCSAFLUSH, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcsetattr ", strerror (errno));
        return;
    }
}

tcflag_t set_stopbits (JNIEnv *env, jint sbits, tcflag_t c_cflag)
{
    tcflag_t c = c_cflag & ~CSIZE;

    switch (sbits)
    {
        case 1:         /* SerialPort.STOPBITS_1 */
            /*
             * clear the two stop bits field in the control flags
             */
            c_cflag &= ~CSTOPB;
            break;
        case 2:         /* SerialPort.STOPBITS_2 */
            /*
             * set the two stop bits field in the control flags
             */
            c_cflag |= CSTOPB;
            break;
        case 3:         /* SerialPort.STOPBITS_1_5 */
        default:
            throw_exception (env, USCOEXCEPTION, "set_stopbits ",
                             "invalid stopbits specified");
            break;
    }
    return (c_cflag);
}

tcflag_t set_databits (JNIEnv *env, jint dbits, tcflag_t c_cflag)
{
    tcflag_t c = c_cflag & ~CSIZE;

    switch (dbits)
    {
        case 5:		/* SerialPort.DATABITS_5 */
            c_cflag = (c | CS5);
            break;
        case 6:		/* SerialPort.DATABITS_6 */
            c_cflag = (c | CS6);
            break;
        case 7:		/* SerialPort.DATABITS_7 */
            c_cflag = (c | CS7);
            break;
        case 8:		/* SerialPort.DATABITS_8 */
            c_cflag = (c | CS8);
            break;
        default:
            throw_exception (env, USCOEXCEPTION, "set_databits ", 
                             "invalid databits specified");
            break;
    }
    return (c_cflag);
}

tcflag_t set_parity (JNIEnv *env, jint parity, tcflag_t c_cflag)
{
    switch (parity)
    {
        case 0:                    /* SerialPort.PARITY_NONE */
            /*
             * here we clear the parity enabled bit in the control flags
             */
            c_cflag &= ~PARENB;
            break;
        case 1:                    /* SerialPort.PARITY_ODD */
            /*
             * here we set the parity enabled and odd bits in the control flags
             */
            c_cflag |= (PARENB | PARODD);
            break;
        case 2:                    /* SerialPort.PARITY_EVEN */
            /*
             * here we set the parity enabled and clear the odd parity bits
             */
            c_cflag |= PARENB;
            c_cflag &= ~PARODD;
            break;
        case 3:                    /* SerialPort.PARITY_MARK */
        case 4:                    /* SerialPort.PARITY_SPACE */
        default:
            throw_exception (env, USCOEXCEPTION, "set_parity ",
                             "unsupported parity specified");
            break;
    }
    return (c_cflag);
}   

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceSetSerialPortParams
 * Signature: (IIIII)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceSetSerialPortParams
  (JNIEnv *env, jobject jobj, jint sd, jint b, jint d, jint s, jint p)
{
    struct termios tty;

    if (tcgetattr ((int)sd, &tty) < 0)
    {
        throw_exception (env, USCOEXCEPTION, "SetSerialPortParams ",
                         strerror (errno));
    }
    if (cfsetspeed (&tty, (speed_t)b) < 0)
    {
        throw_exception (env, USCOEXCEPTION, "SetSerialPortParams ",
                         strerror (errno));
    }
    tty.c_cflag = set_parity (env, p, tty.c_cflag);
    tty.c_cflag = set_databits (env, d, tty.c_cflag);
    tty.c_cflag = set_stopbits (env, s, tty.c_cflag);

    if (tcsetattr ((int)sd, TCSAFLUSH, &tty) < 0)
    {
        throw_exception (env, USCOEXCEPTION, "SetSerialPortParams ",
                         strerror (errno));
    }
    return;
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceSetDTR
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceSetDTR
  (JNIEnv *env, jobject jobj, jint sd, jboolean flag)
{
    struct termios tty;
    
    /* get termios structure for our serial port */
    if (tcgetattr ((int)sd, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcgetattr ", strerror (errno));
        return;
    }

    if (flag == JNI_TRUE)
    {
        tty.c_cflag |= CDTR_IFLOW;
    }
    else
    {
        tty.c_cflag &= ~CDTR_IFLOW;
    }

    if (tcsetattr ((int)sd, TCSAFLUSH, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcsetattr ", strerror (errno));
    }
    return;
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceSetRTS
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceSetRTS
  (JNIEnv *env, jobject jobj, jint sd, jboolean flag)
{
    struct termios tty;
                             
    /* get termios structure for our serial port */
    if (tcgetattr ((int)sd, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcgetattr ", strerror (errno));
        return;
    }
 
    if (flag == JNI_TRUE)
    {
        tty.c_cflag |= CRTS_IFLOW;
    }
    else    
    {
        tty.c_cflag &= ~CRTS_IFLOW;
    }
            
    if (tcsetattr ((int)sd, TCSAFLUSH, &tty) < 0)
    {
        throw_exception (env, IOEXCEPTION, "tcsetattr ", strerror (errno));
    }
    return;    
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceClose
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceClose
  (JNIEnv *env, jobject jobj, jint sd)
{
    close ((int)sd);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceAvailable
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceAvailable
  (JNIEnv *env, jobject jobj, jint sd)
{
    int avail = 0;

    /*
     * this ioctl call will tell us if data is waiting on the port for us...
     */
    if (ioctl ((int)sd, FIONREAD, &avail))
    {
        return (-1);  		/* call failed.... */
    }
    else
    {
        return ((jint)avail);	/* return number of bytes available */
    }
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceRead
 * Signature: (I[BII)I
 */
JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceRead
  (JNIEnv *env, jobject jobj, jint sd, jbyteArray b, jint offset, jint length)
{
    int       ret = 0;
    jbyte    *bytes;
    jboolean  isCopy;

    bytes = (*env)->GetByteArrayElements (env, b, &isCopy);
    ret = read ((int)sd, bytes, (size_t)length);
    (*env)->ReleaseByteArrayElements (env, b, bytes, 0);
    return (ret);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceWrite
 * Signature: (I[BII)I
 */
JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceWrite
  (JNIEnv *env, jobject jobj, jint sd, jbyteArray b, jint offset, jint length)
{
    int       ret = 0;
    jbyte    *bytes;
    jboolean  isCopy;

    bytes = (*env)->GetByteArrayElements (env, b, &isCopy);
    ret = write ((int)sd, bytes, (size_t)length);
    tcdrain ((int)sd);
    (*env)->ReleaseByteArrayElements (env, b, bytes, 0);
    return (ret); 
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceFlush
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceFlush
  (JNIEnv *env, jobject jobj, jint sd)
{
    tcdrain ((int)sd);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceIsDTR
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceIsDTR
  (JNIEnv *env, jobject jobj, jint sd)
{
    unsigned int value = 0;
    jboolean     set   = JNI_FALSE;

    ioctl ((int)sd, TIOCMGET, &value);
    if (value & TIOCM_DTR)
    {
        set = JNI_TRUE;
    }
    return (set);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceIsRTS
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceIsRTS
  (JNIEnv *env, jobject jobj, jint sd)
{
    unsigned int value = 0;
    jboolean     set   = JNI_FALSE;

    ioctl ((int)sd, TIOCMGET, &value);
    if (value & TIOCM_RTS)
    {
        set = JNI_TRUE;
    }
    return (set);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceIsCTS
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceIsCTS
  (JNIEnv *env, jobject jobj, jint sd)
{
    unsigned int value = 0;
    jboolean     set   = JNI_FALSE;

    ioctl ((int)sd, TIOCMGET, &value);
    if (value & TIOCM_CTS)
    {
        set = JNI_TRUE;
    }
    return (set);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceIsDSR
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceIsDSR
  (JNIEnv *env, jobject jobj, jint sd)
{
    unsigned int value = 0;
    jboolean     set   = JNI_FALSE;

    ioctl ((int)sd, TIOCMGET, &value);
    if (value & TIOCM_DSR)
    {
        set = JNI_TRUE;
    }
    return (set);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceIsRI
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceIsRI
  (JNIEnv *env, jobject jobj, jint sd)
{
    unsigned int value = 0;
    jboolean     set   = JNI_FALSE;

    ioctl ((int)sd, TIOCMGET, &value);
    if (value & TIOCM_RI)
    {
        set = JNI_TRUE;
    }
    return (set);
}

/*
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceIsCD
 * Signature: (I)Z
 */
JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceIsCD
  (JNIEnv *env, jobject jobj, jint sd)
{
    unsigned int value = 0;
    jboolean     set   = JNI_FALSE;

    ioctl ((int)sd, TIOCMGET, &value);
    if (value & TIOCM_CD)
    {
        set = JNI_TRUE;
    }
    return (set);
}

/* 
 * Class:     org_freebsd_io_comm_FreebsdSerial
 * Method:    deviceEventLoop
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdSerial_deviceEventLoop 
	(JNIEnv *env, jobject jobj)
{      
	int state,old_state;
	int fd;
	fd_set rfds;
	struct timeval sleep;
	int size;
	int ret;
	
        jfieldID jfield;  
        jmethodID method, interrupt;
        jboolean interrupted = 0;
        jclass jclazz, jthread;
        jclazz = (*env)->GetObjectClass( env, jobj );
        jfield = (*env)->GetFieldID( env, jclazz, "sd", "I" );
        fd = (int)( (*env)->GetIntField( env, jobj, jfield ) );
        method = (*env)->GetMethodID( env, jclazz, "sendEvent", "(IZ)V" );
        jthread = (*env)->FindClass( env, "java/lang/Thread" );
        interrupt = (*env)->GetStaticMethodID( env, jthread, "interrupted", "()Z" );
                                                                                
        FD_ZERO( &rfds );
        FD_SET( fd, &rfds );
        sleep.tv_sec = 1; /* Check every 1 second, or on receive data */
        sleep.tv_usec = 0;
         
        /* Initialization of the current tty state */
        ioctl( fd, TIOCMGET, &old_state);                                                                          
  
  	while( !interrupted ) 
  	{
  		do 
  			{
  			ret=select( fd + 1, &rfds, NULL, NULL, &sleep );
  			}  
  		while ( (ret < 0) && (errno==EINTR));
 
	/* Check for new state on port */
  	ioctl( fd, TIOCMGET, &state);
  	
  	/* Check for data on port */
  	ioctl( fd, FIONREAD, &size );
 
	/*
	PROBLEM : I don't know how to capture the BI,FE,OE,PE, OUTPUT_BUFFER_EMPTY events !!!!!
	if you have ideas, suggestions patches, send them to <jdricot@ulb.ac.be>
	thanks a lot !
	*/

  	if (state!=old_state)
  		{
	  		
  		if( state & TIOCM_CD ) (*env)->CallVoidMethod( env, jobj, method,(jint)SPE_CD, JNI_TRUE );
  		if( state & TIOCM_RI ) (*env)->CallVoidMethod( env, jobj, method,(jint)SPE_RI, JNI_TRUE );
  		if( state & TIOCM_CTS ) (*env)->CallVoidMethod( env, jobj, method,(jint)SPE_CTS, JNI_TRUE );
		if( state & TIOCM_DSR ) (*env)->CallVoidMethod( env, jobj, method,(jint)SPE_DSR, JNI_TRUE );
  		old_state=state;
  		}
  		
  	if ( size > 0 )
  		{
  		(*env)->CallVoidMethod( env, jobj, method,(jint)SPE_DATA_AVAILABLE, JNI_TRUE );
  		}
  	interrupted = (*env)->CallStaticBooleanMethod( env, jthread, interrupt );
  	}
}
/*  EOF */
