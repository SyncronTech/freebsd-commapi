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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
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

#include "org_freebsd_io_comm_FreebsdParallel.h"
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include </sys/dev/ppbus/ppi.h>
#include </sys/dev/ppbus/ppbconf.h>
#include "libParallel.h"
#include <signal.h>
#include <fcntl.h>

JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdParallel_getOutputBufferFree(JNIEnv *env,
        jclass jclazz) {

	return(1);

}

JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdParallel_setLPRMode(JNIEnv *env,
        jclass jclazz) {
/*throws UnsupportedCommOperationException;*/
}

JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdParallel_isPaperOut(JNIEnv *env,
	jobject jobj){

	int status;
	int fd = get_java_fd( env, jobj );

	ioctl(fd, PPIGSTATUS ,&status);
	return( (status & PERROR) ? JNI_TRUE : JNI_FALSE );
}

JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdParallel_isPrinterBusy(JNIEnv *env,
	jobject jobj){
	int status;
	int fd = get_java_fd( env, jobj );
	ioctl(fd,  PPIGSTATUS , &status);
	return( !(status & nBUSY) ? JNI_TRUE : JNI_FALSE );
}

JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdParallel_isPrinterError(JNIEnv *env,
	jobject jobj){
	int status;
	int fd = get_java_fd( env, jobj );
	ioctl(fd,  PPIGSTATUS, &status);
	return( !(status & nFAULT) ? JNI_TRUE : JNI_FALSE );
}

JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdParallel_isPrinterSelected(JNIEnv *env,
	jobject jobj){
	int status;
	int fd = get_java_fd( env, jobj );
	ioctl(fd,  PPIGSTATUS , &status);
	return( (status & SELECT) ? JNI_TRUE : JNI_FALSE );
}

JNIEXPORT jboolean JNICALL Java_org_freebsd_io_comm_FreebsdParallel_isPrinterTimedOut(JNIEnv *env,
	jobject jobj){
	int status;
	int fd = get_java_fd( env, jobj );
	ioctl(fd,  PPIGSTATUS , &status);
	return( (status & TIMEOUT) ? JNI_TRUE : JNI_FALSE );
}


JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdParallel_Initialize( JNIEnv *env,
	jclass jclazz )
{

}


JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdParallel_open( JNIEnv *env, jobject jobj,
	jstring jstr )
{
	u_int8_t control;
	int fd;
	const char *filename = (*env)->GetStringUTFChars( env, jstr, 0 );
	(*env)->ReleaseStringUTFChars( env, jstr, NULL );

	if( (fd = open( filename, O_RDWR )) < 0 ) goto fail;

	return (jint)fd;

fail:
	IOException( env, strerror( errno ) );
	return -1;
}


JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdParallel_deviceClose( JNIEnv *env,
	jobject jobj )
{
	int fd = get_java_fd( env, jobj );
	jclass jclazz = (*env)->GetObjectClass( env, jobj );
	jfieldID jfield = (*env)->GetFieldID( env, jclazz, "fd", "I" );
	int timeout = (int)( (*env)->GetIntField( env, jobj, jfield ) );
	u_int8_t status;
	unsigned int millis;
	
	/* request remote host attention */
	status=(nINIT | STROBE | SELECTIN) & ~(AUTOFEED);
	ioctl(fd, PPIGCTRL, &status );
	usleep(1);
	
	/*  set nSelectin low and nAutoFeed high */
	status=(nINIT | SELECTIN) & ~(STROBE | AUTOFEED);
	ioctl(fd, PPISCTRL, &status );

	/* waiting for peripheral, Xflag ignored */
	ioctl(fd, PPIGSTATUS, &status);
	while ( (millis < timeout ) && !(status & nFAULT) ) 
		{
		usleep(1);
		millis++;
		}
	if (!(status & nFAULT))  
		{ 
		printf("mmm.. failed phase1 \n");
		goto fail;
		 };
	/* set nAutoFd low */
	status=(nINIT | SELECTIN | AUTOFEED) & ~STROBE;
	ioctl(fd, PPIGCTRL, &status );

	/* peripheral set nAck high. Doesn't matter here if it failed or not...
	At least, we tried :-) 		*/
	ioctl(fd, PPIGSTATUS, &status);
	while ( (millis < timeout ) && !(status & nACK) ) 
		{
		usleep(1);
		millis++;
		}
	
	/* end termination, return to idle phase */
	status=(nINIT | SELECTIN) & ~(STROBE | AUTOFEED);
	ioctl(fd, PPISCTRL, &status );
	
fail:
	/* close up File Descriptor connection */
	close( fd );
	return;
}


int write_raw_byte(int fd,JNIEnv *env,jobject jobj, unsigned char byte)
{
	int status=0;
	int millis=0;
	jclass jclazz = (*env)->GetObjectClass( env, jobj );
	jfieldID jfield = (*env)->GetFieldID( env, jclazz, "fd", "I" );
	int timeout = (int)( (*env)->GetIntField( env, jobj, jfield ) );
	
	/* put data on port */
	if (ioctl(fd, PPISDATA, &byte) <0) goto fail;

	/* have to sleep a 1us to let device take a breath */
	usleep(1);
	
	/* wait for host to become ready bfore "STROBEing" */
	ioctl(fd, PPIGSTATUS, &status);
	while ( (millis < timeout ) && !(status & nBUSY) || !(status & nFAULT) ) 
		{
		usleep(1);
		millis++;
		}

	/* check if timed out or OK while leaving the above loop */
	ioctl(fd, PPIGSTATUS, &status);
	if (!(status & nBUSY)) 
		{ 
		/* put TIMEOUT on the line... */
		status &= TIMEOUT;
		ioctl(fd, PPISSTATUS, &status);

		/* Throw "TIMED OUT OPERATION" exception */
		errno=60; 
		goto fail;
		 };
	if (!(status & nFAULT))  
		{ 
		 /* Throw "I/O EXCEPTION" exception */
		errno=5;
		goto fail;
		 };
		 
	ioctl(fd, PPIGCTRL, &byte);
	byte |= STROBE;
	ioctl(fd, PPISCTRL, &byte);
	usleep(1);
	byte &= ~STROBE;
	ioctl(fd, PPISCTRL, &byte);
	return(1);

fail:
	IOException( env, strerror( errno ) );
	return(-1);
}


JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdParallel_writeByte( JNIEnv *env,
	jobject jobj, jint ji ) 
{
	unsigned char byte = (unsigned char)ji;
	int fd = get_java_fd( env, jobj );
	
	if (write_raw_byte(fd,env,jobj,byte)<0)
		{ 
		printf("error in Java_org_freebsd_io_comm_FreebsdParallel_writeByte");
		}
	return;
}


JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdParallel_writeArray( JNIEnv *env,
	jobject jobj, jbyteArray jbarray, jint offset, jint count )
{
	int fd = get_java_fd( env, jobj );
	jbyte *body = (*env)->GetByteArrayElements( env, jbarray, 0 );
	unsigned char *bytes = (unsigned char *)malloc( count );
	int i;
		
	for( i = 0; i < count; i++ ) bytes[ i ] = body[ i + offset ];
	(*env)->ReleaseByteArrayElements( env, jbarray, body, 0 );

	for (i=0;i < count;i++) 
		if (write_raw_byte(fd,env,jobj,bytes[i])<0) goto  fail;
		
	free( bytes );
	return;
fail:
	free(bytes);
	return;
}


int read_byte_array( int fd, unsigned char *buffer, int length, int threshold,
	int timeout )
{
	// return bytes;
}


JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdParallel_readByte( JNIEnv *env,
	jobject jobj )
{ 
	int bytes, fd, timeout;
	unsigned char buffer[ 1 ];
	jfieldID jfield;
	jclass jclazz = (*env)->GetObjectClass( env, jobj );
	jfield = (*env)->GetFieldID( env, jclazz, "fd", "I" );
	fd = (int)( (*env)->GetIntField( env, jobj, jfield ) );
	jfield = (*env)->GetFieldID( env, jclazz, "timeout", "I" );
	timeout = (int)( (*env)->GetIntField( env, jobj, jfield ) );

	bytes = read_byte_array( fd, buffer, 1, 1, timeout );
	if( bytes < 0 ) {
		IOException( env, strerror( errno ) );
		return -1;
	}
	return (bytes ? (jint)buffer[ 0 ] : -1);

}


JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdParallel_readArray( JNIEnv *env,
	jobject jobj, jbyteArray jbarray, jint offset, jint length )
{  
	int bytes, i, fd, threshold, timeout;
	jbyte *body;
	unsigned char *buffer;
	jfieldID jfield;
	jclass jclazz = (*env)->GetObjectClass( env, jobj );
	jfield = (*env)->GetFieldID( env, jclazz, "fd", "I" );
	fd = (int)( (*env)->GetIntField( env, jobj, jfield ) );
	jfield = (*env)->GetFieldID( env, jclazz, "threshold", "I" );
	threshold = (int)( (*env)->GetIntField( env, jobj, jfield ) );
	jfield = (*env)->GetFieldID( env, jclazz, "timeout", "I" );
	timeout = (int)( (*env)->GetIntField( env, jobj, jfield ) );

	if( length < 1 || length > SSIZE_MAX ) {
		IOException( env, "Invalid length" );
		return -1;
	}

	buffer = (unsigned char *)malloc( sizeof( unsigned char ) * length );
	if( buffer == 0 ) {
		IOException( env, "Unable to allocate buffer" );
		return -1;
	}

	bytes = read_byte_array( fd, buffer, length, threshold, timeout );
	if( bytes < 0 ) {
		free( buffer );
		IOException( env, strerror( errno ) );
		return -1;
	}

	body = (*env)->GetByteArrayElements( env, jbarray, 0 );
	for( i = 0; i < bytes; i++ ) body[ i + offset ] = buffer[ i ];
	(*env)->ReleaseByteArrayElements( env, jbarray, body, 0 );
	free( buffer );
	return (bytes ? bytes : -1);
}


JNIEXPORT jint JNICALL Java_org_freebsd_io_comm_FreebsdParallel_nativeavailable( JNIEnv *env,
	jobject jobj )
{
	int fd = get_java_fd( env, jobj );
	int result;
	result=1; // right ?
	return (jint)result;
}


JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdParallel_setHWFC( JNIEnv *env,
	jobject jobj, jboolean state )
{
	int fd = get_java_fd( env, jobj );
	return;
}


JNIEXPORT void JNICALL Java_org_freebsd_io_comm_FreebsdParallel_eventLoop( JNIEnv *env,
	jobject jobj )
{
	int fd, ret, change;
	unsigned int mflags;
	fd_set rfds;
	struct timeval sleep;
	jfieldID jfield;
	jmethodID method, interrupt;
	jboolean interrupted = 0;
	jclass jclazz, jthread;
	jclazz = (*env)->GetObjectClass( env, jobj );
	jfield = (*env)->GetFieldID( env, jclazz, "fd", "I" );
	fd = (int)( (*env)->GetIntField( env, jobj, jfield ) );
	method = (*env)->GetMethodID( env, jclazz, "sendEvent", "(IZ)V" );
	jthread = (*env)->FindClass( env, "java/lang/Thread" );
	interrupt = (*env)->GetStaticMethodID( env, jthread, "interrupted", "()Z" );

	FD_ZERO( &rfds );
	while( !interrupted ) {
		FD_SET( fd, &rfds );
		sleep.tv_sec = 1;	/* Check every 1 second, or on receive data */
		sleep.tv_usec = 0;
		ret = select( fd + 1, &rfds, NULL, NULL, &sleep );
		if( ret < 0 ) break;
	/*	if( ioctl( fd, TIOCGICOUNT, &sis ) ) break;
		if( ioctl( fd, TIOCMGET, &mflags ) ) break;
		serial sepecific
*/
		interrupted = (*env)->CallStaticBooleanMethod( env, jthread, interrupt );
	}
	return;
}


void send_printer_events( JNIEnv *env, jobject jobj, jmethodID method,
	int event, int change, int state )
{
	int i, s;
	jboolean flag;
	if( state ) s = 1;
	else s = 0;

	for( i = 0; i < change; i++ ) {
		if( ( change + s + i ) % 2 ) flag = JNI_FALSE;
		else flag = JNI_TRUE;
		(*env)->CallVoidMethod( env, jobj, method, (jint)event, flag );
	}
}


int get_java_fd( JNIEnv *env, jobject jobj )
{
	jclass jclazz = (*env)->GetObjectClass( env, jobj );
	jfieldID jfd = (*env)->GetFieldID( env, jclazz, "fd", "I" );
	if( !jfd ) {
		(*env)->ExceptionDescribe( env );
		(*env)->ExceptionClear( env );
		return 0;
	}
	return (int)( (*env)->GetIntField( env, jobj, jfd ) );
}


void IOException( JNIEnv *env, char *msg )
{
	jclass clazz = (*env)->FindClass( env, "java/io/IOException" );
	if( clazz == 0 ) return;
	(*env)->ThrowNew( env, clazz, msg );
}


void UnsupportedCommOperationException( JNIEnv *env, char *msg )
{
	jclass clazz = (*env)->FindClass( env,
		"javax/comm/UnsupportedCommOperationException" );
	if( clazz == 0 ) return;
	(*env)->ThrowNew( env, clazz, msg );
}
