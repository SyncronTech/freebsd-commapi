JAVA_PKG_DIR	=	org/freebsd/io/comm

CLASSES		= 	src/$(JAVA_PKG_DIR)/FreebsdSerial.class \
			src/$(JAVA_PKG_DIR)/FreebsdParallel.class \
			src/$(JAVA_PKG_DIR)/FreebsdDriver.class

JAVASRC		= 	src/$(JAVA_PKG_DIR)/FreebsdSerial.java \
			src/$(JAVA_PKG_DIR)/FreebsdParallel.java \
			src/$(JAVA_PKG_DIR)/FreebsdDriver.java

JAVAHFILES	=	src/$(JAVA_PKG_DIR)/org_freebsd_io_comm_FreebsdParallel.h \
			src/$(JAVA_PKG_DIR)/org_freebsd_io_comm_FreebsdSerial.h \
			src/$(JAVA_PKG_DIR)/javax_comm_SerialPort.h \
			src/$(JAVA_PKG_DIR)/javax_comm_SerialPortEvent.h \
			src/$(JAVA_PKG_DIR)/javax_comm_ParallelPort.h \
			src/$(JAVA_PKG_DIR)/javax_comm_ParallelPortEvent.h

LIBS		=	lib/libSerial.so \
			lib/libParallel.so

JAVA_HOME	?= 	/usr/local/jdk1.3.1
JAVAC		=	$(JAVA_HOME)/bin/javac
JAR		=	$(JAVA_HOME)/bin/jar
JAVAC_CLASSPATH	=	$(JAVA_HOME)/jre/lib/ext/comm.jar
JAVAH		=	$(JAVA_HOME)/bin/javah
JARFILE		=	jar/CommDriver.jar
CFLAGS		= 	-O1 -shared -I$(JAVA_HOME)/include -I$(JAVA_HOME)/include/freebsd -I$(JAVA_HOME)/include/bsd

.SUFFIXES:	.java .class

#
# Build jar and libs
#
all: $(JARFILE) $(LIBS)

#
# Install stuff
#
install: all
	install -c -o bin -g bin -m 0444 lib/libParallel.so $(JAVA_HOME)/jre/lib/i386
	install -c -o bin -g bin -m 0444 lib/libSerial.so $(JAVA_HOME)/jre/lib/i386
	install -c -o bin -g bin -m 0444 javax.comm.properties $(JAVA_HOME)/jre/lib
	install -c -o bin -g bin -m 0444 $(JARFILE) $(JAVA_HOME)/jre/lib/ext

# 
# Clean all files produced by compile
#
clean:
	rm -rf jar
	rm -f src/$(JAVA_PKG_DIR)/*.class
	rm -rf lib
	rm -f $(JAVAHFILES)
# 
# Actual jar file build
#
$(JARFILE):		$(CLASSES)
	if [ ! -d jar ]; then mkdir jar; fi
	cd src; \
	$(JAR) -cvf ../$(JARFILE) $(JAVA_PKG_DIR)/*.class

#
# Java compilation
#
$(CLASSES):	$(JAVASRC)

src/$(JAVA_PKG_DIR)/org_freebsd_io_comm_FreebsdParallel.h:	$(CLASSES)
	cd src/$(JAVA_PKG_DIR); \
	$(JAVAH) -jni -classpath ../../../..:$(JAVAC_CLASSPATH) org.freebsd.io.comm.FreebsdParallel

src/$(JAVA_PKG_DIR)/org_freebsd_io_comm_FreebsdSerial.h:	$(CLASSES)
	cd src/$(JAVA_PKG_DIR); \
	$(JAVAH) -jni -classpath ../../../..:$(JAVAC_CLASSPATH) org.freebsd.io.comm.FreebsdSerial

src/$(JAVA_PKG_DIR)/javax_comm_SerialPort.h:	$(JAVA_HOME)/jre/lib/ext/comm.jar
	cd src/$(JAVA_PKG_DIR); \
	$(JAVAH) -jni -classpath $(JAVAC_CLASSPATH) javax.comm.SerialPort

src/$(JAVA_PKG_DIR)/javax_comm_SerialPortEvent.h:	$(JAVA_HOME)/jre/lib/ext/comm.jar
	cd src/$(JAVA_PKG_DIR); \
	$(JAVAH) -jni -classpath $(JAVAC_CLASSPATH) javax.comm.SerialPortEvent

src/$(JAVA_PKG_DIR)/javax_comm_ParallelPort.h:	$(JAVA_HOME)/jre/lib/ext/comm.jar
	cd src/$(JAVA_PKG_DIR); \
	$(JAVAH) -jni -classpath $(JAVAC_CLASSPATH) javax.comm.ParallelPort

src/$(JAVA_PKG_DIR)/javax_comm_ParallelPortEvent.h:	$(JAVA_HOME)/jre/lib/ext/comm.jar
	cd src/$(JAVA_PKG_DIR); \
	$(JAVAH) -jni -classpath $(JAVAC_CLASSPATH) javax.comm.ParallelPortEvent

#
# Parallel driver JNI part
#
lib/libParallel.so:	src/$(JAVA_PKG_DIR)/libParallel.c \
			src/$(JAVA_PKG_DIR)/org_freebsd_io_comm_FreebsdParallel.h \
			src/$(JAVA_PKG_DIR)/javax_comm_ParallelPort.h \
			src/$(JAVA_PKG_DIR)/javax_comm_ParallelPortEvent.h
	if [ ! -d lib ]; then mkdir lib; fi
	gcc $(CFLAGS) -o lib/libParallel.so src/$(JAVA_PKG_DIR)/libParallel.c 
#
# Serial driver JNI part
#
lib/libSerial.so:	src/$(JAVA_PKG_DIR)/libSerial.c \
			src/$(JAVA_PKG_DIR)/org_freebsd_io_comm_FreebsdSerial.h \
			src/$(JAVA_PKG_DIR)/javax_comm_SerialPort.h \
			src/$(JAVA_PKG_DIR)/javax_comm_SerialPortEvent.h
	if [ ! -d lib ]; then mkdir lib; fi
	gcc $(CFLAGS) -o lib/libSerial.so src/$(JAVA_PKG_DIR)/libSerial.c 

.java.class:
	$(JAVAC) -classpath src:$(JAVAC_CLASSPATH) $*.java
	
