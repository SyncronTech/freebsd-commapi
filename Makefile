JAVA_PKG_DIR	=	org/freebsd/io/comm

CLASSES		= 	classes/$(JAVA_PKG_DIR)/FreebsdDriver.class \
			classes/$(JAVA_PKG_DIR)/FreebsdSerial.class \
			classes/$(JAVA_PKG_DIR)/FreebsdParallel.class

JAVASRC		= 	src/$(JAVA_PKG_DIR)/FreebsdDriver.java \
			src/$(JAVA_PKG_DIR)/FreebsdSerial.java \
			src/$(JAVA_PKG_DIR)/FreebsdParallel.java

JAVAHFILES	=	src/$(JAVA_PKG_DIR)/freebsd_io_comm_FreebsdParallel.h \
			src/$(JAVA_PKG_DIR)/freebsd_io_comm_FreebsdSerial.h

LIBS		=	lib/libSerial.so \
			lib/libParallel.so

JAVA_HOME	?= 	/usr/local/jdk1.3.1
JAVAC		=	$(JAVA_HOME)/bin/javac
JAVAH		=	$(JAVA_HOME)/bin/javah
JARFILE		=	jar/CommDriver.jar
CFLAGS		= 	-O2 -shared -I /usr/java/include -I /usr/java/include/freebsd -L /usr/java/lib/i386/green_threads/

#
# Build jar and libs
#
all: $(JARS) $(LIBS)

# 
# Clean all files produced by compile
#
clean:
	rm -rf jar/*
	rm -rf classes/*
	rm -f lib/*
	rm -f $(JAVAHFILES)
# 
# Actual jar file build
#
$(JARFILE):		$(CLASSES)
	cd classes; \
	jar -cvf ../$(JARFILE) $(JAVA_PKG_DIR)

#
# Java compilation
#
$(CLASSES):	$(JAVASRC)
	$(JAVAC) -d classes $(JAVASRC)

src/$(JAVA_PKG_DIR)/freebsd_io_comm_FreebsdParallel.h:	$(CLASSES)
	cd classes; \
	$(JAVAH) -jni org.freebsd.io.comm.FreebsdParallel

src/$(JAVA_PKG_DIR)/freebsd_io_comm_FreebsdSerial.h:	$(CLASSES)
	cd classes; \
	$(JAVAH) -jni org.freebsd.io.comm.FreebsdSerial

#
# Parallel driver JNI part
#
lib/libParallel.so:	src/$(JAVA_PKG_DIR)/libParallel.c \
			src/$(JAVA_PKG_DIR)/freebsd_io_comm_FreebsdParallel.h
	gcc $(CFLAGS) -o lib/libParallel.so src/$(JAVA_PKG_DIR)/libParallel.c 
#
# Serial driver JNI part
#
lib/libSerial.so:	src/$(JAVA_PKG_DIR)/libSerial.c \
			src/$(JAVA_PKG_DIR)/freebsd_io_comm_FreebsdSerial.h
	gcc $(CFLAGS) -o lib/libSerial.so src/$(JAVA_PKG_DIR)/libSerial.c 

#
#JAVAC=javac
#OBJDIR= obj
#JARFILE=jar/CommDriver.jar
#JFLAGS=
#LIBDIR=/usr/local/lib
#CFLAGS= -O2 -shared -I /usr/java/include -I /usr/java/include/freebsd -L /usr/java/lib/i386/green_threads/
#obj/%.class: src/%.java
#	cd src;\
#	$(JAVAC) ../$< $(JFLAGS) -d ../obj
#
#all: $(OBJ) jar libs
#
#$(JARFILE): $(CLASSES)
#	cd classes; \
#	jar -cvf0 ../$(JARFILE) org/freebsd/io/comm/ 
#	cp jar/* tests/sun/
#	cp jar/* tests/java/
#
#jar: $(JARFILE)
#
#clean:
#	rm -f src/org/freebsd/io/comm*~ *~ 
#	rm -f src/org/freebsd/io/comm/*~
#	rm -rf jar/*.jar
#	rm -rf obj/*
#	rm -f doc/*
#	rm -f lib/*
#
#mrproper: clean
#	rm -rf obj/*
#	rm -rf jar/*
#
#doc/tree.html:
#	cd src; \
#	javadoc -package -version -author -private -d ../doc/ org.freebsd.io.comm
#
#doc: $(SRC) doc/tree.html
#
#src/org/freebsd/io/comm/org_freebsd_io_comm_FreebsdParallel.h: src/org/freebsd/io/comm/libParallel.c
#	cd obj; \
#	javah -jni -d ../src/org/freebsd/io/comm/ org.freebsd.io.comm.FreebsdParallel
#
##src/org/freebsd/io/comm/org_freebsd_io_comm_FreebsdSerial.h: src/org/freebsd/io/comm/libSerial.c
#	cd obj; \
#	javah -jni -d ../src/org/freebsd/io/comm/ org.freebsd.io.comm.FreebsdSerial
#
#lib/libParallel.so:  src/org/freebsd/io/comm/libParallel.c src/org/freebsd/io/comm/org_freebsd_io_comm_FreebsdParallel.h
#	gcc $(CFLAGS) -o lib/libParallel.so src/org/freebsd/io/comm/libParallel.c 
#
#lib/libSerial.so:  src/org/freebsd/io/comm/libSerial.c src/org/freebsd/io/comm/org_freebsd_io_comm_FreebsdSerial.h
#	gcc $(CFLAGS) -o lib/libSerial.so src/org/freebsd/io/comm/libSerial.c 
#
#
#libs: lib/libParallel.so lib/libSerial.so
#
#install: libs
#	install -c -o bin -g bin -m 0444 lib/libParallel.so $(LIBDIR)
#	install -c -o bin -g bin -m 0444 lib/libSerial.so $(LIBDIR)
#####
