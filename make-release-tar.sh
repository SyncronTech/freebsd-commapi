#!/bin/sh

	REL=$1
	RELCVS=`echo $REL | tr . _` 
	mkdir release
	cd release
	cvs export -rRELENG_${RELCVS}_RELEASE -d freebsd-commapi-$REL FreeBSD/commapi

	tar cvzf freebsd-commapi-$REL.tar.gz freebsd-commapi-$REL
