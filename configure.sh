#!/bin/bash
buildDir=`pwd`
srcDir=$buildDir/`dirname $0`
SRCFILES=$srcDir/src/GeneratedFiles/*

MAKEFILE=makefile
echo "VPATH=$srcDir/src/GeneratedFiles/" > $MAKEFILE
cat $srcDir/header.txt >> $MAKEFILE

bins="BINS="
clean_files="CLEAN_FILES="

for f in $SRCFILES
do
  y=${f%.*}
  FILENAME=${y##*/}
  echo -e "# $FILENAME" >> $MAKEFILE
  echo -e "${FILENAME}_SRCS=../src/GeneratedFiles/${FILENAME}.cc" >> $MAKEFILE
  echo -e "${FILENAME}_OBJS=${FILENAME}.o" >> $MAKEFILE
  echo -e "${FILENAME}_BIN=../bin/${FILENAME}" >> $MAKEFILE
  echo -e "${FILENAME}_DEPS=.${FILENAME}.deps\n" >> $MAKEFILE
  
  echo -e "${FILENAME}_CLEAN= \${${FILENAME}_OBJS} \${${FILENAME}_BIN} \${${FILENAME}_DEPS}" >> $MAKEFILE
  
  echo -e "\$(${FILENAME}_BIN): \$(${FILENAME}_OBJS)" >> $MAKEFILE
	echo -e "\t\$(CXX) \$< \$(LDFLAGS) -o \$@" >> $MAKEFILE
  echo -e "-include \$(${FILENAME}_DEPS)" >> $MAKEFILE
  echo -e '###\n' >> $MAKEFILE

  bins="$bins \${${FILENAME}_BIN}"
  clean_files="$clean_files \${${FILENAME}_CLEAN}"
done

echo $bins >> $MAKEFILE

echo $clean_files >> $MAKEFILE

cat $srcDir/footer.txt >> $MAKEFILE
