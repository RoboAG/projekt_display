#!/bin/bash

### travis file for building all software modules



echo ""
echo "### Sourcing robolib"
. ../../robolib/scripts/bashrc.sh



echo ""
echo "### Building"
cd ../Software/
PATH_SOFTWARE="$(pwd)/"
PATH_DEVEL="${PATH_SOFTWARE}devel/"



echo ""
echo "## Peter"
PATH_PETER="${PATH_DEVEL}Peter/"

echo "# test"
cd ${PATH_PETER}test/
#robolib_all
echo "no build, since uart for atmega8 is not supported yet"
if [ $? -ne 0 ]; then exit -1; fi



echo ""
echo "## Main Peter"
PATH_MAIN_PETER="${PATH_SOFTWARE}Peter/"

echo "# simple_text"
cd ${PATH_MAIN_PETER}simple_text/
robolib_all
if [ $? -ne 0 ]; then exit -1; fi
