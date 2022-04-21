# File      :     autobot-code-header-generator.sh
# Author    :     Luyao Han
#	Email     :     luyaohan1001@gmail.com 
# Brief     :     This bot script automatically generate header file for codes.
# Date      :     04-08-2022
# Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal / non-commercial use.


# Check if file name is specified
if [[ ! $1 ]]; then
	echo 'Usage: ./autobot-code-head-generator.sh <file-name>'
	echo '- autobot exiting... -'
	exit 1
fi

# Check if header already exist in that file.
if [[ `grep 'Author' $1` ]]; then
	echo 'Code header already exists.'
	echo '- autobot exiting... -'
	exit 1
fi

# Add the header to the file
file=$1
author='Luyao Han'
email='luyaohan1001@gmail.com'

# -n will get rid of new line native to echo command.
echo "Please enter brief description to this file."
echo -n "> "
read brief

# Get date
date=`date +%m-%d-%Y`
copyright_message='Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.'

# Construct the header paragraph according to file type. ( .sh / .c / .cpp )
if [[ `echo $1 | grep .h$` ]]; then  # check if file has .h as file extension. '$' indicates that .h must be the last character as file name.
	header+="/**\n"
	header+="  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********\n"
	header+="  * @file      :     $file\n"
	header+="  * @author    :     $author\n"
	header+="  * @email     :     $email\n"
	header+="  * @brief     :     $brief\n"
	header+="  * @date      :     $date\n"
	header+="  * $copyright_message\n"
	header+="  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********"
	header+="  */\n"
	header+="\n"
elif [[ `echo $1 | grep .c$` ]]; then  # check if file has .h as file extension. '$' indicates that .h must be the last character as file name.
	header+="/**\n"
	header+="  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********\n"
	header+="  * @file      :     $file\n"
	header+="  * @author    :     $author\n"
	header+="  * @email     :     $email\n"
	header+="  * @brief     :     $brief\n"
	header+="  * @date      :     $date\n"
	header+="  * $copyright_message\n"
	header+="  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********"
	header+="  */\n"
	header+="\n"
elif [[ `echo $1 | grep .sh$` ]]; then # check if the input file is a bash .sh file.
	header+="# file      :     $file\n"
	header+="# author    :     $author\n"
	header+="# email     :     $email\n"
	header+="# brief     :     $brief\n"
	header+="# date      :     $date\n"
	header+="# $copyright_message\n"
	header+="# -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- --------\n"
	header+="\n"

else 
	echo "- The file $file is not supported format by this script. -"
	echo "- No header has been added to $file. Exiting... -"
	exit 1
fi

# use forward slash '/' with paragraph seems to cause problem. Use '|' instead works as well.
sed -i "1s|^|$header|" $1

echo "- code header has been generated by autobot. -"
echo "- exiting successfully... -"
exit 0



