#!/bin/bash
# Script that converts all markdown files in a folder to pdf using the markdown-pdf tool
# Link to tool: https://github.com/alanshaw/markdown-pdf
# Install tool with: sudo npm install -g markdown-pdf

# Convert all markdown files in a folder to pdf files. Does not work properly, and since there are not a lot of files, it was easier to just type all the filenames that needed conversion. Implementation left here for possible future extensions
convert_folder () { 
for FILE_PATH in "$PWD"/*
do 
 FILE_PATH=`printf "%q\n" "$FILE_PATH"`
 FILE="${FILE_PATH##*/}" # Full file name
 EXT="${FILE##*.}" # File extension
 if [ $EXT == 'md' ]
  then markdown-pdf $FILE_PATH
 fi
done 
}

cd ../wiki

markdown-pdf Getting-started.md -o pdf/Getting-started.pdf
markdown-pdf Turtlebot_GT.md -o pdf/Turtlebot_GT.pdf
markdown-pdf FAQ.md -o pdf/FAQ.pdf
pdftk pdf/Getting-started.pdf pdf/Turtlebot_GT.pdf pdf/FAQ.pdf cat output pdf/Documentation.pdf

cd Mobile\ robotics/
markdown-pdf MR.md -o ../pdf/MR.pdf
markdown-pdf Session_1.md -o ../pdf/Session_1.pdf
markdown-pdf Session_2.md -o ../pdf/Session_2.pdf
markdown-pdf Session_3.md -o ../pdf/Session_3.pdf
pdftk ../pdf/MR.pdf ../pdf/Session_1.pdf ../pdf/Session_2.pdf ../pdf/Session_3.pdf cat output ../pdf/Mobile_robotics.pdf

cd ../../common/
