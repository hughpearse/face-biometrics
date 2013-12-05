#!/bin/bash
#Segment leaf from uniform background color

metrics=`./flandmark_1 $1 | cut -d',' -f -10`;
file=`java -jar search.jar $metrics /home/hughpear/Desktop/biometrics/face-biometrics/10-metric-DB-faces-cleaned.arff`;
echo "$file";
eog $file;

