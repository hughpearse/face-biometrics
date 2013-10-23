#!/bin/bash
#Segment leaf from uniform background color
for i in `ls -1 ./face-dataset/`;
do
echo $i;
metrics=`./flandmark_1 ./face-dataset/$i`
echo "$metrics,$i" >> ./10-metric-DB-faces.arff
done

