#!/usr/bin/bash

set -e

REBUILD_LIMIT=5

MY_PATH=$(dirname $0)
MY_PATH=$(cd $MY_PATH && pwd)
if [ -z $MY_PATH ] ; then
  printf "\nPath error: ${MY_PATH}\n"
  exit 1
fi

cd $MY_PATH/../latex/texfiles

files_to_remove=$(ls | { grep -E '.toc|.aux|.out|.log|.pdf' || test $? = 1; } || "")
if [[ $files_to_remove != "" ]]; then
  printf "\nDeleting files: $(echo ${files_to_remove})\n"
  rm $files_to_remove 
fi

counter=1
output=$(pdflatex main.tex)
printf "\nBuild counter: ${counter}\n\n${output}\n\n"
while [[ $output == *"(rerunfilecheck)"* && $counter -lt $REBUILD_LIMIT ]]; do
  counter=$((counter + 1))
  output=$(pdflatex main.tex)
  printf "\nBuild counter: ${counter}\n\n${output}\n\n"
done

if [[ $counter -eq $REBUILD_LIMIT ]]; then
  printf "Limited rebuilds to %d, PDF probably has errors.\n" $counter
  exit 1
else
  printf "It took %d runs to complete the PDF.\n" $counter
fi

cd - > /dev/null
exit 0
