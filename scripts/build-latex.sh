#!/usr/bin/bash

set -e

rebuild_limit=5

my_path=$(dirname $0)
my_path=$(cd $my_path && pwd)
if [ -z $my_path ] ; then
  printf "\nPath error: ${my_path}\n"
  exit 1
fi

cd $my_path/../latex/texfiles

files_to_remove=$(ls | { grep -E '.toc|.aux|.out|.log|.pdf' || test $? = 1; } || "")
if [[ $files_to_remove != "" ]]; then
  printf "\nDeleting files: $(echo ${files_to_remove})\n"
  rm $files_to_remove 
fi

counter=1
output=$(pdflatex main.tex)
printf "\nBuild counter: ${counter}\n\n${output}\n\n"
while [[ $output == *"(rerunfilecheck)"* && $counter -lt $rebuild_limit ]]; do
  counter=$((counter + 1))
  output=$(pdflatex main.tex)
  printf "\nBuild counter: ${counter}\n\n${output}\n\n"
done

if [[ $counter -eq $rebuild_limit ]]; then
  printf "Limited rebuilds to %d, PDF probably has errors.\n" $counter
  exit 1
else
  printf "It took %d runs to complete the PDF.\n" $counter
fi

exit 0
