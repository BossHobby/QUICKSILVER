#! /bin/bash

for target in $(find src/targets/* -type d); do
  make clean &> /dev/null

  if make -j32 $(basename $target) &> /dev/null; then 
    echo -e "\e[32mSuccessfully\e[39m built target $target"
  else
    echo -e "\e[31mError\e[39m building target $target"
  fi
done