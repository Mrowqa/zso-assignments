#!/bin/zsh

for f in *-64; do
    echo ----------- $f
    LD_LIBRARY_PATH=.: ./$f
    if [ $? -ne 0 ]; then
        echo "FAIL"
    fi
done

