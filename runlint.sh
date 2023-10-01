#!/bin/sh

# Enabled Filters:
#    legal/copyright    - No warnings for missing copyright
#    readability/todo   - No warnings for TODO Style
#    build/include_subdir   - No warnings about include paths

cpplint --recursive --linelength=240 --headers=h --extensions=c,cpp \
    --filter=-legal/copyright,-readability/todo,-build/include_subdir \
    ./main/

cpplint --recursive --linelength=240 --headers=h --extensions=c,cpp \
    --filter=-legal/copyright,-readability/todo,-build/include_subdir \
    --exclude=./components/u8g2/* \
    ./components/

echo "finished analysis"
