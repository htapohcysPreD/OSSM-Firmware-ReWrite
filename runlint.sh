#!/bin/sh

set -e

cpplint --recursive --linelength=240 --headers=h --extensions=c --filter=-runtime/int,-readability/casting ./main/
cpplint --recursive --linelength=240 --headers=h --extensions=c --filter=-runtime/int,-readability/casting,-readability/fn_size ./components/

echo "finished analysis"
