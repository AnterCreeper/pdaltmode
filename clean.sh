#!/bin/bash
rm ./obj/pdaltmode.elf
rm ./obj/pdaltmode.hex
rm ./obj/pdaltmode.lst
rm ./obj/pdaltmode.map
find . -name "*.o"  | xargs rm
