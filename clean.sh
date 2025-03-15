#!/bin/bash
rm -r ./EVB/pdaltmode-backups
rm ./EVB/fp-info-cache
rm ./EVB/pdaldmode.kicad_prl
rm ./obj/pdaltmode.elf
rm ./obj/pdaltmode.hex
rm ./obj/pdaltmode.lst
rm ./obj/pdaltmode.map
find . -name "*.o"  | xargs rm
