#ifndef __MPD_H
#define __MPD_H

#define MPD_DEV_ADR     0x68    //7b1101000

void MPD_Init();
int MPD_CfgLink();
void MPD_CfgVideo();
void MPD_CfgTest();

#endif