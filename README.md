# USB PD Alt-mode Negotiation

If any questions, welcome for Issues & PRs   
如果有疑问，欢迎提交Issues或PR

**WARNING! This Project is not been fully productive tested, use at your own risk!**   
**警告！该项目未经充分的生产测试，使用需要你自己衡量**

### License

This project is licensed under the LGPL-2.1-or-later license. **DO NOT** download or clone this project until you have read and agree the LICENSE.     
该项目采用 `LGPL-2.1 以及之后版本` 授权。当你下载或克隆项目时，默认已经阅读并同意该协定。

### TODO

1. Active Cable Support (not in plan)
2. Audio Transmission Function Testing

### Overview

This Project is used to perform PD Logic for Host based on the cost-effective (30 cents at the time of writing) WCH CH32X035 32-bit RISC-V microcontroller, to achieve DisplayPort output over Type-C, which can be used on space compact scene.    
该项目基于来自 WCH 的低成本 CH32X035 微控制器，包含了所需的所有主机端的 USB Power Delivery 协议的功能，以实现 Type-C 上 DisplayPort 视频输出功能，可用于 PCB 面积紧凑场景。

Besides, This Project include a validation hardware with GERBER and schematics, 4 layers JLC04161H-3313 Stacking, using TOSHIBA TC358867XBG to convert Video stream from Parrallel to DisplayPort Signal.   
此外，该项目附有一个验证硬件，包含 GERBER 和原理图与装配图纸，四层 1.6mm JLC04161H-3313 叠层，采用 TOSHIBA TC358867XBG 将 RGB 并行接口 转换至 DisplayPort 信号输出。

![PCB Preview Picture](https://github.com/AnterCreeper/pdaltmode/blob/main/evb-front.jpg?raw=true)

This Project implements state machines defined in:
1. Universal Serial Bus Power Delivery Specification Rev 2.0 Ver 1.3
2. Universal Serial Bus Type-C Cable and Connector Specification Ver 2.2.
3. VESA DisplayPort Alt Mode on USB Type-C Standard. Ver 1.3/1.4.
4. VESA DisplayPort Standard. Version 1, Revision 2.

Doc. 1 and 2 can be accessed from https://github.com/usb-c/USB-Type-C   
For Doc. 3, since **DISTRIBUTION TO NON-MEMBERS IS PROHIBITED**, contact me at [wangzhihao9@hotmail.com](mailto:wangzhihao9@hotmail.com)  
Doc.4 is located at https://glenwing.github.io/docs/DP-1.2.pdf

Datasheet of TC358867XBG is included in the project, which includes external LICENSE.

Some other Specification needs:
1. CEA 861-E: A DTV Profile for Uncompressed High Speed Digital Interfaces
2. IEC60958 Digital Audio Interfaces

### Usage

1. Download and Open with MounRiver Studio II [Download Page](http://www.mounriver.com/download)
2. Connect the board with WCH Link-E, then Compile and Run
3. ~~Files except in User is remain unchanged from default.~~
