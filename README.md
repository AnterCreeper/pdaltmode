# USB PD Alt-mode Negotiation

** WARNING! This Project is not been fully tested, use at your own risk! **   
** 警告！该项目未经充分的测试，使用需要你自己衡量！**   
This Project is used to perform PD Logic for Host based on the cost-effective (30 cents at the time of writing) WCH CH32X035 32-bit RISC-V microcontroller, to achieve DisplayPort output over Type-C, which can be used on space compact scene.    
该项目基于来自WCH的低成本CH32X035微控制器，包含了所需的所有主机端的 USB Power Delivery 协议的功能，以实现Type-C上DisplayPort视频输出功能，可用于PCB面积紧凑场景。   

Recommended:    
1. TOSHIBA TC358867XBG， MIPI DPI(RGB Parallel) to DisplayPort 2-lane Bridge IC    
2. Newcosemi NCS8801S, RGB Parallel to DisplayPort 4-lane Bridge IC, with integrated lane multiplex    
3. WCH CH482D DPDT HighSpeed MUX    
4. DIODES PI3USB302 DPDT HighSpeed MUX    
5. TI TS3A5223 DPDT Analog Mux for SBU Mux    

### Overview

This Project implements state machines defined in:
1. Universal Serial Bus Power Delivery Specification Rev 2.0 Ver 1.3
2. Universal Serial Bus Type-C Cable and Connector Specification Ver 2.2.
3. VESA DisplayPort Alt Mode on USB Type-C Standard. Ver 1.3/1.4.

#### Known Issues

T.B.D

#### License

This project is licensed under the LGPL-2.1-or-later license. For more details, see the `COPYING` file.
