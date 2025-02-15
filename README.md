# USB PD Alt-mode Negotiation

** WARNING! This Project is not been fully tested, use at your own risk! **   
** 警告！该项目未经充分的测试，使用需要你自己衡量！**

If any questions, welcome for Issues & PR   
如果有疑问，欢迎提交Issues或PR

#### TODO

1. Active Cable Support
2. Validation Hardware Implementation

1. 主动线缆支持
2. 验证硬件实现

### Overview

This Project is used to perform PD Logic for Host based on the cost-effective (30 cents at the time of writing) WCH CH32X035 32-bit RISC-V microcontroller, to achieve DisplayPort output over Type-C, which can be used on space compact scene.    
该项目基于来自WCH的低成本CH32X035微控制器，包含了所需的所有主机端的 USB Power Delivery 协议的功能，以实现Type-C上DisplayPort视频输出功能，可用于PCB面积紧凑场景。

This Project implements state machines defined in:
1. Universal Serial Bus Power Delivery Specification Rev 2.0 Ver 1.3
2. Universal Serial Bus Type-C Cable and Connector Specification Ver 2.2.
3. VESA DisplayPort Alt Mode on USB Type-C Standard. Ver 1.3/1.4.

Recommended:    
1. TOSHIBA TC358867XBG， MIPI DSI or DPI(RGB Parallel) to DisplayPort 2-lane Bridge IC
2. WCH CH482D DPDT HighSpeed MUX for DP Signal
3. WCH CH442E DPDT Analog Mux for SBU

### Usage

1. Download and Open with MounRiver Studio II
2. Compile and Run

1. 下载并使用MounRiver Studio II打开
2. 编译运行

#### License

This project is licensed under the LGPL-2.1-or-later license.
