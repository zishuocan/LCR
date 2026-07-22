# LCR项目硬件接线还原清单

更新时间：2026-07-20  
适用硬件：NUCLEO-G474RE + LCR_GROUP31模拟板 + 0.96英寸O96W4-small四针OLED

本文是外部接线的唯一完整还原清单。只看本文即可从完全拆线状态恢复当前硬件，不需要从开发过程日志猜测。

## 1. 接插件方向和针号

- LCR板按PCB元件/丝印面朝上、`LCR_GROUP31`文字正向放置。
- J5位于板上边缘，方形焊盘为1脚；从左到右依次为1～7脚。
- J7位于J5左侧，方形焊盘为1脚；从上到下依次为1～4脚。J7-1、J7-2均为`+5V`，J7-3、J7-4均为`GND`。
- J6位于板下边缘，方形1脚在左侧；从左到右依次为1～4脚。
- J1、J2、J3均为同轴接口封装：1脚/中心信号分别是`V_AMP`、`I_AMP`、`SINE`；2脚/外壳焊盘为`GND`。
- NUCLEO端优先按Arduino丝印`A0～A2`、`D4～D10`、`D14～D15`找针；下表同时给出MCU端口和连接器针号，避免只凭丝印方向判断。

## 2. NUCLEO与LCR板：模拟信号

| NUCLEO-G474RE端 | 方向 | LCR板端 | 说明 |
|---|---|---|---|
| CN8-3，A2 / PA4 | → | J3-1，SINE | DAC激励输出 |
| CN8-1，A0 / PA0 | ← | J1-1，V_AMP | 电压采样，ADC1 |
| CN8-2，A1 / PA1 | ← | J2-1，I_AMP | 电流采样，ADC2 |
| 任一GND | ↔ | J1/J2/J3的2脚及LCR板GND | 两板必须共地；使用同轴线时外屏蔽接2脚/GND |

注意：J2是`I_AMP`采样输出，不是正弦输出。需要看原始DAC正弦时测NUCLEO A2/PA4；LCR板正弦输入点是J3-1。

## 3. NUCLEO与LCR板：量程/PGA控制线

| NUCLEO Arduino针 | MCU端口 | LCR板J5 | 信号 |
|---|---|---|---|
| D6（CN9-7） | PB10 | J5-1 | MUX_A0 |
| D7（CN9-8） | PA8 | J5-2 | MUX_A1 |
| D8（CN5-1） | PA9 | J5-3 | GAIN_A2 |
| D9（CN5-2） | PC7 | J5-4 | GAIN_A1 |
| D10（CN5-3） | PB6 | J5-5 | GAIN_A0 |
| D4（CN9-5） | PB5 | J5-6 | VGA_CS1，电压PGA片选 |
| D5（CN9-6） | PB4 | J5-7 | VGA_CS2，电流PGA片选 |

J5只接以上7根控制信号，不额外向J5送电。LCR板已把`GAIN_A0～A2`和`VGA_CS1/2`上拉到5V，固件把对应NUCLEO引脚配置为开漏；禁止改成推挽高电平。`MUX_A0/1`为普通推挽控制。

## 4. 电源与公共地

### 当前实际接法（按此还原）

1. NUCLEO的CN1 Micro-USB接电脑，负责NUCLEO供电、ST-LINK烧录和虚拟串口；JP5保持默认1-2位置（USB ST-LINK 5V供电）。
2. 独立稳压5V电源正极接LCR板J7-1或J7-2。
3. 独立稳压5V电源负极接LCR板J7-3或J7-4。
4. LCR板GND再接NUCLEO任一GND（可用CN6-6或CN6-7），使外部电源负极、LCR板GND、NUCLEO GND三者共地。
5. 当前记录的实际接法不使用NUCLEO CN6-5给J7供电；不要把NUCLEO 5V与外部5V并联。
6. J7不得接3.3V；通电后J7的`+5V`对`GND`应约为5.0V。

## 5. OLED四线接法

OLED各厂家的四针物理排列可能不同，必须按模块自身丝印`GND/VCC/SCL/SDA`接，不按“从左到右”猜测。

| OLED丝印 | NUCLEO-G474RE端 |
|---|---|
| VCC | 3.3V，CN6-4 |
| GND | GND，CN6-6或CN6-7 |
| SCL | D15 / PB8，CN5-10 |
| SDA | D14 / PB9，CN5-9 |

OLED只能接3.3V，不接J7的5V。模块实测7位I2C地址为`0x3C`，SSD1306 128×64初始化、测试图和`READY`页均已通过。

## 6. DUT四线端子J6

| J6针号 | 信号 | 接到两脚被测件 |
|---|---|---|
| 1 | DUT+（Force+） | 被测件同一侧正端/第一端 |
| 2 | DUT+SENSE（Sense+） | 与J6-1接到被测件同一侧，连接点尽量靠近元件引脚 |
| 3 | DUT-SENSE（Sense-） | 与J6-4接到被测件另一侧，连接点尽量靠近元件引脚 |
| 4 | DUT-（Force-） | 被测件另一侧负端/第二端 |

使用真正Kelvin夹时，Force和Sense分别走线，到被测件端子处才汇合。只有普通两线夹时：J6-1与J6-2并到元件一端，J6-3与J6-4并到另一端；不能只接两根Sense线，也不能缺少J6-1到J6-4的电流回路。更换电容前先放电。

## 7. 按键、串口和无需外接的部分

- 蓝色板载`B1 USER`对应PC13：待机时短按一次开始测量，测量中再次按下为中止。
- 黑色板载`B2 RESET`只复位并返回`READY`，不启动测量。
- 调试串口为115200 bit/s的LPUART1（PA2/PA3）。NUCLEO默认焊桥已把它接到板载ST-LINK虚拟串口，因此不需要另接USB转串口线；Windows COM号可能变化，不能把历史`COM23`当成固定接线。
- ST-LINK/SWD已集成在NUCLEO板上，正常烧录不需要再接外部SWD线。

## 8. 上电后的还原自检

1. 断电状态逐项核对3根模拟信号、7根J5控制线、J7电源/地和OLED四线。
2. 先确认外部电源负极、LCR板GND和NUCLEO GND已经共地，再接通电源；J7应约5.0V，OLED VCC应约3.3V。
3. 复位后OLED应显示`READY`；串口应报告OLED地址`0x3C`、100kΩ反馈档隔离、激励停止。
4. 正常供电且待机时，J1/V_AMP和J2/I_AMP的直流偏置应约1.67～1.68V。若两者约0.8V，优先检查J7是否欠压，不改算法。
5. 先接已知约10kΩ电阻做一次完整测量，再接已放电电容。100kΩ反馈支路是已知坏档，固件会明确隔离并回退到健康档；这不是外部接线缺失。

## 9. 依据

- LCR板针号、网络名和方向来自项目内`.agents/原理图.png`及`.agents/pcb.png`。
- NUCLEO Arduino/Morpho针号、电源和默认LPUART1虚拟串口连接依据ST官方UM2505（STM32G4 Nucleo-64 boards，MB1367）：https://www.st.com/resource/en/user_manual/um2505-getting-started-with-stm32g4-series-nucleo64-board-stmicroelectronics.pdf
