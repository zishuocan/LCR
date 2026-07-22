# LCR_G474 开发计划

#必须注意的事项
- 每次只完成较小且完整的改动，让然后和我对接进度。严谨闷头一直干活。
- - 有问题及时询问，严禁自作主张。

## 1. 项目目标

使用 NUCLEO-G474RE 控制现有自平衡电桥板，实现一键自动阻抗测量。

验收范围：

- 电阻：10 Ω～100 kΩ
- 电容：1 nF～4.7 µF
- 电感：10 µH～100 µH
- 显示复阻抗：R ± jX
- 自动识别电阻、电容、电感
- 电容显示 C 和 ESR
- 电感显示 L 和串联电阻
- 相对误差绝对值不超过 5%
- 使用商用 LCR 表进行校准
- 不使用 AD5933 等专用阻抗测量芯片

模拟板已经验证功能正常，不要重新排查或修改模拟电路，除非 STM32 采集结果明确异常。

---

## 2. 当前硬件配置

完整的逐线还原说明见`.agents/hardware_wiring.md`。该文件包含NUCLEO连接器针号、LCR板J1/J2/J3/J5/J6/J7针号、OLED、电源、DUT Kelvin并接方式和上电自检；本节仅保留软件设计所需的信号摘要。

### 模拟信号

| MCU 引脚 | 外设功能 | LCR 板信号 |
|---|---|---|
| PA4 | DAC1_OUT1 | SINE |
| PA0 | ADC1_IN1 | V_AMP |
| PA1 | ADC2_IN2 | I_AMP |

物理连线以此表为准：NUCLEO A2/PA4 → LCR板J3/SINE，LCR板J1/V_AMP → NUCLEO A0/PA0，LCR板J2/I_AMP → NUCLEO A1/PA1，两板GND共地。LCR板J7接稳定+5 V和公共地。DUT接J6四线端子，顺序为1=DUT+、2=DUT+SENSE、3=DUT-SENSE、4=DUT-。

### 量程和增益控制

| MCU 引脚 | 信号 | GPIO 模式 |
|---|---|---|
| PB10 | MUX_A0 | Push-Pull |
| PA8 | MUX_A1 | Push-Pull |
| PA9 | GAIN_A2 | Open-Drain |
| PC7 | GAIN_A1 | Open-Drain |
| PB6 | GAIN_A0 | Open-Drain |
| PB5 | VGA_CS1 | Open-Drain |
| PB4 | VGA_CS2 | Open-Drain |

GAIN_A0～A2、VGA_CS1、VGA_CS2 由 LCR 板上拉至 5 V，必须保持开漏输出，禁止改成推挽高电平。

### 其他外设

- 系统时钟：160 MHz
- TIM6：同时触发 DAC 和双 ADC
- TIM6 的 CubeMX 初值为 PSC=0、ARR=499；64 点/10 kHz 运行时由激励模块设为 ARR=249
- 10 kHz 基准档使用 64 点正弦表（由首次验证的 32 点表升级）
- DAC1 DMA：Circular，外设端 Word、内存端 Half Word
- ADC1、ADC2：Dual regular simultaneous mode
- ADC1 DMA：Normal，Word
- I2C1：PB8/SCL（Arduino D15）、PB9/SDA（Arduino D14），400 kHz，用于0.96英寸四针OLED；实测7位地址0x3C，OLED VCC接NUCLEO 3.3 V、GND接公共地
- LPUART1：PA2/PA3，115200，用于调试
- PC13：蓝色板载B1 USER测量按键；黑色RESET键只复位、不启动测量
- PA5：板载状态 LED

---

## 3. 软件模块

建议创建：

- lcr_excitation.c/.h：DAC 激励和频率控制
- lcr_capture.c/.h：双 ADC 同步采集
- lcr_range.c/.h：MUX 和 PGA 控制
- lcr_dsp.c/.h：单频 DFT 和复数运算
- lcr_calibration.c/.h：复数校准参数
- lcr_measurement.c/.h：自动测量状态机
- lcr_classify.c/.h：R/C/L 识别与换算
- lcr_display.c/.h：串口和 OLED 显示

业务代码放在独立文件或 CubeMX 的 USER CODE 区域，不要修改自动生成区域。

---

## 4. 激励模块

实现 DAC1 + DMA 循环正弦输出。

要求：

- 初期使用 32 点正弦表，滤波后波形验证阶段升级为 64 点
- 默认频率 10 kHz
- 至少支持 1 kHz、10 kHz 和高频电感测量档
- 频率通过修改 TIM6 ARR 切换
- 修改频率前停止 TIM6、DAC DMA
- 修改完成后重新启动
- 激励幅度可调
- 避免模拟前端和 ADC 削顶
- 64 点表的 DAC/ADC 同步触发率限制为 800 kS/s；更高频档使用较短正弦表

建议函数：

- LCR_ExcitationInit()
- LCR_SetFrequency(uint32_t frequency_hz)
- LCR_SetAmplitude(uint16_t amplitude)
- LCR_ExcitationStart()
- LCR_ExcitationStop()

高频档是否使用 100 kHz，需要根据当前模拟板带宽和 ADC/DAC 采样结果验证。必要时高频档改用 16 点正弦表。

---

## 5. 量程模块

实现：

- LCR_SetFeedbackRange()
- LCR_SetVoltageGain()
- LCR_SetCurrentGain()

反馈挡由 MUX_A1、MUX_A0 控制。

必须根据原理图确认每个编码对应的实际反馈网络，在代码中建立明确表格，不要凭猜测写死。

PGA 增益编码：

| A2 A1 A0 | 增益 |
|---|---:|
| 000 | 1 |
| 001 | 2 |
| 010 | 4 |
| 011 | 8 |
| 100 | 16 |
| 101 | 32 |
| 110 | 64 |
| 111 | 128 |

设置某一路增益时：

1. 设置 GAIN_A2、GAIN_A1、GAIN_A0
2. 将对应 VGA_CS 拉低
3. 延时数微秒
4. 释放 VGA_CS 为高
5. 等待模拟链路稳定

上电默认状态：

- 最低跨阻或最安全反馈挡
- V 通道增益 1
- I 通道增益 1
- DAC 暂不启动

---

## 6. 双 ADC 采集模块

ADC1 采集 V_AMP，ADC2 采集 I_AMP。

必须使用双 ADC 同步模式，禁止分别启动两个 ADC。

启动接口使用：

    HAL_ADCEx_MultiModeStart_DMA(...)

缓冲区使用：

    uint32_t adc_dual_buffer[SAMPLE_COUNT];

初期：

    SAMPLE_COUNT = 1024

每个 32 位采样值包含 ADC1 和 ADC2 的结果。必须结合 STM32G4 HAL、寄存器文档或实际测试确认高低 16 位顺序，不能直接假定。

采集流程：

1. 校准 ADC1 和 ADC2
2. 设置频率、反馈挡和增益
3. 启动 DAC 激励
4. 等待若干完整周期稳定
5. 启动双 ADC DMA
6. 收满固定长度后停止
7. DMA 回调只设置完成标志
8. 在主循环中处理数据

ADC 校准：

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

不要在中断中执行 DFT、printf 或 OLED 刷屏。

---

## 7. DSP 模块

禁止只使用峰峰值相除。

对 V、I 两路执行单频 DFT或数字锁相计算：

1. 分别计算两路平均值
2. 去除直流偏置
3. 在已知激励频率处计算实部和虚部
4. 得到 V、I 复数相量
5. 计算复数比例 V/I
6. 输出幅值和相位

建议复数结构：

    typedef struct {
        float real;
        float imag;
    } complex_f32_t;

采样应尽量包含整数个激励周期，避免频谱泄漏。

打开 Cortex-M4 FPU，DSP 和复数计算使用 float。

---

## 8. 校准模块

最终阻抗不要只依赖标称反馈电阻计算。

使用：

    Z = K × Vphasor / Iphasor

其中 K 是复数校准系数，可同时修正幅度误差和相位误差。

校准参数至少按以下条件区分：

- 测量频率
- 反馈挡
- V 通道增益
- I 通道增益

开发初期先用商用 LCR 表测得的标准电阻进行单点复数校准。

后续加入：

- 开路校准
- 短路校准
- 标准负载校准
- Flash 保存与读取
- 校准数据版本和有效性检查

与商用仪器比较时，必须保持相同测试频率、激励幅值和串并联等效模式。

---

## 9. 自动量程

自动量程同时检查 V_AMP 和 I_AMP。

判断条件：

- ADC 接近 0 或 4095：削顶
- 交流幅度过大：降低 PGA 增益、反馈阻抗或激励幅度
- 交流幅度过小：提高 PGA 增益或反馈阻抗
- 两路信号都必须处于有效范围
- 每次切换量程后重新等待稳定并重新采集
- 设置最大换挡次数，禁止无限循环

建议将信号有效范围定义为 ADC 满量程的约 10%～80%，具体阈值根据实测调整。

---

## 10. 测量状态机

实现非阻塞状态机：

    IDLE
    SETUP
    SETTLING
    CAPTURE
    RANGE_CHECK
    DSP
    CALIBRATE
    CLASSIFY
    DISPLAY
    ERROR

按下 PC13 后启动一次完整测量。

测量完成后回到 IDLE，等待下一次按键。

状态机必须具备：

- DMA 超时
- ADC 削顶
- 信号过小
- 量程切换失败
- 无被测元件或开路
- 测量结果无效
- 最大重试次数限制

---

## 11. 元件识别与参数换算

测得：

    Z = R + jX

保留：

    R = Re(Z)
    X = Im(Z)

初步分类：

- 相位接近 0：电阻
- X < 0：电容
- X > 0：电感

参数换算：

    C = -1 / (2πfX)
    L = X / (2πf)

电容同时显示 ESR = R。

电感同时显示串联电阻 R。

不要仅凭一次相位阈值直接判定。模糊区域应切换频率重新测量。

建议策略：

- 10 kHz：粗测和电阻测量
- 1 kHz：较大电容
- 高频档：10～100 µH 电感
- 根据初测阻抗和相位自动选择精测频率

---

## 12. 显示和调试输出

串口至少输出：

- 测量频率
- 反馈挡
- V 通道增益
- I 通道增益
- V 通道均值、幅值、相位
- I 通道均值、幅值、相位
- 原始复阻抗
- 校准后复阻抗
- 元件类型
- R、C 或 L
- 削顶、过小或超范围状态

OLED 最终显示：

- Z = R ± jX Ω
- Type = R / C / L
- R = ...
- C = ... 或 L = ...
- f = ...

---

## 13. 推荐开发顺序

### 阶段 1：底层输出和控制

- DAC DMA 输出稳定正弦
- 示波器确认 PA4 频率和幅值
- 完成 MUX 控制
- 完成两个 PGA 增益控制
- 串口输出当前硬件状态

### 阶段 2：同步采集

- ADC1、ADC2 校准
- 双 ADC DMA 采集 1024 组数据
- 正确拆分两路数据
- 串口输出均值、最大值、最小值和峰峰值
- 检测削顶和异常

### 阶段 3：复数相量

- 完成去直流
- 完成单频 DFT
- 输出 V、I 幅值和相位
- 验证重复测量结果稳定

### 阶段 4：固定量程电阻测量

固定 10 kHz、反馈挡和增益，使用标准电阻校准并测试：

- 10 Ω
- 100 Ω
- 1 kΩ
- 10 kΩ
- 100 kΩ

### 阶段 5：自动量程

- 自动切换反馈挡
- 独立设置 V、I 增益
- 自动处理削顶和信号过小
- 增加最大重试次数

### 阶段 6：电容和电感

- 加入多频率测量
- 完成 R/C/L 自动识别
- 验证 1 nF～4.7 µF
- 验证 10 µH～100 µH
- 根据结果调整高频档采样点数和频率

### 阶段 7：校准和显示

- 多频率、多挡位复数校准
- Flash 保存校准参数
- OLED 显示
- 一键完整测量
- 完成误差统计

---

