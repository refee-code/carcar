# Carfast 上位机测试 App

用于室内架车调试。App 可以通过 USB 串口读取车端遥测，也可以开启模拟数据先看界面。

## 启动

```powershell
cd host_app
python app.py
```

如果需要串口连接，先安装依赖：

```powershell
python -m pip install -r requirements.txt
```

不装依赖也可以先打开 App，点击“模拟数据”查看界面。

## 车端遥测格式

App 解析一行一个数据帧，格式如下：

```text
TEL t=12345 mode=RUN x=12.3 y=-4.5 h=18.0 d=120.0 v=25 st=40 p=12 n=80 e=15 adc=1785 se=20 so=2800 l=30 r=-32
```

字段含义：

- `x/y`: 惯导位置，单位 cm
- `h`: 航向角，单位 deg
- `d`: 累计距离，单位 cm
- `v`: 速度指令百分比
- `st`: 转向指令百分比
- `p/n`: 当前目标点/总点数
- `e`: 航向误差，单位 deg
- `adc`: 方向 ADC 当前值
- `se`: 舵机 PID 误差
- `so`: 舵机 PID 输出
- `l/r`: 左右后轮编码器计数
