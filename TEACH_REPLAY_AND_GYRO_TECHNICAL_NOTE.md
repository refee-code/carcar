# Teach & Replay 技术说明 + IMU963RA 陀螺仪校准记录

## 1. 文档目的

本文档用于说明当前项目中：

1. **IMU963RA 航向链路的校准情况**
2. **“推车走一遍，车辆再自主走一遍”功能的当前技术现状**
3. **为什么地图/路线会不准**
4. **下一步最该改什么**

项目路径：
- 自动模块：[auto/](auto/)
- TC377工程：[auto_subject1_tc377/](auto_subject1_tc377/)

---

## 2. 陀螺仪校准结论

### 2.1 背景

车辆当前的平面位姿估计主要依赖：
- **IMU航向角**
- **编码器累计距离**

核心积分关系在 [auto_pose.c:79-98](auto/auto_pose.c#L79-L98)：
- 先读取编码器增量 `delta_cm`
- 再用 `heading_deg` 把距离投影到 `x_cm / y_cm`

也就是说，只要航向角比例不对，地图和回放路线就一定会歪。

---

### 2.2 实测方法

测试步骤：
1. 上电，等待菜单稳定显示 `HX`
2. 记录初始 `HX`
3. **精准旋转车头 90°**
4. 记录终止 `HX`
5. 用 `ΔHX` 和 90° 对比，反推比例系数

---

### 2.3 最终校准结果

最终一次有效测试：
- 初始 `HX = 0.3`
- 旋转 90° 后 `HX = 87.2`
- 实际变化量 `ΔHX = 86.9°`

误差约：
- `90 - 86.9 = 3.1°`
- 相对误差约 `3.4%`

当前建议使用的陀螺仪比例系数：

```c
#define AUTO_SEEKFREE_GYRO_SCALE (1.03f)
```

所在文件：[auto_config.h:104](auto/auto_config.h#L104)

应用位置：[auto_seekfree_port.c:82-84](auto/auto_seekfree_port.c#L82-L84)

```c
gyro_z_dps = imu963ra_gyro_transition(imu963ra_gyro_z)
             * AUTO_SEEKFREE_GYRO_SCALE;
```

---

### 2.4 校准结论

**结论：陀螺仪比例链路已经基本可用，不再是当前最主要矛盾。**

它会影响长期积分误差，但从当前测试看，最大的系统性误差已经不是“90度只转出30多度”这种级别的问题了。

---

## 3. 当前 Teach & Replay 架构真实现状

下面是按代码核对后的事实结论。

---

### 3.1 自动定位只用了一个编码器

当前位姿估计接口定义在 [auto_platform.h:12-14](auto/auto_platform.h#L12-L14)：

```c
read_imu_heading_deg
read_gps_lat_lon
read_encoder_distance_cm
```

这里只有**一个编码器距离输入**，没有左右轮各自的输入。

实际 Seekfree 适配层也只实现了一个编码器读数：[auto_seekfree_port.c:119-140](auto/auto_seekfree_port.c#L119-L140)

配置中只定义了一个编码器：
- [auto_config.h:98-101](auto/auto_config.h#L98-L101)
- `AUTO_SEEKFREE_ENCODER_INDEX = TIM5_ENCODER`

#### 影响

这意味着当前自动位姿模型不是“左右后轮求车体中心里程”，而是：

> **单编码器距离 + IMU航向角 → 推算 x/y**

在直线时问题不大，但转弯时单轮里程不等于车体中心里程，所以地图会变形。

---

### 3.2 位姿模型太简单

位姿更新在 [auto_pose.c:79-98](auto/auto_pose.c#L79-L98)

核心逻辑：

```c
delta_cm = encoder_distance_cm - last_encoder_distance_cm;
pose.x_cm += delta_cm * cosf(heading_rad);
pose.y_cm += delta_cm * sinf(heading_rad);
```

#### 这代表什么

当前模型本质上是：

> **航向已知 + 前进距离已知 → 平面积分**

它没有：
- 左右轮差速模型
- Ackermann / 自行车模型
- 前轮实际转角参与

#### 影响

只要车在转弯，真实车体中心运动轨迹就不是这么简单的“沿 heading 平移”，所以地图和回放误差会随着曲率增加而放大。

---

### 3.3 前轮转向反馈没有进入自动定位

前轮反馈 ADC 在 legacy PID 里是存在的：
- [PID.c:90-147](auto_subject1_tc377/code/PID.c#L90-L147)

但自动定位接口 [auto_platform.h:12-14](auto/auto_platform.h#L12-L14) **没有 steering angle read 接口**。

也就是说：
- 代码知道 IMU 朝向变了多少
- 代码知道编码器累计距离
- **代码不知道前轮实际打了多少角**

#### 影响

推车录弯道时，车辆真实的几何轨迹依赖转向角。

但当前定位模型完全看不到这个量，所以即使地图能大致画出来，也不是“真正用整车运动学估算”的结果。

---

### 3.4 自动转向目前仍然是开环 PWM

自动转向输出路径：
- 导航输出 `steer_percent`：[auto_nav.c:173-176](auto/auto_nav.c#L173-L176)
- 底盘应用命令：[auto_app.c:80-86](auto/auto_app.c#L80-L86)
- 最终执行：[auto_seekfree_port.c:175-190](auto/auto_seekfree_port.c#L175-L190)

当前执行方式是：

```c
Set_Steering_Pwm(-pwm);
```

这意味着自动模式下是**开环打PWM**，而不是“目标角度 -> ADC反馈 -> PID闭环到位”。

#### 已有但未接入的能力

项目里已经有前轮 PID：
- 目标角接口：[PID.h:55-58](auto_subject1_tc377/code/PID.h#L55-L58)
- PID执行：[PID.c:142-148](auto_subject1_tc377/code/PID.c#L142-L148)

但自动路径没有走这套闭环。

#### 影响

导航即使算出了一个合理转角，前轮实际有没有打到位，当前自动代码并不保证。

这会直接破坏“路线复现准确性”。

---

### 3.5 前进 / 后退路线当前并没有真正可靠记录

类型系统里定义了：
- [auto_types.h:67-70](auto/auto_types.h#L67-L70)
- `AUTO_WAYPOINT_FLAG_REVERSE`

导航也会读取它：
- [auto_nav.c:110-114](auto/auto_nav.c#L110-L114)

但当前录点逻辑里，录制点时传入的是：
- [auto_seekfree_runtime.c:97-101](auto/auto_seekfree_runtime.c#L97-L101)
- [auto_seekfree_runtime.c:257-262](auto/auto_seekfree_runtime.c#L257-L262)

都还是：

```c
AUTO_WAYPOINT_FLAG_NONE
```

#### 影响

现在系统并没有稳定记录“这一段是前进还是后退”。

所以如果用户推车里包含倒车段，当前回放大概率不能严格按原始前进/倒退方式复现。

---

### 3.6 GPS代码还在，但当前已禁用

GPS在位姿估计代码中仍然存在：
- [auto_pose.c:101-137](auto/auto_pose.c#L101-L137)

但当前配置已经禁用：
- [auto_config.h:86](auto/auto_config.h#L86)
- `AUTO_SEEKFREE_USE_GNSS = 0`

#### 结论

GPS 当前不会影响实时 pose，这一点是正确的，至少不会继续污染局部坐标系。

---

## 4. 为什么会出现“地图路线和推车路线不一样”

### 4.1 已经不是单纯的显示问题

显示层之前确实有“红点擦除残影”的问题，但那只能解释黑色粗轨迹残影，**解释不了灰线本身的几何形状错误**。

如果灰线从一开始就和你推车路线不一致，那就是：

> **采点坐标本身就不对**

---

### 4.2 当前最核心的误差来源

当前最可能的根因是：

1. **车体中心里程没有正确估计**（只用一个编码器）
2. **转弯几何没有建模**（没用前轮角）
3. **自动转向没有闭环保证实际到位**

所以现在系统更像：

> 一个“单编码器 + IMU + 开环转向”的近似路径跟随器

而不是：

> 一个“用完整车辆运动学进行高保真示教复现”的系统

---

## 5. 当前地图显示缓存还有一个额外问题

地图显示使用的是：
- `s_converted_waypoints`

这个数组不是实时 recorder buffer，而是“录点结束/自动启动时转换出来的缓存”。

相关代码：
- [auto_seekfree_runtime.c:53-64](auto/auto_seekfree_runtime.c#L53-L64)
- [auto_seekfree_runtime.c:199-214](auto/auto_seekfree_runtime.c#L199-L214)
- [auto_seekfree_runtime.c:322-328](auto/auto_seekfree_runtime.c#L322-L328)

#### 影响

如果清空、撤销、重新录点过程中缓存没有同步更新，地图可能显示的不是“此刻 recorder 真正的内容”，而是旧缓存。

这会让显示问题和定位问题混在一起，更难判断。

---

## 6. 现阶段的总判断

### 6.1 可以确认已经做对的部分

- GPS 已隔离，不再污染局部坐标
- 陀螺仪比例经过校准，航向基本可用
- 编码器 100cm → 102cm，比例基本正确
- 路径地图能画出来，说明采点链路基本是通的

### 6.2 仍然没有解决的核心问题

当前代码还**不能稳定做到**：

> “我推着车走一遍，不管直线、转弯、前进、后退，第二次小车自主高精度完整复现原路线。”

不是因为某一个小参数没调好，而是因为架构还差关键传感器和模型：

- 单编码器，不是双后轮车体中心里程
- 没有前轮转角进入定位
- 自动转向开环，不知道前轮是否真打到位
- 前进/后退段没有可靠记录

---

## 7. 改造优先级（强烈建议顺序）

### Priority 1：把自动转向改成闭环角度控制

目标：让自动模式也走前轮 ADC + PID。

需要做的事：
- 给 auto 模式增加 `steer target angle` 概念
- 自动导航输出的是“目标转角百分比”
- 平台层不再 `Set_Steering_Pwm(-pwm)` 直接开环打电机
- 而是走 `Steering_Set_Target_Angle()` + `Steering_Control_Update()`

#### 为什么优先级最高

如果实际前轮角都不可靠，定位和回放再准也没意义。

---

### Priority 2：把定位升级成“双后轮编码器平均 + IMU航向”

目标：估计车体中心的真实位移。

理想模型：

```text
center_distance = (left_distance + right_distance) / 2
heading = imu_heading
x += center_distance_delta * cos(heading)
y += center_distance_delta * sin(heading)
```

#### 这一步的意义

即使还没上 Ackermann，自少车体平移距离比单轮靠谱得多。

---

### Priority 3：把前轮转角真正接入定位模型

目标：从“二维平移积分”升级到“车辆运动学模型”。

至少需要：
- 前轮实际转角读入 auto 平台层
- pose estimator 里引入 wheelbase / steer angle
- 用 bicycle / Ackermann 模型更新位置与航向

---

### Priority 4：可靠记录前进 / 后退段

需要做到：
- 每一段路径都明确知道是 forward 还是 reverse
- 回放时严格按记录方向执行
- 不再依赖 fallback 的 heading 角猜测

---

### Priority 5：地图显示只使用当前录点的真实数据源

建议：
- 录点界面优先显示 `s_route_recorder.points`
- 自动启动后再显示转换后的 replay route
- 避免 recorder / converted cache 两套数据互相误导

---

## 8. 结论（给项目决策用）

### 当前最准确的技术结论

> **现在的主矛盾不是陀螺仪。**
>
> 当前最大的结构性问题是：
> - 自动定位没有使用完整车辆运动学
> - 自动转向还是开环
> - 只用了一个后轮编码器
> - 没把前轮转角反馈纳入定位
>
> 所以地图和回放路线不准，不是某一个参数没调，而是“车体中心轨迹”本来就没有被正确估计。

### 如果目标是“高精度示教复现”

当前推荐路线是：

1. **自动转向改闭环**
2. **定位改双编码器中心里程**
3. **接入前轮转角做运动学模型**
4. **记录前进/后退段**

只有做到这四步，系统才有可能稳定接近：

> “我推着车怎么走，它第二次就怎么走。”

---

## 9. 相关代码索引

### 位姿 / 定位
- [auto_pose.c](auto/auto_pose.c)
- [auto_platform.h](auto/auto_platform.h)
- [auto_seekfree_port.c](auto/auto_seekfree_port.c)
- [auto_config.h](auto/auto_config.h)

### 自动转向 / 执行
- [auto_nav.c](auto/auto_nav.c)
- [auto_app.c](auto/auto_app.c)
- [auto_chassis.c](auto/auto_chassis.c)
- [PID.c](auto_subject1_tc377/code/PID.c)
- [isr.c](auto_subject1_tc377/user/isr.c)

### 录点 / 回放 / 地图缓存
- [auto_seekfree_runtime.c](auto/auto_seekfree_runtime.c)
- [auto_route_recorder.c](auto/auto_route_recorder.c)
- [auto_route.c](auto/auto_route.c)
- [auto_screen_seekfree.c](auto/auto_screen_seekfree.c)
