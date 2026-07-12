# AURIX Development Studio 工程打开与配置说明

## 已配置内容

- ADS 路径：C:\Infineon\AURIX-Studio-1.9.12\AURIX-studio.exe
- 工作区路径：C:\Users\23898\Desktop\carfast\.ads-workspace
- 一键打开脚本：C:\Users\23898\Desktop\carfast\打开AURIX工程_carfast.bat
- 命令行干净 ini：C:\Users\23898\Desktop\carfast\.ads-workspace\AURIX-studioc-clean.ini

## 当前建议工程

先导入并使用：

- E3_04_drv8701e_double_motor_contro_demo：你的手动开车工程，当前不修改源码。
- Seekfree_TC264_Opensource_Library：逐飞 TC264 库工程/例程。

`auto` 文件夹是自动驾驶模块源码目录，不是单独 ADS 工程。后续需要真正编译自动驾驶时，再选一个目标工程，把 `auto` 目录加入源码路径和头文件路径。

## 如果 ADS 左侧没显示工程

1. File -> Import...
2. General -> Existing Projects into Workspace
3. Select root directory 填：C:\Users\23898\Desktop\carfast
4. 勾选 E3_04_drv8701e_double_motor_contro_demo 和 Seekfree_TC264_Opensource_Library
5. 不勾选 Copy projects into workspace
6. 点击 Finish

## 后续接入 auto

等确认目标工程后，再添加：

- 源码目录：C:\Users\23898\Desktop\carfast\auto
- 头文件目录：C:\Users\23898\Desktop\carfast\auto

目前保持手动开车工程不被修改。

## 已新增自动驾驶工程

已创建并导入 ADS：`auto_subject1_tc377`

这个工程是从 `E3_04_drv8701e_double_motor_contro_demo` 复制出来的自动驾驶版本，原手动工程没有改动。

当前已配置：

- 工程名：`auto_subject1_tc377`
- 链接源码目录：`C:\Users\23898\Desktop\carfast\auto`
- 头文件路径：已加入 `${workspace_loc:/${ProjName}/auto}`
- 主循环入口：`user/cpu0_main.c` 调用 `auto_seekfree_runtime_init/start/loop`
- 10ms 中断入口：`user/isr.c` 调用 `auto_seekfree_runtime_update10ms()`
- 已生成固件：`auto_subject1_tc377/Debug/auto_subject1_tc377.elf` 和 `.hex`

打开 ADS 后，在左侧选择 `auto_subject1_tc377`，直接 `Build Project`；需要烧录时使用该工程 `Debug` 下的 `.elf` 或 `.hex`。

## 左下角感叹号说明

如果 ADS 左侧出现名为 `carfast` 的工程并带黄色感叹号，这是因为总目录曾经有一个空的 `.project`，ADS 把整个总文件夹误识别成普通工程。

现在已经把这个空工程文件改名为 `carfast.project.disabled`。真正要用的工程是 `auto_subject1_tc377`。

如果 ADS 里还残留 `carfast`，右键它选择 `Delete`，不要勾选删除磁盘文件，只从工作区移除即可。

