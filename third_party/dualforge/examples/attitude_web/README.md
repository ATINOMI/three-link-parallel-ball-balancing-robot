# DualSense 姿态观测 / Motion sketch

复用现有DualForge驱动，新增一个独立、仅用于手柄IMU观测的Windows示例。没有接入三电机控制，没有修改原控制器驱动实现。

## 启动

项目根目录双击`start_attitude.bat`，会运行`build-attitude/dualforge_attitude_web.exe`并打开：

**http://127.0.0.1:8876/**

用USB连接普通DualSense（VID 054C / PID 0CE6）。当前DualForge版本仅支持USB；蓝牙和DualSense Edge不在现有驱动支持范围内。首次连接或点击“静止校准”后，将手柄放稳2秒；有明显运动时该2秒窗口重新计时。页面提供：

- 手绘风立方体，按四元数实时旋转；橙色F面为正面。
- 横滚Roll、俯仰Pitch、相对偏航Yaw，以及真实传感器事件频率。
- 姿态归零：只改变显示参考，不重置融合状态或零偏。
- 静止校准：估计并扣除SDL标定后残余的陀螺仪零偏。
- 可展开的原始传感器数据和零偏读数。

关闭浏览器不会停止后台采样；在程序控制台按Ctrl+C停止。可选`--seconds 10`限时运行、`--no-browser`不自动开浏览器、`--port 8877`更换本地端口。`--demo`为明确标记的合成旋转演示，不打开手柄，不是实机数据。

## 姿态算法与坐标

`attitude.hpp`为独立的C++17四元数互补估计器：陀螺仪积分＋Mahony形式的重力方向比例反馈，重力纠偏增益2.5/s。用加速度方向初始化倾斜角；加速度模长偏离1g时降低重力反馈，超过1.5m/s²偏差时暂停重力纠偏，减小挥动时的倾角污染。只有陀螺仪，没有地磁航向参考，偏航是相对角度，长期会漂移。

通过原库`Context(EventMode::External)`打开设备、启用传感器和管理连接，按公开SDL事件逐条读取IMU，避免把每次refresh合并后的状态当成物理采样。积分优先使用`SDL_GamepadSensorEvent.sensor_timestamp`（ns），没有时用事件timestamp；大于50ms的间隔不积分，避免暂停/重连造成跳变。SDL手柄API全部在主线程调用，HTTP线程只交换快照和按钮请求。

沿用SDL坐标：X向右、Y向上、Z朝向玩家；gyro为rad/s，accel为m/s²。四元数为body→world，世界重力参考为+Y；界面欧拉角按`Ry(yaw) Rx(pitch) Rz(roll)`分解，因此俯仰绕X、偏航绕Y、横滚绕Z。立方体始终使用四元数，欧拉角在接近俯仰±90°时存在常规分解歧义。浏览器仅作25ms时间常数的显示插值，不能把显示插值当成融合器。

静止校准要求陀螺仪模长小于0.08rad/s、加速度模长与1g偏差小于0.5m/s²，连续积累2秒；移动会清空该窗口。它不是磁航向校准，也不会由手柄运动估计高度。

参考：[SDL传感器坐标与单位](https://wiki.libsdl.org/SDL3/SDL_SensorType)、[SDL事件时间戳](https://wiki.libsdl.org/SDL3/SDL_GamepadSensorEvent)、[Mahony/Madgwick原作者开源说明](https://x-io.co.uk/open-source-ahrs-with-x-imu/)。实现是本项目独立编写的比例重力反馈，没有复制第三方算法代码。

## 构建和测试

根CMake新增`DUALFORGE_BUILD_ATTITUDE_WEB`，默认ON，仅Windows编译该HTTP示例；库原有源文件和API不变。

```powershell
# 已有SDL 3.4.18源码时可以传入，避免联网下载。
.\build_attitude.ps1 -SdlSource '你的SDL3-3.4.18源码目录'
```

脚本使用本机CLion工具链、临时英文构建目录及独立PATH，成品和SDL/MinGW运行时DLL复制回项目`build-attitude`。工具链位置可在脚本的toolRoot修改；不修改全局PATH。普通CMake构建同样支持`-DDUALFORGE_SDL_SOURCE_DIR=...`。

```powershell
ctest --test-dir "$env:TEMP\dualforge-attitude-build" --output-on-failure
node --test examples/attitude_web/test_cube.cjs
```

算法测试覆盖重力初始对齐、SDL轴符号、偏航积分、归零、陀螺仪零偏、线性加速度抑制、间隔中断及40秒三维连续翻转。浏览器几何测试覆盖四元数轴映射、插值和各姿态Canvas绘制。真实数据可在`/api/state`核对；无设备或数据过期时页面明确显示等待状态，不用合成数据替代。
