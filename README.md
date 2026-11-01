# 三连杆并联板球平台

当前控制代码快照（2026-11-01）。三个工程各自的本地提交、树对象及依赖来源记录在 [SOURCE_VERSIONS.json](SOURCE_VERSIONS.json)。

| 当前工程 | 路径 | 说明 |
| --- | --- | --- |
| STM32F407 三电机固件 | [embedded/three_motor](embedded/three_motor) | V11角度外环/速度内环、重力补偿、最小角速度及USB回读 |
| 三电机调参上位机 | [host/3motor](host/3motor) | Web Serial调参，角度/速度跟踪浮窗、参数导入导出 |
| DualSense 姿态控制 | [host/dual-control-3rrs](host/dual-control-3rrs) | Roll/Pitch映射、轴线拖动、灵敏度、方向键5°等效升降 |
| DualForge 手柄驱动 | [ATINOMI/dualforge-cpp](https://github.com/ATINOMI/dualforge-cpp) | 独立维护的驱动依赖，本仓库不重复收录源码 |

嵌入式代码只使用 `embedded/three_motor/`，对应本地 `E:\HAL32\F4\three_motor`；旧版 Key 固件已从当前分支移除，历史版本仍可通过 Git 提交记录查阅。仓库原有的 `host/src/` 保留，DualForge 驱动统一从独立仓库获取。上表是当前版本的入口；不包含本机构建目录、IDE状态和运行时日志。

## 当前固件参数

详见 [FINAL_PARAMETERS.md](embedded/three_motor/FINAL_PARAMETERS.md)。最新用户JSON原件位于固件 `config/`：角度Kp为9，M1/M2双向Ki=2，M3上升Ki=3、下降Ki=2；最小角速度50/50/40°/s，最大100/100/150°/s；到位范围±0.7°、连续10ms。角度预设[-70,-30,-30]°，需手动启动角度环。

固件沿用用户已确认的策略：取消0°/−80°固定角度保护，姿态控制范围由严格三维逆解判定；保留S/Q急停、限流与通信/反馈超时。初始编码器角度12.77°，上升为负方向。手柄控制默认Pitch轴对齐中心至M3径向，网页可修改轴向、互换、正负方向和灵敏度；这些设置保存在各浏览器本地，不包含在Git备份中。

## 启动和构建

调参台在 `host/3motor` 中运行 `start.bat`，使用Chrome/Edge连接平台USB；配套固件为V11。两个上位机不要同时占用平台串口。

手柄上位机的构建脚本沿用本机CLion/MinGW和SDL离线源配置。先从 [DualForge 仓库](https://github.com/ATINOMI/dualforge-cpp) 获取驱动，再通过 `-DriverRoot` 指定路径；已有本地驱动可直接复用。从本仓库根目录执行：

```powershell
git clone https://github.com/ATINOMI/dualforge-cpp.git ..\dualforge-cpp
$driverRoot = (Resolve-Path ..\dualforge-cpp).Path
Set-Location host/dual-control-3rrs
powershell -ExecutionPolicy Bypass -File .\build.ps1 -DriverRoot $driverRoot -SdlSource C:\path\to\SDL3-3.4.18
.\start_control.bat
```

旧版 Qt 上位机（`host/CMakeLists.txt`）同样使用外部驱动，默认查找本仓库旁的 `dualforge-cpp/`；可用 CMake 参数 `-DDUALFORGE_ROOT=C:/path/to/dualforge-cpp` 修改。

根据本机环境调整 `build.ps1` 的工具链路径。STM32固件在其目录使用ARM工具链与CMake的Debug预设构建，详见工程CMake配置；不会因运行网页而自动烧录。

## 已完成的离线验证

- 固件已编译；真实初始化、USB解析和1ms控制回调的离线检查通过。
- 调参台61项JavaScript回归测试通过，覆盖新旧协议、配置文件和控制退出。
- 手柄控制31项JavaScript测试通过，覆盖轴向/灵敏度、可达性、方向键和停止。

```powershell
# 在host/3motor目录
node --test tests/*.test.cjs
python tests/firmware/check_firmware.py ..\..\embedded\three_motor
# 在host/dual-control-3rrs目录
node --test tests/*.test.cjs
```

本次Git同步不进行烧录或实机运动；实机表现由用户验收。
