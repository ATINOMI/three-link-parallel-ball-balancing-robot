# 角度环最小角速度 V11

当前固件已按最新调参文件设置三路最小速度50/50/40°/s、最大100/100/150°/s；以下默认0描述是功能加入时的基线。当前值见FINAL_PARAMETERS.md。


每个电机新增一个上下方向共用的minSpeed（输出轴°/s），默认0，范围0～该电机maxSpeed。仅在误差超出到位范围时，将非零角度PID速度输出的绝对值抬至此下限，保留PID符号；PID本身输出0仍保持0。进入到位范围的稳定确认期间不抬高，稳定到位后仍输出0；速度内环、急停和超时逻辑不变。

J扩展为 `J,motor,direction,kp_milli,ki_milli,kd_milli,max_speed_milli,min_speed_milli`，一次原子配置。旧6参数J继续兼容，下限恢复0。V11保留V10的105字段，再追加三路minSpeed×1000，共108字段，文本上限1280不变。JSON仍version3，可选position.pid[i].minSpeed；旧文件/旧帧不伪造新字段，旧固件保持原角度功能，但不能下发带minSpeed的新配置。设备回读确认包含下限，不得用发送值冒充已生效。

固件提供ANGLE_MIN_SPEED_DPS默认宏。界面入口在角度环调参，按M1/M2/M3选择，切换方向仍共享该电机下限；导入、导出、批量配置与离线演示同步支持。手柄上位机仅补V11解析与运行兼容，不更改用户保存在浏览器的轴向/灵敏度设置。新功能需重新烧录，未执行自动烧录或实机运动。

加入前固件2bca96a、调参台51cca29，标签before-min-angle-speed-20261101；不移动原最终参数标签。

验证：在配套3motor目录运行 `python tests/firmware/check_firmware.py`，使用实际main.c的USB解析、PID和1ms回调进行主机离线测试；不连接硬件。
