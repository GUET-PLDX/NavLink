# HostDataLink

## LibXR 2.0 静态装配

使用 xrobot 1.0.1 的 `module` / 有序 `args`；构造函数显式注入外设与服务，实例按依赖顺序创建。参数名和顺序以头文件中的构造函数为准，固件示例见 `User/RobotConfig/`。

保留 PLDX 控制、协议、超时与离线保护。

导航包使用 13 字节速度协议，`vx_mps/vy_mps/wz_rad_s` 为车体系速度和偏航角速度，另有一个速度有效标志。有效速度命令写入 `CMD::Data::chassis.vx/vy/wz`，不启用力前馈。接收端检查标志、有限值和 CAN 编码范围。

控制输入回调在模块锁内缓存导航数据并通过 `FeedAIChassis` 更新 AI 底盘分片；行为回调分发模式候选及决策 Topic。导航命令按 150 ms 判定新鲜度，失效或速度标志关闭时撤销 AI 底盘控制，CMD 按现有逻辑回退到遥控输入。导航普通模式和小陀螺模式分别使用物理速度 CAN 编码，实际是否采用由 DualBoard 的一致控制结果决定。

视觉目标与开火改由 TalosLink 通过 `FeedAIGimbal` 提交；本模块不再订阅 `target_euler` / `fire_notify`。

`gimbal_feedback` 的字段有效位按字段独立计时，250 ms 未更新即清除；角速度仍使用 50 ms 窗口。反馈维护周期会在静默后补发清除后的 flags。`BehaviorData` 的购买/复活请求是独立累计计数器：接收端以首帧建立基线，某个计数下降视为该计数器重置并更新基线，该计数器本帧不生成事件。该兼容协议没有 session 标记，发送端重启后的首帧不会重放事件。
