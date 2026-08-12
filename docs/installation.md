# 安装文档：CAN 环境配置（NVIDIA Jetson L4T + gs_usb 驱动）

本文档记录在 **NVIDIA Jetson（Ubuntu 22.04, kernel `5.15.148-tegra`）** 上让
两个 **OpenMoko GS USB-CAN 转接器（`ID 1d50:606f`）** 正常工作的完整步骤，
使每台 Piper 机械臂独占一路 CAN 总线，实现双臂独立、并行控制。

> 本文档与硬件无关的开发机（Intel 平台）无关，所有命令都在 **Jetson** 上执行。

---

## 1. 背景与适用场景

- 机器人采用**双臂**结构：左右各一台 Piper 机械臂。
- 每台臂通过自带的 USB-CAN 转接器接入 Jetson 的两个 USB 口。
- 目标：两臂分别占用**独立的 CAN 接口**（如 `can1`/`can2`），各自使用默认
  CAN ID，互不干扰，同时可读可控制。

**判断是否适用**：`lsusb` 能识别到转接器，但 `ip link` 看不到对应 CAN 接口。

---

## 2. 问题现象

| 检查项 | 命令 | 预期 |
|---|---|---|
| USB 设备识别 | `lsusb` | 出现 `1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter`（两个） |
| CAN 接口列表 | `ip -br link show type can` | 只有板载 `can0`(mttcan)，**没有** USB 转接器对应的接口 |
| 内核日志 | `sudo dmesg \| grep -i can` | 只有 `mttcan device registered`，无 gs_usb 相关 |
| 内核模块 | `lsmod \| grep gs_usb` | 无输出 |

**根本原因**：L4T 内核未包含 `gs_usb` 驱动。

```text
CONFIG_CAN_RAW=m
CONFIG_CAN_BCM=m
CONFIG_CAN_DEV=m
# CONFIG_CAN_GS_USB is not set      <-- 缺失
```

---

## 3. 前置检查

```bash
# 内核版本与头文件（必须安装头文件）
uname -r                                  # 5.15.148-tegra
ls /usr/src/ | grep -i linux              # linux-headers-5.15.148-tegra-ubuntu22.04_aarch64
ls -l /lib/modules/$(uname -r)/build      # 应软链到头文件/内核源码目录
```

若 `build` 软链不存在，手动创建：

```bash
sudo ln -s /usr/src/linux-headers-5.15.148-tegra-ubuntu22.04_aarch64 \
           /lib/modules/$(uname -r)/build
```

---

## 4. 编译安装 gs_usb 内核模块

> 编译在目标机（Jetson）本地进行，使用与运行内核同源的头文件。
> 编译时若出现 "compiler differs from the one used to build the kernel" 警告
> 属正常现象，不影响结果。

```bash
cd ~ && mkdir -p gs_usb_build && cd gs_usb_build

# 下载 5.15 稳定分支的 gs_usb 驱动源码
wget https://raw.githubusercontent.com/gregkh/linux/linux-5.15.y/drivers/net/can/usb/gs_usb.c -O gs_usb.c

# 编写 Makefile
cat > Makefile <<'EOF'
obj-m += gs_usb.o
KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
EOF

# 编译（成功后会生成 gs_usb.ko）
make

# 安装并刷新模块依赖
sudo mkdir -p /lib/modules/$(uname -r)/extra
sudo cp gs_usb.ko /lib/modules/$(uname -r)/extra/
sudo depmod -a
```

**加载**（按依赖顺序）：

```bash
sudo modprobe can
sudo modprobe can-dev
sudo modprobe gs_usb
```

验证接口出现：

```bash
ip -br link show type can
# 现在应看到：can0（板载）+ can1 + can2（两个 USB-CAN）
```

---

## 5. 激活接口并设置波特率

两个 USB-CAN 接口波特率必须为 **1000000**（1 Mbps）：

```bash
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000

# 验证
ip -details link show can1
# 预期：state UP、bitrate 1000000、can state ERROR-ACTIVE
```

> 板载 `can0`(mttcan) 未接设备，保持 DOWN 即可，不要使用。

---

## 6. 确认机械臂在线与左右对应关系

机械臂需处于**从站模式**才会主动发送反馈帧。先探测：

```bash
candump can1 -T 3000     # 3 秒内应有 0x2A1 / 0x2A5 / 0x251 等帧
candump can2 -T 3000
```

若无数据，将该口机械臂设为从站（发送一次 0x470 配置）：

```bash
cargo run --release --example set_slave -- can1
cargo run --release --example set_slave -- can2
candump can1 -T 3000
candump can2 -T 3000
```

用库读取、手动晃动对应机械臂，确定左右对应关系：

```bash
cargo run --release --example read_joint -- can1
cargo run --release --example read_joint -- can2
```

记录每个 USB-CAN 转接器的 USB 硬件地址（用于第 7 步固定命名）：

```bash
ip -details link show can1 | grep parentdev   # 如 1-2.4.4.3:1.0
ip -details link show can2 | grep parentdev   # 如 1-2.4.4.2:1.0
```

---

## 7. （推荐）固定接口名 + 开机自动激活

`can1`/`can2` 编号在重启后可能互换，且 CAN 接口重启后为 DOWN。已提供一键
脚本完成全部配置（udev 固定命名 + systemd 开机自动激活）：

```bash
# 在 Jetson 上执行前，先确认脚本顶部 LEFT_USB / RIGHT_USB 与你的 USB 地址一致
sudo bash scripts/setup_can_dual_arm.sh
```

脚本会：
1. 确保 `gs_usb` 内核模块**开机自动加载**（`/etc/modules-load.d/gs_usb.conf`）并立即加载；
2. 写入 udev 规则，按 USB 硬件地址把接口固定命名为 `can_left`/`can_right`；
3. 立即重命名当前已加载的接口（若未生效，重新插拔转接器）；
4. 写入并启用 `can-bringup.service`，开机自动把两接口设为 UP、1 Mbps。

> **重启后 `can_left` 不存在**的排查：先 `lsmod | grep gs_usb`，若未加载执行
> `sudo modprobe gs_usb`；之后 `ip -br link show type can` 应出现
> `can_left`/`can_right`。若为旧版脚本，直接重跑
> `sudo bash scripts/setup_can_dual_arm.sh` 即可补齐开机自动加载。

> **注意：机械臂上电后需要约 30~60 秒完成启动**，启动完成前 CAN 收发器不在
> 总线上——此时收不到任何反馈、发送会报 ENOBUFS/bus-off，属正常现象，不是
> 硬件故障。请**先等待臂启动完成**再运行程序。

> **重插 USB-CAN 转接器后**接口会重建且默认 DOWN（`cansend` 报 "Network is
> down"）。新版配置脚本已通过 udev `RUN+=` 在重插时自动拉起；老版本请手动
> `sudo ip link set can_left up type can bitrate 1000000`。

如需手动配置，等价内容如下。

### 7.1 udev 规则固定名字

按 USB 硬件地址（`parentdev`，如 `1-2.4.4.3`）编写规则（等价于脚本生成的规则）：

```bash
sudo tee /etc/udev/rules.d/70-piper-can.rules > /dev/null <<'EOF'
# 左臂 USB-CAN → can_left
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="gs_usb", KERNELS=="1-2.4.4.3", NAME="can_left"
# 右臂 USB-CAN → can_right
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="gs_usb", KERNELS=="1-2.4.4.2", NAME="can_right"
EOF

sudo udevadm control --reload-rules
# 重新插拔转接器后生效
```

> `1-2.4.4.3` / `1-2.4.4.2` 需按第 6 步实际 `parentdev` 替换。

### 7.2 systemd 服务自动激活

```bash
sudo tee /etc/systemd/system/can-bringup.service > /dev/null <<'EOF'
[Unit]
Description=Bring up Piper CAN interfaces
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can_left  up type can bitrate 1000000
ExecStart=/sbin/ip link set can_right up type can bitrate 1000000

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now can-bringup
```

---

## 8. 故障排查

| 现象 | 排查 |
|---|---|
| `modprobe gs_usb` 报 `Unknown symbol` | 先 `sudo modprobe can-dev`，再 `sudo insmod gs_usb.ko`；若仍失败，需为编译补上 `Module.symvers` |
| `make` 报 API 编译错误 | 换用 v5.15 分支其它提交的 `gs_usb.c`，或把报错信息提供维护者 |
| 接口出现但 DOWN | `sudo ip link set canX up type can bitrate 1000000` |
| candump 无反馈帧 | 机械臂未在从站模式，先发送 0x470 从站配置（`set_slave` 示例） |
| 接口不稳定/重启消失 | 参见第 7 节 udev + systemd 固定 |

---

## 9. 最终验证

```bash
ip -br link show type can
# can0(忽略)  can1  can2  (或 can_left / can_right) 均 UP

# 双臂同时控制
cargo run --release --example read_joint -- can1
cargo run --release --example read_joint -- can2
```

确认两臂各自返回关节角后，即可在程序中创建两个 `PiperInterface` 实例分别控制：

```rust,no_run
let left  = piper_arm::PiperInterface::open_socketcan("can1")?;
let right = piper_arm::PiperInterface::open_socketcan("can2")?;
```
