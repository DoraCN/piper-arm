#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# 双臂 CAN 固定命名 + 开机自启 一键配置脚本 (Jetson / 任意 Linux)
#
# 功能:
#   1. 编写 udev 规则: 按 USB 硬件地址把两臂的 CAN 接口固定命名为
#      can_left / can_right (重启/重插不变)
#   2. 立即重命名当前已加载的接口
#   3. 编写 systemd 服务: 开机自动把 can_left / can_right 设为 UP 并配置
#      1 Mbps 波特率
#
# 用法:
#   sudo bash setup_can_dual_arm.sh
#
# 配置项 (按实际环境修改):
#   LEFT_USB  / RIGHT_USB  来自 `ip -details link show canX | grep parentdev`
#                           (去掉末尾的 ":1.0")
# -----------------------------------------------------------------------------
set -euo pipefail

# ---------- 可配置项 ----------
# 左臂 (当前 can2) 的 USB 硬件地址
LEFT_USB="1-2.4.4.2"
# 右臂 (当前 can1) 的 USB 硬件地址
RIGHT_USB="1-2.4.4.3"
# CAN 波特率 (Piper 固定 1000000)
BITRATE="1000000"
# 固定后的接口名
LEFT_IFACE="can_left"
RIGHT_IFACE="can_right"
# udev 规则文件
UDEV_RULES="/etc/udev/rules.d/70-piper-can.rules"
# systemd 服务
SYSTEMD_SERVICE="/etc/systemd/system/can-bringup.service"

# ---------- 权限检查 ----------
if [ "$(id -u)" -ne 0 ]; then
    echo "[ERROR] 请用 root 运行: sudo bash $0" >&2
    exit 1
fi

echo "==> 0/6 确保 gs_usb 内核模块开机自动加载"
# 模块已在 /lib/modules/$(uname -r)/extra/ 且 depmod -a 过，
# 配置开机自动加载，并立即加载一次
cat > /etc/modules-load.d/gs_usb.conf <<EOF
# Piper 双臂 USB-CAN (gs_usb) 开机自动加载
gs_usb
EOF
modprobe can 2>/dev/null || true
modprobe can-dev 2>/dev/null || true
modprobe gs_usb 2>/dev/null || echo "  [WARN] gs_usb 加载失败，请确认已按 docs/installation.md 编译安装 gs_usb.ko"
sleep 1

echo "==> 1/6 写入 udev 规则: $UDEV_RULES"
cat > "$UDEV_RULES" <<EOF
# Piper 双臂 CAN 固定命名 (由 setup_can_dual_arm.sh 生成)
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="gs_usb", KERNELS=="${LEFT_USB}",  NAME="${LEFT_IFACE}"
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="gs_usb", KERNELS=="${RIGHT_USB}", NAME="${RIGHT_IFACE}"
EOF

echo "==> 2/6 重新加载 udev 规则"
udevadm control --reload-rules
udevadm trigger

echo "==> 3/6 立即重命名当前接口 (按 USB 地址匹配)"
rename_iface() {
    local target_usb="$1" new_name="$2" cur_usb cur_iface
    for cur_iface in $(ip -br link show type can | awk '{print $1}'); do
        cur_usb=$(ip -details link show "$cur_iface" 2>/dev/null \
                  | grep -oP 'parentdev \K[^:]+' || true)
        if [ -n "$cur_usb" ] && [ "$cur_usb" = "$target_usb" ] \
           && [ "$cur_iface" != "$new_name" ]; then
            echo "    $cur_iface (USB $cur_usb) -> $new_name"
            ip link set "$cur_iface" down || true
            ip link set "$cur_iface" name "$new_name" || true
        fi
    done
}
rename_iface "$LEFT_USB"  "$LEFT_IFACE"
rename_iface "$RIGHT_USB" "$RIGHT_IFACE"

echo "==> 4/6 写入 systemd 服务: $SYSTEMD_SERVICE"
cat > "$SYSTEMD_SERVICE" <<EOF
[Unit]
Description=Bring up Piper dual-arm CAN interfaces
Wants=network.target
After=network.target systemd-udev-settle.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/bin/udevadm settle --timeout=15
ExecStart=/sbin/ip link set ${LEFT_IFACE}  up type can bitrate ${BITRATE}
ExecStart=/sbin/ip link set ${RIGHT_IFACE} up type can bitrate ${BITRATE}

[Install]
WantedBy=multi-user.target
EOF

echo "==> 5/6 启用并启动服务"
systemctl daemon-reload
systemctl enable can-bringup.service
systemctl restart can-bringup.service

echo
echo "===== 配置完成 ====="
echo "当前 CAN 接口:"
ip -br link show type can
echo
echo "验证 (应均为 UP, bitrate 1000000):"
ip -details link show "$LEFT_IFACE"  | grep -E "state |bitrate"
ip -details link show "$RIGHT_IFACE" | grep -E "state |bitrate"
echo
echo "提示:"
echo "  1. 若接口名未立即变成 can_left/can_right, 重新插拔两个 USB-CAN 转接器即可"
echo "  2. 如左右臂对应关系错误, 修改本脚本顶部 LEFT_USB/RIGHT_USB 后重跑"
