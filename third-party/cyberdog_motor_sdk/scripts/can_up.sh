#!/bin/bash

# CAN接口配置脚本
# 功能：配置can0到can3四个CAN接口，设置比特率为1000000，并优化发送队列长度

# 检查脚本是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请使用sudo或以root权限运行此脚本"
    exit 1
fi

# 加载必要的CAN内核模块
echo "加载CAN内核模块..."
modprobe can 2>/dev/null
modprobe can_raw 2>/dev/null
echo "CAN内核模块加载完成"

# 配置的CAN接口列表
CAN_INTERFACES=("can0" "can1" "can2" "can3")
BITRATE=1000000
TX_QUEUE_LEN=3000  # 设置发送队列长度 [1,7](@ref)

echo "开始配置CAN接口，比特率: $BITRATE, 发送队列长度: $TX_QUEUE_LEN"

# 循环配置每个CAN接口
for interface in "${CAN_INTERFACES[@]}"; do
    echo "正在配置接口 $interface..."
    
    # 检查接口是否存在
    if ! ip link show "$interface" &>/dev/null; then
        echo "警告: 接口 $interface 不存在，跳过配置"
        continue
    fi
    
    # 关闭接口
    if ip link set "$interface" down 2>/dev/null; then
        echo "  ✓ $interface 已关闭"
    else
        echo "  ✗ 无法关闭 $interface"
        continue
    fi
    
    # 设置比特率 [6](@ref)
    if ip link set "$interface" type can bitrate $BITRATE 2>/dev/null; then
        echo "  ✓ $interface 比特率设置为 $BITRATE"
    else
        echo "  ✗ 无法设置 $interface 的比特率"
        continue
    fi
    
    # 设置发送队列长度 [1,7](@ref)
    if ip link set "$interface" txqueuelen $TX_QUEUE_LEN 2>/dev/null; then
        echo "  ✓ $interface 发送队列长度设置为 $TX_QUEUE_LEN"
    else
        echo "  ✗ 无法设置 $interface 的发送队列长度"
        continue
    fi
    
    # 启用接口
    if ip link set "$interface" up 2>/dev/null; then
        echo "  ✓ $interface 已启用"
    else
        echo "  ✗ 无法启用 $interface"
        continue
    fi
    
    echo "  ✓ $interface 配置完成"
    echo
done

# 显示配置结果
echo "CAN接口配置完成，当前状态:"
echo "----------------------------------------"
for interface in "${CAN_INTERFACES[@]}"; do
    if ip link show "$interface" &>/dev/null; then
        state=$(ip -o link show "$interface" | awk '{print $3}')
        bitrate=$(ip -details link show "$interface" 2>/dev/null | grep -o "bitrate [0-9]*" || echo "bitrate unknown")
        qlen=$(ip -details link show "$interface" 2>/dev/null | grep -o "qlen [0-9]*" || echo "qlen unknown")
        echo "  $interface: 状态=$state, $bitrate, $qlen"
    else
        echo "  $interface: 接口不存在"
    fi
done
echo "----------------------------------------"

echo "所有CAN接口配置完成！"