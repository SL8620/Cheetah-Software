#!/bin/bash

# CAN接口批量关闭脚本
# 功能：自动识别并关闭系统中所有已启用的CAN接口

# 检查脚本是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请使用sudo或以root权限运行此脚本"
    exit 1
fi

echo "正在扫描系统中的CAN接口..."

# 获取所有CAN接口列表
# 使用grep查找类型为CAN的接口，然后用awk提取接口名称
CAN_INTERFACES=$(ip link show type can 2>/dev/null | grep -oE 'can[0-9]+' | sort -u)

if [ -z "$CAN_INTERFACES" ]; then
    echo "未找到任何CAN接口"
    exit 0
fi

echo "找到以下CAN接口: $(echo $CAN_INTERFACES | tr '\n' ' ')"
echo

# 循环关闭每个CAN接口
for interface in $CAN_INTERFACES; do
    echo "正在关闭接口 $interface..."
    
    # 检查接口当前状态
    if ip link show "$interface" | grep -q "state UP"; then
        # 关闭接口[1,3,6](@ref)
        if ip link set "$interface" down 2>/dev/null; then
            echo "  ✓ $interface 已成功关闭"
        else
            echo "  ✗ 无法关闭 $interface"
        fi
    else
        echo "  - $interface 已处于关闭状态，无需操作"
    fi
done

echo
echo "操作完成，当前CAN接口状态:"
echo "----------------------------------------"

# 显示最终状态
for interface in $CAN_INTERFACES; do
    state=$(ip -o link show "$interface" 2>/dev/null | awk '{print $3}')
    if [ -n "$state" ]; then
        echo "  $interface: $state"
    else
        echo "  $interface: 接口不存在或无法访问"
    fi
done
echo "----------------------------------------"