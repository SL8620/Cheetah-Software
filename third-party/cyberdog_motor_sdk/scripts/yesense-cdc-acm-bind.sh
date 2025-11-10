#!/bin/bash

# YESENSE USB CDC ACM 设备绑定脚本
# 此脚本将创建一个 udev 规则，为指定的 YESENSE 设备提供固定的设备节点名称。

RULES_FILE="/etc/udev/rules.d/99-yesense-cdc-acm.rules"
VENDOR_ID="5953"
PRODUCT_ID="5543"
SYMLINK_NAME="yesenseIMU"

echo "正在为 YESENSE USB CDC ACM 设备创建 udev 规则..."
echo "设备ID: ${VENDOR_ID}:${PRODUCT_ID}"
echo "固定符号链接: ${SYMLINK_NAME}"

# 创建 udev 规则内容
# 关键规则：当检测到符合指定 vendor_id 和 product_id 的 CDC-ACM 设备时，为其创建一个固定的符号链接。
RULE_CONTENT="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${VENDOR_ID}\", ATTRS{idProduct}==\"${PRODUCT_ID}\", MODE:=\"0666\", SYMLINK+=\"${SYMLINK_NAME}\""

echo "正在写入规则到 ${RULES_FILE} ..."

# 需要root权限来写入规则文件
sudo bash -c "cat > ${RULES_FILE}" << EOF
# YESENSE USB CDC ACM 固定设备节点规则
# 设备示例: Bus 003 Device 005: ID ${VENDOR_ID}:${PRODUCT_ID} Yesense Co.Ltd YESENSE USB CDC ACM
${RULE_CONTENT}

EOF

if [ $? -eq 0 ]; then
    echo "规则文件写入成功。"
else
    echo "错误：规则文件写入失败。请检查是否有足够的权限。"
    exit 1
fi

echo "重新加载 udev 规则并触发设备重新识别..."
# 重新加载 udev 规则[8](@ref)
sudo udevadm control --reload-rules
# 触发规则立即生效（对已连接的设备）[8](@ref)
sudo udevadm trigger --subsystem-match=tty

echo "完成！"
echo "请尝试重新插拔您的 YESENSE USB CDC ACM 设备。"
echo "设备通常仍会在 /dev/ttyACM* 下出现，但同时会有一个固定的符号链接指向它："
echo -e "固定设备节点: \033[1;32m/dev/${SYMLINK_NAME}\033[0m"
echo "您可以在程序中使用 /dev/${SYMLINK_NAME} 来访问设备，无需再关心变化的 ttyACM 编号。"

# 提示用户如何验证
echo -e "\n--- 验证步骤 ---"
echo "1. 保持设备连接，或重新插入。"
echo "2. 运行:   ls -l /dev/ttyYESENSE"
echo "3. 如果输出类似以下内容，则说明规则已生效："
echo "   lrwxrwxrwx 1 root root 7 [日期] /dev/ttyYESENSE -> ttyACM6"