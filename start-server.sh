#!/bin/bash

# 激光雷达树木检测系统启动脚本
# 适用于Ubuntu/Linux系统

echo "🌲 激光雷达树木检测系统启动脚本"
echo "=================================="

# 检查Node.js是否安装
if ! command -v node &> /dev/null; then
    echo "❌ Node.js未安装，正在安装..."
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get install -y nodejs
    echo "✅ Node.js安装完成"
else
    echo "✅ Node.js已安装: $(node --version)"
fi

# 检查npm是否安装
if ! command -v npm &> /dev/null; then
    echo "❌ npm未安装，正在安装..."
    sudo apt-get install -y npm
    echo "✅ npm安装完成"
else
    echo "✅ npm已安装: $(npm --version)"
fi

# 安装依赖
echo "📦 正在安装项目依赖..."
npm install

# 创建数据目录
echo "📁 创建数据存储目录..."
mkdir -p data

# 设置权限
echo "🔐 设置文件权限..."
chmod +x websocket-server.js

# 启动服务器
echo "🚀 启动WebSocket服务器..."
echo "服务器将在以下地址启动："
echo "  - HTTP服务器: http://localhost:8080"
echo "  - WebSocket服务器: ws://localhost:8080/ws"
echo ""
echo "按 Ctrl+C 停止服务器"
echo ""

# 启动服务器
npm start
