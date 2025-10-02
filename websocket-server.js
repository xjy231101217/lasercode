/**
 * 激光雷达树木检测系统 - WebSocket服务器
 * 用于实时数据同步和多设备协作
 * 
 * @author AI Assistant
 * @version 1.0.0
 * @description 提供WebSocket服务，支持实时数据同步、设备状态共享
 */

const WebSocket = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

class LidarWebSocketServer {
    constructor(port = 8080) {
        this.port = port;
        this.clients = new Map(); // 存储连接的客户端
        this.dataHistory = []; // 存储历史数据
        this.maxHistorySize = 1000; // 最大历史记录数
        
        // 创建HTTP服务器
        this.server = http.createServer((req, res) => {
            this.handleHttpRequest(req, res);
        });
        
        // 创建WebSocket服务器
        this.wss = new WebSocket.Server({ 
            server: this.server,
            path: '/ws'
        });
        
        this.setupWebSocketHandlers();
        this.setupDataPersistence();
    }
    
    /**
     * 处理HTTP请求（用于静态文件服务）
     */
    handleHttpRequest(req, res) {
        let filePath = req.url === '/' ? '/index.html' : req.url;
        filePath = path.join(__dirname, filePath);
        
        // 检查文件是否存在
        fs.access(filePath, fs.constants.F_OK, (err) => {
            if (err) {
                res.writeHead(404, { 'Content-Type': 'text/plain' });
                res.end('File not found');
                return;
            }
            
            // 根据文件扩展名设置Content-Type
            const ext = path.extname(filePath);
            const contentTypes = {
                '.html': 'text/html',
                '.js': 'application/javascript',
                '.css': 'text/css',
                '.json': 'application/json'
            };
            
            const contentType = contentTypes[ext] || 'text/plain';
            
            fs.readFile(filePath, (err, data) => {
                if (err) {
                    res.writeHead(500, { 'Content-Type': 'text/plain' });
                    res.end('Internal server error');
                    return;
                }
                
                res.writeHead(200, { 'Content-Type': contentType });
                res.end(data);
            });
        });
    }
    
    /**
     * 设置WebSocket事件处理器
     */
    setupWebSocketHandlers() {
        this.wss.on('connection', (ws, req) => {
            const clientId = this.generateClientId();
            const clientInfo = {
                id: clientId,
                ws: ws,
                ip: req.socket.remoteAddress,
                connectedAt: new Date(),
                lastActivity: new Date(),
                deviceType: 'unknown'
            };
            
            this.clients.set(clientId, clientInfo);
            console.log(`[${new Date().toISOString()}] 客户端连接: ${clientId} (${clientInfo.ip})`);
            
            // 发送欢迎消息和历史数据
            this.sendToClient(ws, {
                type: 'welcome',
                clientId: clientId,
                message: '连接成功',
                timestamp: new Date().toISOString()
            });
            
            // 发送历史数据
            if (this.dataHistory.length > 0) {
                this.sendToClient(ws, {
                    type: 'history',
                    data: this.dataHistory.slice(-50), // 发送最近50条记录
                    timestamp: new Date().toISOString()
                });
            }
            
            // 处理客户端消息
            ws.on('message', (message) => {
                this.handleClientMessage(clientId, message);
            });
            
            // 处理客户端断开连接
            ws.on('close', () => {
                console.log(`[${new Date().toISOString()}] 客户端断开: ${clientId}`);
                this.clients.delete(clientId);
                this.broadcastClientList();
            });
            
            // 处理错误
            ws.on('error', (error) => {
                console.error(`[${new Date().toISOString()}] 客户端错误 ${clientId}:`, error);
            });
            
            // 发送当前客户端列表
            this.broadcastClientList();
        });
    }
    
    /**
     * 处理客户端消息
     */
    handleClientMessage(clientId, message) {
        try {
            const data = JSON.parse(message);
            const client = this.clients.get(clientId);
            
            if (client) {
                client.lastActivity = new Date();
                
                switch (data.type) {
                    case 'device_info':
                        client.deviceType = data.deviceType || 'unknown';
                        client.deviceName = data.deviceName || 'Unknown Device';
                        console.log(`[${new Date().toISOString()}] 设备信息更新: ${clientId} - ${client.deviceName}`);
                        this.broadcastClientList();
                        break;
                        
                    case 'scan_data':
                        this.handleScanData(clientId, data);
                        break;
                        
                    case 'tree_data':
                        this.handleTreeData(clientId, data);
                        break;
                        
                    case 'height_data':
                        this.handleHeightData(clientId, data);
                        break;
                        
                    case 'status_update':
                        this.handleStatusUpdate(clientId, data);
                        break;
                        
                    case 'ping':
                        this.sendToClient(client.ws, {
                            type: 'pong',
                            timestamp: new Date().toISOString()
                        });
                        break;
                        
                    default:
                        console.log(`[${new Date().toISOString()}] 未知消息类型: ${data.type}`);
                }
            }
        } catch (error) {
            console.error(`[${new Date().toISOString()}] 消息解析错误:`, error);
        }
    }
    
    /**
     * 处理扫描数据
     */
    handleScanData(clientId, data) {
        const scanData = {
            type: 'scan_data',
            clientId: clientId,
            timestamp: new Date().toISOString(),
            data: {
                points: data.points,
                treeCount: data.treeCount,
                scanTime: data.scanTime,
                avgDistance: data.avgDistance,
                maxDistance: data.maxDistance
            }
        };
        
        this.addToHistory(scanData);
        this.broadcastToOthers(clientId, scanData);
    }
    
    /**
     * 处理树木数据
     */
    handleTreeData(clientId, data) {
        const treeData = {
            type: 'tree_data',
            clientId: clientId,
            timestamp: new Date().toISOString(),
            data: {
                trees: data.trees,
                treeCount: data.treeCount,
                avgDiameter: data.avgDiameter
            }
        };
        
        this.addToHistory(treeData);
        this.broadcastToOthers(clientId, treeData);
    }
    
    /**
     * 处理高度数据
     */
    handleHeightData(clientId, data) {
        const heightData = {
            type: 'height_data',
            clientId: clientId,
            timestamp: new Date().toISOString(),
            data: {
                currentHeight: data.currentHeight,
                avgHeight: data.avgHeight,
                heightCount: data.heightCount
            }
        };
        
        this.addToHistory(heightData);
        this.broadcastToOthers(clientId, heightData);
    }
    
    /**
     * 处理状态更新
     */
    handleStatusUpdate(clientId, data) {
        const statusData = {
            type: 'status_update',
            clientId: clientId,
            timestamp: new Date().toISOString(),
            data: {
                lidarConnected: data.lidarConnected,
                stp23lConnected: data.stp23lConnected,
                isScanning: data.isScanning,
                isMeasuring: data.isMeasuring
            }
        };
        
        this.broadcastToOthers(clientId, statusData);
    }
    
    /**
     * 添加到历史记录
     */
    addToHistory(data) {
        this.dataHistory.push(data);
        
        // 限制历史记录大小
        if (this.dataHistory.length > this.maxHistorySize) {
            this.dataHistory = this.dataHistory.slice(-this.maxHistorySize);
        }
    }
    
    /**
     * 广播给其他客户端（排除发送者）
     */
    broadcastToOthers(senderId, data) {
        this.clients.forEach((client, clientId) => {
            if (clientId !== senderId && client.ws.readyState === WebSocket.OPEN) {
                this.sendToClient(client.ws, data);
            }
        });
    }
    
    /**
     * 广播客户端列表
     */
    broadcastClientList() {
        const clientList = Array.from(this.clients.values()).map(client => ({
            id: client.id,
            deviceType: client.deviceType,
            deviceName: client.deviceName || 'Unknown Device',
            connectedAt: client.connectedAt,
            lastActivity: client.lastActivity
        }));
        
        const message = {
            type: 'client_list',
            clients: clientList,
            timestamp: new Date().toISOString()
        };
        
        this.broadcast(message);
    }
    
    /**
     * 广播消息给所有客户端
     */
    broadcast(data) {
        this.clients.forEach((client) => {
            if (client.ws.readyState === WebSocket.OPEN) {
                this.sendToClient(client.ws, data);
            }
        });
    }
    
    /**
     * 发送消息给指定客户端
     */
    sendToClient(ws, data) {
        if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(data));
        }
    }
    
    /**
     * 生成客户端ID
     */
    generateClientId() {
        return 'client_' + Math.random().toString(36).substr(2, 9);
    }
    
    /**
     * 设置数据持久化
     */
    setupDataPersistence() {
        // 每5分钟保存一次数据
        setInterval(() => {
            this.saveDataToFile();
        }, 5 * 60 * 1000);
        
        // 启动时加载数据
        this.loadDataFromFile();
    }
    
    /**
     * 保存数据到文件
     */
    saveDataToFile() {
        const dataFile = path.join(__dirname, 'data', 'lidar_data.json');
        const dataDir = path.dirname(dataFile);
        
        // 确保目录存在
        if (!fs.existsSync(dataDir)) {
            fs.mkdirSync(dataDir, { recursive: true });
        }
        
        const data = {
            timestamp: new Date().toISOString(),
            history: this.dataHistory,
            clients: Array.from(this.clients.keys())
        };
        
        fs.writeFile(dataFile, JSON.stringify(data, null, 2), (err) => {
            if (err) {
                console.error('保存数据失败:', err);
            } else {
                console.log(`[${new Date().toISOString()}] 数据已保存到文件`);
            }
        });
    }
    
    /**
     * 从文件加载数据
     */
    loadDataFromFile() {
        const dataFile = path.join(__dirname, 'data', 'lidar_data.json');
        
        fs.readFile(dataFile, 'utf8', (err, data) => {
            if (err) {
                console.log('没有找到历史数据文件，将创建新的数据存储');
                return;
            }
            
            try {
                const parsedData = JSON.parse(data);
                this.dataHistory = parsedData.history || [];
                console.log(`[${new Date().toISOString()}] 已加载 ${this.dataHistory.length} 条历史记录`);
            } catch (error) {
                console.error('加载历史数据失败:', error);
            }
        });
    }
    
    /**
     * 启动服务器
     */
    start() {
        this.server.listen(this.port, () => {
            console.log(`[${new Date().toISOString()}] 激光雷达WebSocket服务器已启动`);
            console.log(`[${new Date().toISOString()}] HTTP服务器: http://localhost:${this.port}`);
            console.log(`[${new Date().toISOString()}] WebSocket服务器: ws://localhost:${this.port}/ws`);
            console.log(`[${new Date().toISOString()}] 等待客户端连接...`);
        });
    }
    
    /**
     * 停止服务器
     */
    stop() {
        console.log(`[${new Date().toISOString()}] 正在停止服务器...`);
        
        // 保存数据
        this.saveDataToFile();
        
        // 关闭所有客户端连接
        this.clients.forEach((client) => {
            client.ws.close();
        });
        
        // 关闭服务器
        this.server.close(() => {
            console.log(`[${new Date().toISOString()}] 服务器已停止`);
        });
    }
}

// 启动服务器
const server = new LidarWebSocketServer(8080);

// 优雅关闭
process.on('SIGINT', () => {
    console.log('\n收到中断信号，正在关闭服务器...');
    server.stop();
    process.exit(0);
});

process.on('SIGTERM', () => {
    console.log('\n收到终止信号，正在关闭服务器...');
    server.stop();
    process.exit(0);
});

server.start();

module.exports = LidarWebSocketServer;
