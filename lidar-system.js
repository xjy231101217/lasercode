/**
 * 激光雷达树木检测系统
 * 基于HOKUYO URG-04LX-UG01的Web版本实现
 * 完全按照MATLAB代码逻辑实现
 * 
 * @author AI Assistant
 * @version 1.0.0
 * @description 实时激光雷达扫描、数据处理、树木检测和可视化系统
 */

// STP-23L传感器常量定义
const STP23L_CONSTANTS = {
    // 串口配置
    BAUD_RATE: 230400,
    DATA_BITS: 8,
    STOP_BITS: 1,
    PARITY: 'none',
    FLOW_CONTROL: 'none',
    
    // 数据包配置
    PACKET_SIZE: 195,
    START_BYTE: 0xAA,
    DATA_START_INDEX: 10,
    DATA_INTERVAL: 15,
    
    // 测量参数
    MAX_DISTANCE: 30000, // 30米
    MIN_DISTANCE: 20,    // 2厘米
    MEASUREMENT_INTERVAL: 100, // 100ms
    
    // 数据字段索引
    DATA_FIELDS: {
        DISTANCE_LOW: 0,
        DISTANCE_HIGH: 1,
        NOISE_LOW: 2,
        NOISE_HIGH: 3,
        PEAK_1: 4,
        PEAK_2: 5,
        PEAK_3: 6,
        PEAK_4: 7,
        CONFIDENCE: 8,
        INTG_1: 9,
        INTG_2: 10,
        INTG_3: 11,
        INTG_4: 12,
        REFTOF_LOW: 13,
        REFTOF_HIGH: 14
    }
};

// 常量定义
const LIDAR_CONSTANTS = {
    // 串口配置
    BAUD_RATE: 115200,
    DATA_BITS: 8,
    STOP_BITS: 1,
    PARITY: 'none',
    FLOW_CONTROL: 'none',
    
    // 数据包配置
    EXPECTED_DATA_SIZE: 2134,
    MAX_BUFFER_SIZE: 10000,
    
    // 扫描参数
    START_ANGLE: -120,
    END_ANGLE: 120,
    TOTAL_POINTS: 682,
    MAX_RANGE: 5500,
    MIN_RANGE: 20,
    
    // 协议命令
    COMMANDS: {
        SCIP: 'SCIP2.0',
        VERSION: 'VV',
        LASER_ON: 'BM',
        SCAN: 'GD0044072500',
        LASER_OFF: 'QT'
    },
    
    // 延迟时间
    DELAYS: {
        INIT_RESPONSE: 300,
        SCAN_RESPONSE: 300,
        READ_TIMEOUT: 50,
        SCAN_INTERVAL: 100
    }
};

/**
 * STP-23L单点测距传感器类
 * 用于测量装置距离地面的垂直高度
 */
class STP23LSensor {
    constructor() {
        // 连接状态
        this.isConnected = false;
        this.isMeasuring = false;
        
        // 串口相关
        this.port = null;
        this.reader = null;
        
        // 测量数据
        this.currentHeight = 0;
        this.heightHistory = [];
        this.measurementCount = 0;
        
        // 定时器
        this.measurementInterval = null;
        
        // 阈值设置
        this.heightThreshold = 1000; // 默认1米
        
        this.log('STP-23L传感器类已初始化', 'info');
    }
    
    /**
     * 连接STP-23L传感器
     */
    async connect() {
        try {
            this.log('正在连接STP-23L传感器...', 'info');
            
            // 请求串口访问权限
            this.port = await navigator.serial.requestPort();
            
            // 打开串口
            await this.port.open({
                baudRate: STP23L_CONSTANTS.BAUD_RATE,
                dataBits: STP23L_CONSTANTS.DATA_BITS,
                stopBits: STP23L_CONSTANTS.STOP_BITS,
                parity: STP23L_CONSTANTS.PARITY,
                flowControl: STP23L_CONSTANTS.FLOW_CONTROL
            });
            
            this.isConnected = true;
            this.updateStatus('已连接', 'connected');
            this.updateButtons();
            this.log('STP-23L传感器连接成功！', 'success');
            
        } catch (error) {
            this.handleConnectionError(error);
        }
    }
    
    /**
     * 断开STP-23L传感器连接
     */
    async disconnect() {
        try {
            this.log('正在断开STP-23L传感器连接...', 'info');
            
            // 停止测量
            this.stopMeasuring();
            
            // 关闭串口
            if (this.port) {
                await this.port.close();
                this.port = null;
            }
            
            // 清理状态
            this.cleanup();
            
            this.updateStatus('已断开', 'disconnected');
            this.updateButtons();
            this.log('STP-23L传感器已断开连接', 'info');
            
        } catch (error) {
            this.log(`断开连接失败: ${error.message}`, 'error');
        }
    }
    
    /**
     * 开始测量
     */
    startMeasuring() {
        if (!this.isConnected) {
            this.log('STP-23L传感器未连接', 'warning');
            return;
        }
        
        this.isMeasuring = true;
        this.updateButtons();
        this.log('开始STP-23L高度测量...', 'info');
        
        // 开始连续测量
        this.measurementInterval = setInterval(async () => {
            if (this.isMeasuring) {
                await this.performMeasurement();
            }
        }, STP23L_CONSTANTS.MEASUREMENT_INTERVAL);
    }
    
    /**
     * 停止测量
     */
    stopMeasuring() {
        this.isMeasuring = false;
        if (this.measurementInterval) {
            clearInterval(this.measurementInterval);
            this.measurementInterval = null;
        }
        this.updateButtons();
        this.log('STP-23L测量已停止', 'info');
    }
    
    /**
     * 执行单次测量
     */
    async performMeasurement() {
        if (!this.port) {
            this.log('STP-23L传感器未连接', 'error');
            return;
        }
        
        try {
            // 读取串口数据
            const data = await this.readSerialData();
            
            if (data && data.length >= STP23L_CONSTANTS.PACKET_SIZE) {
                // 解析数据
                const measurements = this.parseData(data);
                
                if (measurements && measurements.length > 0) {
                    // 使用第一个测量点的距离数据
                    const measurement = measurements[0];
                    this.currentHeight = measurement.distance;
                    
                    // 添加到历史记录
                    this.heightHistory.push({
                        height: this.currentHeight,
                        timestamp: Date.now(),
                        noise: measurement.noise,
                        confidence: measurement.confidence
                    });
                    
                    this.measurementCount++;
                    
                    // 更新显示
                    this.updateDisplay();
                    
                    // 更新高度图表
                    this.updateHeightChart();
                    
                    // 检查高度变化
                    this.checkHeightChange();
                }
            }
            
        } catch (error) {
            this.log(`STP-23L测量失败: ${error.message}`, 'error');
        }
    }
    
    /**
     * 读取串口数据
     */
    async readSerialData() {
        if (!this.port) return null;
        
        try {
            // 释放之前的读取器
            if (this.reader) {
                try {
                    this.reader.releaseLock();
                } catch (e) {
                    // 忽略释放错误
                }
                this.reader = null;
            }
            
            // 获取新的读取器
            this.reader = this.port.readable.getReader();
            
            // 读取数据包
            const { value, done } = await this.reader.read();
            if (done) return null;
            
            return new Uint8Array(value);
            
        } catch (error) {
            this.log(`读取STP-23L数据失败: ${error.message}`, 'error');
            return null;
        } finally {
            // 释放读取器
            if (this.reader) {
                try {
                    this.reader.releaseLock();
                } catch (e) {
                    // 忽略释放错误
                }
                this.reader = null;
            }
        }
    }
    
    /**
     * 解析STP-23L数据包
     */
    parseData(data) {
        try {
            // 检查起始符
            if (data[0] !== STP23L_CONSTANTS.START_BYTE) {
                this.log('STP-23L数据包起始符错误', 'warning');
                return null;
            }
            
            const measurements = [];
            const fields = STP23L_CONSTANTS.DATA_FIELDS;
            
            // 从第11个字节开始解析测量数据
            for (let i = STP23L_CONSTANTS.DATA_START_INDEX; i < data.length; i += STP23L_CONSTANTS.DATA_INTERVAL) {
                if (i + STP23L_CONSTANTS.DATA_INTERVAL < data.length) {
                    // 解析距离数据（高字节在前，低字节在后）
                    const distanceHigh = data[i + fields.DISTANCE_HIGH];
                    const distanceLow = data[i + fields.DISTANCE_LOW];
                    const distance = (distanceHigh << 8) | distanceLow;
                    
                    // 解析环境噪声
                    const noiseHigh = data[i + fields.NOISE_HIGH];
                    const noiseLow = data[i + fields.NOISE_LOW];
                    const noise = (noiseHigh << 8) | noiseLow;
                    
                    // 解析接收强度信息（4字节）
                    const peak1 = data[i + fields.PEAK_1];
                    const peak2 = data[i + fields.PEAK_2];
                    const peak3 = data[i + fields.PEAK_3];
                    const peak4 = data[i + fields.PEAK_4];
                    const peak = (peak1 << 24) | (peak2 << 16) | (peak3 << 8) | peak4;
                    
                    // 解析置信度
                    const confidence = data[i + fields.CONFIDENCE];
                    
                    // 解析积分次数（4字节）
                    const intg1 = data[i + fields.INTG_1];
                    const intg2 = data[i + fields.INTG_2];
                    const intg3 = data[i + fields.INTG_3];
                    const intg4 = data[i + fields.INTG_4];
                    const intg = (intg1 << 24) | (intg2 << 16) | (intg3 << 8) | intg4;
                    
                    // 解析温度表征值
                    const reftofHigh = data[i + fields.REFTOF_HIGH];
                    const reftofLow = data[i + fields.REFTOF_LOW];
                    const reftof = (reftofHigh << 8) | reftofLow;
                    
                    measurements.push({
                        distance: distance,
                        noise: noise,
                        peak: peak,
                        confidence: confidence,
                        intg: intg,
                        reftof: reftof
                    });
                }
            }
            
            return measurements;
            
        } catch (error) {
            this.log(`STP-23L数据解析失败: ${error.message}`, 'error');
            return null;
        }
    }
    
    /**
     * 更新高度图表
     */
    updateHeightChart() {
        if (window.lidarSystem && typeof window.lidarSystem.drawHeightChart === 'function') {
            window.lidarSystem.drawHeightChart(this.heightHistory);
            // 调试信息
            if (this.heightHistory.length > 0) {
                console.log(`[STP-23L] 更新图表: ${this.heightHistory.length} 个数据点`);
            }
        } else {
            console.warn('[STP-23L] 主系统未初始化或图表绘制方法不可用');
        }
    }
    
    /**
     * 更新显示
     */
    updateDisplay() {
        // 更新当前高度
        const currentHeightElement = document.getElementById('currentHeight');
        if (currentHeightElement) {
            currentHeightElement.textContent = Math.round(this.currentHeight);
        }
        
        // 更新平均高度
        const avgHeightElement = document.getElementById('avgHeight');
        if (avgHeightElement && this.heightHistory.length > 0) {
            const avgHeight = this.heightHistory.reduce((sum, h) => sum + h.height, 0) / this.heightHistory.length;
            avgHeightElement.textContent = Math.round(avgHeight);
        }
        
        // 更新测量次数
        const heightCountElement = document.getElementById('heightCount');
        if (heightCountElement) {
            heightCountElement.textContent = this.measurementCount;
        }
    }
    
    /**
     * 检查高度变化
     */
    checkHeightChange() {
        if (this.heightHistory.length < 2) return;
        
        const current = this.heightHistory[this.heightHistory.length - 1];
        const previous = this.heightHistory[this.heightHistory.length - 2];
        
        const heightChange = Math.abs(current.height - previous.height);
        
        if (heightChange > this.heightThreshold) {
            this.log(`高度变化警告: ${Math.round(heightChange)}mm (当前: ${Math.round(current.height)}mm, 上次: ${Math.round(previous.height)}mm)`, 'warning');
        }
    }
    
    /**
     * 清除高度数据
     */
    clearHeightData() {
        this.heightHistory = [];
        this.measurementCount = 0;
        this.currentHeight = 0;
        this.updateDisplay();
        this.log('STP-23L高度数据已清除', 'info');
    }
    
    /**
     * 处理连接错误
     */
    handleConnectionError(error) {
        this.log(`STP-23L连接失败: ${error.message}`, 'error');
        this.isConnected = false;
        this.updateStatus('连接失败', 'disconnected');
        this.updateButtons();
        this.cleanup();
    }
    
    /**
     * 清理资源
     */
    cleanup() {
        // 释放读取器
        if (this.reader) {
            try {
                this.reader.releaseLock();
            } catch (e) {
                // 忽略释放错误
            }
            this.reader = null;
        }
        
        // 重置状态
        this.isConnected = false;
        this.isMeasuring = false;
    }
    
    /**
     * 更新状态显示
     */
    updateStatus(text, type) {
        const statusText = document.getElementById('stp23lStatusText');
        const statusDot = document.getElementById('stp23lStatusDot');
        
        if (statusText) {
            statusText.textContent = text;
        }
        
        if (statusDot) {
            statusDot.className = `status-dot status-${type}`;
        }
    }
    
    /**
     * 更新按钮状态
     */
    updateButtons() {
        const connectBtn = document.getElementById('stp23lConnectBtn');
        const disconnectBtn = document.getElementById('stp23lDisconnectBtn');
        const startBtn = document.getElementById('stp23lStartBtn');
        const stopBtn = document.getElementById('stp23lStopBtn');
        
        if (connectBtn) connectBtn.disabled = this.isConnected;
        if (disconnectBtn) disconnectBtn.disabled = !this.isConnected;
        if (startBtn) startBtn.disabled = !this.isConnected || this.isMeasuring;
        if (stopBtn) stopBtn.disabled = !this.isConnected || !this.isMeasuring;
    }
    
    /**
     * 日志记录
     */
    log(message, type = 'info') {
        // 使用主系统的日志功能
        if (window.lidarSystem && typeof window.lidarSystem.log === 'function') {
            window.lidarSystem.log(`[STP-23L] ${message}`, type);
        } else {
            console.log(`[STP-23L] ${message}`);
        }
    }
}

class LidarSystem {
    /**
     * 初始化激光雷达系统
     */
    constructor() {
        // 系统状态
        this.isConnected = false;
        this.isScanning = false;
        this.scanData = [];
        this.trees = [];
        
        // STP-23L传感器实例
        this.stp23lSensor = new STP23LSensor();
        
        // 高度图表相关
        this.heightChart = null;
        this.heightChartCtx = null;
        
        // DOM元素引用
        this.canvas = null;
        this.ctx = null;
        
        // 定时器和动画
        this.scanInterval = null;
        this.scanAnimationFrame = null;
        this.currentScanAngle = LIDAR_CONSTANTS.START_ANGLE;
        
        // 串口相关
        this.port = null;
        this.reader = null;
        
        // 视图控制
        this.viewSettings = {
            showGrid: true,
            showLabels: true,
            showScanLine: true,
            zoom: 1.0,
            offsetX: 0,
            offsetY: 0,
            rotation: 0 // 旋转角度（度），确保初始化为0
        };
        
        // 扫描参数（使用常量）
        this.scanParams = {
            startAngle: LIDAR_CONSTANTS.START_ANGLE,
            endAngle: LIDAR_CONSTANTS.END_ANGLE,
            numPoints: LIDAR_CONSTANTS.TOTAL_POINTS,
            maxRange: LIDAR_CONSTANTS.MAX_RANGE,
            minRange: LIDAR_CONSTANTS.MIN_RANGE
        };
        
        // 检测参数
        this.detectionParams = {
            eps: 100,        // DBSCAN聚类半径
            minPoints: 5,    // 最小点数
            minRadius: 50,   // 最小圆半径
            maxRadius: 500   // 最大圆半径
        };
        
        this.init();
    }
    
    init() {
        this.setupCanvas();
        this.setupHeightChart();
        this.setupEventListeners();
        this.updateButtons();
        this.updateVisualization();
        
        // 确保旋转角度正确初始化
        console.log('系统初始化 - 旋转角度:', this.viewSettings.rotation);
        this.log(`激光雷达系统已初始化 - 旋转角度: ${this.viewSettings.rotation}°`, 'info');
    }
    
    setupCanvas() {
        this.canvas = document.getElementById('scanCanvas');
        if (!this.canvas) {
            console.error('Canvas元素未找到: scanCanvas');
            throw new Error('Canvas元素未找到');
        }
        
        this.ctx = this.canvas.getContext('2d');
        if (!this.ctx) {
            console.error('无法获取Canvas 2D上下文');
            throw new Error('无法获取Canvas 2D上下文');
        }
        
        // 设置画布大小
        const rect = this.canvas.getBoundingClientRect();
        this.canvas.width = rect.width * window.devicePixelRatio;
        this.canvas.height = rect.height * window.devicePixelRatio;
        this.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        
        console.log('Canvas设置完成:', this.canvas);
    }
    
    setupHeightChart() {
        // 延迟初始化，确保DOM完全加载
        setTimeout(() => {
            this.heightChart = document.getElementById('heightChart');
            if (!this.heightChart) {
                console.error('高度图表Canvas元素未找到: heightChart');
                return;
            }
            
            this.heightChartCtx = this.heightChart.getContext('2d');
            if (!this.heightChartCtx) {
                console.error('无法获取高度图表Canvas 2D上下文');
                return;
            }
            
            // 初始化图表
            this.drawHeightChart();
            console.log('高度图表设置完成:', this.heightChart);
        }, 100);
    }
    
    setupEventListeners() {
        console.log('开始设置事件监听器...');
        
        // 连接控制
        const connectBtn = document.getElementById('connectBtn');
        const disconnectBtn = document.getElementById('disconnectBtn');
        
        if (connectBtn) {
            connectBtn.addEventListener('click', () => this.connect());
            console.log('连接按钮事件监听器已设置');
        } else {
            console.error('连接按钮未找到');
        }
        
        if (disconnectBtn) {
            disconnectBtn.addEventListener('click', () => this.disconnect());
            console.log('断开按钮事件监听器已设置');
        } else {
            console.error('断开按钮未找到');
        }
        
        // 扫描控制
        const startScanBtn = document.getElementById('startScanBtn');
        const pauseScanBtn = document.getElementById('pauseScanBtn');
        const singleScanBtn = document.getElementById('singleScanBtn');
        
        if (startScanBtn) {
            startScanBtn.addEventListener('click', () => this.startScanning());
            console.log('开始扫描按钮事件监听器已设置');
        } else {
            console.error('开始扫描按钮未找到');
        }
        
        if (pauseScanBtn) {
            pauseScanBtn.addEventListener('click', () => this.pauseScanning());
            console.log('暂停扫描按钮事件监听器已设置');
        } else {
            console.error('暂停扫描按钮未找到');
        }
        
        if (singleScanBtn) {
            singleScanBtn.addEventListener('click', () => this.singleScan());
            console.log('单次扫描按钮事件监听器已设置');
        } else {
            console.error('单次扫描按钮未找到');
        }
        
        // 树木检测
        const detectTreesBtn = document.getElementById('detectTreesBtn');
        const clearTreesBtn = document.getElementById('clearTreesBtn');
        const debugBtn = document.getElementById('debugBtn');
        const calibrateBtn = document.getElementById('calibrateBtn');
        
        if (detectTreesBtn) {
            detectTreesBtn.addEventListener('click', () => this.detectTrees());
            console.log('检测树木按钮事件监听器已设置');
        } else {
            console.error('检测树木按钮未找到');
        }
        
        if (clearTreesBtn) {
            clearTreesBtn.addEventListener('click', () => this.clearResults());
            console.log('清除结果按钮事件监听器已设置');
        } else {
            console.error('清除结果按钮未找到');
        }
        
        if (debugBtn) {
            debugBtn.addEventListener('click', () => this.debugData());
            console.log('调试数据按钮事件监听器已设置');
        } else {
            console.error('调试数据按钮未找到');
        }
        
        if (calibrateBtn) {
            calibrateBtn.addEventListener('click', () => this.calibrateDistance());
            console.log('校准测试按钮事件监听器已设置');
        } else {
            console.error('校准测试按钮未找到');
        }
        
        // 数据管理
        const importBtn = document.getElementById('importBtn');
        const exportBtn = document.getElementById('exportBtn');
        const exportTreesBtn = document.getElementById('exportTreesBtn');
        
        if (importBtn) {
            importBtn.addEventListener('click', () => this.importData());
            console.log('导入数据按钮事件监听器已设置');
        } else {
            console.error('导入数据按钮未找到');
        }
        
        if (exportBtn) {
            exportBtn.addEventListener('click', () => this.exportData());
            console.log('导出数据按钮事件监听器已设置');
        } else {
            console.error('导出数据按钮未找到');
        }
        
        if (exportTreesBtn) {
            exportTreesBtn.addEventListener('click', () => this.exportTrees());
            console.log('导出树木信息按钮事件监听器已设置');
        } else {
            console.error('导出树木信息按钮未找到');
        }
        
        // 参数调整
        const epsilonInput = document.getElementById('epsilon');
        const minPtsInput = document.getElementById('minPts');
        const minRadiusInput = document.getElementById('minRadius');
        const maxRadiusInput = document.getElementById('maxRadius');
        
        if (epsilonInput) {
            epsilonInput.addEventListener('input', (e) => {
                this.detectionParams.eps = parseInt(e.target.value);
            });
            console.log('聚类半径输入框事件监听器已设置');
        } else {
            console.error('聚类半径输入框未找到');
        }
        
        if (minPtsInput) {
            minPtsInput.addEventListener('input', (e) => {
                this.detectionParams.minPoints = parseInt(e.target.value);
            });
            console.log('最小点数输入框事件监听器已设置');
        } else {
            console.error('最小点数输入框未找到');
        }
        
        if (minRadiusInput) {
            minRadiusInput.addEventListener('input', (e) => {
                this.detectionParams.minRadius = parseInt(e.target.value);
            });
            console.log('最小半径输入框事件监听器已设置');
        } else {
            console.error('最小半径输入框未找到');
        }
        
        if (maxRadiusInput) {
            maxRadiusInput.addEventListener('input', (e) => {
                this.detectionParams.maxRadius = parseInt(e.target.value);
            });
            console.log('最大半径输入框事件监听器已设置');
        } else {
            console.error('最大半径输入框未找到');
        }
        
        // 视图控制 - 使用事件委托确保按钮点击能被捕获
        const self = this; // 保存this引用
        document.addEventListener('click', function(event) {
            console.log('点击事件捕获:', event.target.id, event.target);
            if (event.target.id === 'resetViewBtn') {
                console.log('事件委托: 重置视图按钮被点击');
                self.log('重置视图按钮被点击 (事件委托)', 'info');
                self.resetView();
            } else if (event.target.id === 'toggleGridBtn') {
                console.log('事件委托: 网格按钮被点击');
                self.log('网格按钮被点击 (事件委托)', 'info');
                self.toggleGrid();
            } else if (event.target.id === 'toggleLabelsBtn') {
                console.log('事件委托: 标签按钮被点击');
                self.log('标签按钮被点击 (事件委托)', 'info');
                self.toggleLabels();
            } else if (event.target.id === 'rotateViewBtn') {
                console.log('事件委托: 旋转按钮被点击');
                self.log('旋转按钮被点击 (事件委托)', 'info');
                self.rotateView();
            } else if (event.target.id === 'testBtn') {
                console.log('事件委托: 测试按钮被点击');
                self.log('测试按钮被点击 (事件委托)', 'info');
                alert('测试按钮工作正常！JavaScript事件系统正常。');
            }
        });
        
        // 也尝试直接绑定事件监听器
        const resetViewBtn = document.getElementById('resetViewBtn');
        const toggleGridBtn = document.getElementById('toggleGridBtn');
        const toggleLabelsBtn = document.getElementById('toggleLabelsBtn');
        
        if (resetViewBtn) {
            resetViewBtn.addEventListener('click', function(e) {
                console.log('直接绑定: 重置视图按钮被点击');
                e.preventDefault();
                self.log('重置视图按钮直接点击', 'info');
                self.resetView();
            });
            this.log('重置视图按钮事件监听器已设置', 'info');
        } else {
            this.log('重置视图按钮未找到', 'error');
        }
        
        if (toggleGridBtn) {
            toggleGridBtn.addEventListener('click', function(e) {
                console.log('直接绑定: 网格按钮被点击');
                e.preventDefault();
                self.log('网格按钮直接点击', 'info');
                self.toggleGrid();
            });
            this.log('网格按钮事件监听器已设置', 'info');
        } else {
            this.log('网格按钮未找到', 'error');
        }
        
        if (toggleLabelsBtn) {
            toggleLabelsBtn.addEventListener('click', function(e) {
                console.log('直接绑定: 标签按钮被点击');
                e.preventDefault();
                self.log('标签按钮直接点击', 'info');
                self.toggleLabels();
            });
            this.log('标签按钮事件监听器已设置', 'info');
        } else {
            this.log('标签按钮未找到', 'error');
        }
        
        // 旋转按钮 - 移除直接绑定，只使用事件委托
        // 注意：旋转按钮使用事件委托，不需要直接绑定
        
        // 画布交互
        this.setupCanvasInteraction();
        
        // STP-23L传感器事件监听器
        this.setupSTP23LEventListeners();
        
        // 窗口大小变化监听器
        this.setupWindowResizeListener();
    }
    
    /**
     * 设置窗口大小变化监听器
     */
    setupWindowResizeListener() {
        window.addEventListener('resize', () => {
            // 延迟重绘，避免频繁调用
            clearTimeout(this.resizeTimeout);
            this.resizeTimeout = setTimeout(() => {
                if (this.heightChartCtx) {
                    this.drawHeightChart(this.stp23lSensor.heightHistory);
                }
                if (this.ctx) {
                    this.updateVisualization();
                }
            }, 100);
        });
    }
    
    /**
     * 设置STP-23L传感器事件监听器
     */
    setupSTP23LEventListeners() {
        // STP-23L连接控制
        const stp23lConnectBtn = document.getElementById('stp23lConnectBtn');
        const stp23lDisconnectBtn = document.getElementById('stp23lDisconnectBtn');
        
        if (stp23lConnectBtn) {
            stp23lConnectBtn.addEventListener('click', () => this.stp23lSensor.connect());
            console.log('STP-23L连接按钮事件监听器已设置');
        } else {
            console.error('STP-23L连接按钮未找到');
        }
        
        if (stp23lDisconnectBtn) {
            stp23lDisconnectBtn.addEventListener('click', () => this.stp23lSensor.disconnect());
            console.log('STP-23L断开按钮事件监听器已设置');
        } else {
            console.error('STP-23L断开按钮未找到');
        }
        
        // STP-23L测量控制
        const stp23lStartBtn = document.getElementById('stp23lStartBtn');
        const stp23lStopBtn = document.getElementById('stp23lStopBtn');
        
        if (stp23lStartBtn) {
            stp23lStartBtn.addEventListener('click', () => this.stp23lSensor.startMeasuring());
            console.log('STP-23L开始测量按钮事件监听器已设置');
        } else {
            console.error('STP-23L开始测量按钮未找到');
        }
        
        if (stp23lStopBtn) {
            stp23lStopBtn.addEventListener('click', () => this.stp23lSensor.stopMeasuring());
            console.log('STP-23L停止测量按钮事件监听器已设置');
        } else {
            console.error('STP-23L停止测量按钮未找到');
        }
        
        // STP-23L数据管理
        const clearHeightBtn = document.getElementById('clearHeightBtn');
        const heightThresholdInput = document.getElementById('heightThreshold');
        
        if (clearHeightBtn) {
            clearHeightBtn.addEventListener('click', () => this.stp23lSensor.clearHeightData());
            console.log('STP-23L清除数据按钮事件监听器已设置');
        } else {
            console.error('STP-23L清除数据按钮未找到');
        }
        
        if (heightThresholdInput) {
            heightThresholdInput.addEventListener('input', (e) => {
                this.stp23lSensor.heightThreshold = parseInt(e.target.value);
            });
            console.log('STP-23L高度阈值输入框事件监听器已设置');
        } else {
            console.error('STP-23L高度阈值输入框未找到');
        }
        
        // 高度图表控制
        const exportHeightBtn = document.getElementById('exportHeightBtn');
        const resetHeightChartBtn = document.getElementById('resetHeightChartBtn');
        
        if (exportHeightBtn) {
            exportHeightBtn.addEventListener('click', () => this.exportHeightData());
            console.log('导出高度数据按钮事件监听器已设置');
        } else {
            console.error('导出高度数据按钮未找到');
        }
        
        if (resetHeightChartBtn) {
            resetHeightChartBtn.addEventListener('click', () => this.resetHeightChart());
            console.log('重置高度图表按钮事件监听器已设置');
        } else {
            console.error('重置高度图表按钮未找到');
        }
        
        // 添加测试数据按钮（仅用于调试）
        const testHeightBtn = document.getElementById('testHeightBtn');
        if (testHeightBtn) {
            testHeightBtn.addEventListener('click', () => this.generateTestHeightData());
            console.log('测试高度数据按钮事件监听器已设置');
        }
    }
    
    /**
     * 连接激光雷达
     * @returns {Promise<void>}
     */
    async connect() {
        try {
            this.log('正在连接激光雷达...', 'info');
            
            // 请求串口访问权限
            this.port = await navigator.serial.requestPort();
            
            // 打开串口，使用常量配置
            await this.port.open({
                baudRate: LIDAR_CONSTANTS.BAUD_RATE,
                dataBits: LIDAR_CONSTANTS.DATA_BITS,
                stopBits: LIDAR_CONSTANTS.STOP_BITS,
                parity: LIDAR_CONSTANTS.PARITY,
                flowControl: LIDAR_CONSTANTS.FLOW_CONTROL
            });
            
            this.log('串口已打开，开始初始化激光雷达...', 'info');
            
            // 初始化激光雷达
            await this.initializeLidar();
            
            this.isConnected = true;
            this.updateStatus('已连接', 'connected');
            this.updateButtons();
            this.log('激光雷达连接成功！', 'success');
            
        } catch (error) {
            this.handleConnectionError(error);
        }
    }
    
    /**
     * 处理连接错误
     * @param {Error} error - 错误对象
     */
    handleConnectionError(error) {
        this.log(`连接失败: ${error.message}`, 'error');
        this.isConnected = false;
        this.updateStatus('连接失败', 'disconnected');
        this.updateButtons();
        
        // 清理资源
        this.cleanup();
    }
    
    /**
     * 断开激光雷达连接
     * @returns {Promise<void>}
     */
    async disconnect() {
        try {
            this.log('正在断开连接...', 'info');
            
            // 停止扫描
            this.stopScanning();
            
            // 发送关闭激光命令
            await this.sendLaserOffCommand();
            
            // 关闭串口
            await this.closeSerialPort();
            
            // 清理状态
            this.cleanup();
            
            this.updateStatus('已断开', 'disconnected');
            this.updateButtons();
            this.updateVisualization();
            this.log('激光雷达已断开连接', 'info');
            
        } catch (error) {
            this.log(`断开连接失败: ${error.message}`, 'error');
        }
    }
    
    /**
     * 停止扫描
     */
    stopScanning() {
        this.isScanning = false;
        
        if (this.scanInterval) {
            clearInterval(this.scanInterval);
            this.scanInterval = null;
        }
        
        if (this.scanAnimationFrame) {
            cancelAnimationFrame(this.scanAnimationFrame);
            this.scanAnimationFrame = null;
        }
    }
    
    /**
     * 发送关闭激光命令
     * @returns {Promise<void>}
     */
    async sendLaserOffCommand() {
        if (!this.port) return;
        
        try {
            const writer = this.port.writable.getWriter();
            await this.sendCommand(writer, LIDAR_CONSTANTS.COMMANDS.LASER_OFF);
            writer.releaseLock();
            this.log('已发送QT命令关闭激光', 'info');
        } catch (error) {
            this.log('发送QT命令失败', 'warning');
        }
    }
    
    /**
     * 关闭串口
     * @returns {Promise<void>}
     */
    async closeSerialPort() {
        if (!this.port) return;
        
        try {
            await this.port.close();
            this.port = null;
        } catch (error) {
            this.log('关闭串口失败', 'warning');
        }
    }
    
    /**
     * 清理系统资源
     */
    cleanup() {
        // 释放读取器
        if (this.reader) {
            try {
                this.reader.releaseLock();
            } catch (e) {
                // 忽略释放错误
            }
            this.reader = null;
        }
        
        // 重置状态
        this.isConnected = false;
        this.scanData = [];
        this.trees = [];
    }
    
    /**
     * 初始化激光雷达
     * @returns {Promise<void>}
     */
    async initializeLidar() {
        if (!this.port) {
            throw new Error('串口未连接');
        }
        
        try {
            const writer = this.port.writable.getWriter();
            
            // 初始化序列：SCIP2.0 -> VV -> BM
            const initSequence = [
                { command: LIDAR_CONSTANTS.COMMANDS.SCIP, description: 'SCIP2.0协议切换' },
                { command: LIDAR_CONSTANTS.COMMANDS.VERSION, description: '获取传感器信息' },
                { command: LIDAR_CONSTANTS.COMMANDS.LASER_ON, description: '使能激光' }
            ];
            
            for (const step of initSequence) {
                await this.sendCommand(writer, step.command);
                await this.delay(LIDAR_CONSTANTS.DELAYS.INIT_RESPONSE);
                await this.readResponse();
                this.log(`${step.description}完成`, 'info');
            }
            
            writer.releaseLock();
            this.log('激光雷达初始化完成', 'success');
            
        } catch (error) {
            this.log(`初始化失败: ${error.message}`, 'error');
            throw error;
        }
    }
    
    async readResponse() {
        // 读取激光雷达的响应
        try {
            if (this.reader) {
                try {
                    this.reader.releaseLock();
                } catch (e) {
                    // 忽略释放错误
                }
                this.reader = null;
            }
            
            this.reader = this.port.readable.getReader();
            let response = '';
            let attempts = 0;
            const maxAttempts = 10;
            
            while (attempts < maxAttempts) {
                const { value, done } = await this.reader.read();
                if (done) break;
                
                const chunk = new TextDecoder().decode(value);
                response += chunk;
                
                // 检查是否收到完整响应（通常以换行符结束）
                if (response.includes('\n') || response.includes('\r')) {
                    break;
                }
                
                attempts++;
                await this.delay(50);
            }
            
            if (response.trim()) {
                this.log(`收到响应: ${response.trim()}`, 'info');
            }
            
            return response;
            
        } finally {
            if (this.reader) {
                try {
                    this.reader.releaseLock();
                } catch (e) {
                    // 忽略释放错误
                }
                this.reader = null;
            }
        }
    }

    async sendCommand(writer, command) {
        const encoder = new TextEncoder();
        const data = encoder.encode(command + '\r');
        await writer.write(data);
        this.log(`发送命令: ${command}`, 'info');
    }
    
    delay(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
    
    /**
     * 开始连续扫描
     */
    async startScanning() {
        if (!this.isConnected) {
            this.log('激光雷达未连接', 'warning');
            return;
        }
        
        this.isScanning = true;
        this.updateButtons();
        this.log('开始连续扫描...', 'info');
        
        this.scanInterval = setInterval(async () => {
            if (this.isScanning) {
                await this.performScan();
            }
        }, LIDAR_CONSTANTS.DELAYS.SCAN_INTERVAL);
    }
    
    pauseScanning() {
        this.isScanning = false;
        if (this.scanInterval) {
            clearInterval(this.scanInterval);
            this.scanInterval = null;
        }
        this.updateButtons();
        this.log('扫描已暂停', 'info');
    }
    
    async singleScan() {
        if (!this.isConnected) return;
        
        this.log('执行单次扫描...', 'info');
        await this.performScan();
    }
    
    async performScan() {
        if (!this.port) {
            this.log('激光雷达未连接', 'error');
            return;
        }
        
        const startTime = performance.now();
        
        try {
            // 从真实激光雷达获取数据
            this.log('开始执行扫描...', 'info');
            const scanData = await this.getRealLidarData();
            this.scanData = scanData;
            
            this.log(`扫描完成，获得 ${scanData.length} 个数据点`, 'success');
            
            // 更新显示
            this.updateVisualization();
            this.updateStats(scanData, performance.now() - startTime);
            
            // 更新按钮状态 - 修复bug：扫描完成后启用树木检测按钮
            this.updateButtons();
            
        } catch (error) {
            this.log(`扫描失败: ${error.message}`, 'error');
        }
    }
    
    /**
     * 获取真实激光雷达数据
     * @returns {Promise<Array>} 扫描数据数组
     */
    async getRealLidarData() {
        if (!this.port) {
            throw new Error('激光雷达未连接');
        }
        
        try {
            // 发送扫描命令
            const writer = this.port.writable.getWriter();
            await this.sendCommand(writer, LIDAR_CONSTANTS.COMMANDS.SCAN);
            writer.releaseLock();
            
            this.log('已发送扫描命令，等待数据...', 'info');
            
            // 等待激光雷达响应
            await this.delay(LIDAR_CONSTANTS.DELAYS.SCAN_RESPONSE);
            
            // 读取串口数据
            const data = await this.readSerialData();
            
            // 验证数据包完整性
            if (data.length < LIDAR_CONSTANTS.EXPECTED_DATA_SIZE) {
                throw new Error(`数据包不完整，只收到 ${data.length} 字节，期望 ${LIDAR_CONSTANTS.EXPECTED_DATA_SIZE} 字节`);
            }
            
            // 处理数据包（按照MATLAB逻辑）
            return this.processLidarData(data);
            
        } catch (error) {
            this.log(`获取激光雷达数据失败: ${error.message}`, 'error');
            throw error;
        }
    }
    
    /**
     * 读取串口数据
     * @returns {Promise<string>} 原始数据字符串
     */
    async readSerialData() {
        // 按照MATLAB逻辑：等待字节可用性 >= 2134
        let proceed = false;
        let data = '';
        
        // 确保之前的读取器被释放
        this.releaseReader();
        
        // 获取新的读取器
        this.reader = this.port.readable.getReader();
        
        try {
            while (!proceed) {
                // 检查是否有足够的数据可用（模拟MATLAB的BytesAvailable）
                const { value, done } = await this.reader.read();
                if (done) break;
                
                const chunk = new TextDecoder().decode(value);
                data += chunk;
                
                this.log(`接收到数据块: ${chunk.length} 字节，总计: ${data.length} 字节`, 'info');
                
                // 检查是否收到完整数据包
                if (data.length >= LIDAR_CONSTANTS.EXPECTED_DATA_SIZE) {
                    this.log(`收到完整数据包: ${data.length} 字节`, 'success');
                    proceed = true;
                }
                
                // 防止无限循环
                if (data.length > LIDAR_CONSTANTS.MAX_BUFFER_SIZE) {
                    this.log('数据包过大，可能有问题', 'warning');
                    break;
                }
            }
            
            return data;
            
        } catch (error) {
            this.log(`读取串口数据失败: ${error.message}`, 'error');
            throw error;
        } finally {
            // 释放读取器
            this.releaseReader();
        }
    }
    
    /**
     * 释放读取器
     */
    releaseReader() {
        if (this.reader) {
            try {
                this.reader.releaseLock();
            } catch (e) {
                // 忽略释放错误，但记录警告
                if (e.message !== 'Releasing Default reader') {
                    this.log(`释放读取器警告: ${e.message}`, 'warning');
                }
            }
            this.reader = null;
        }
    }
    
    
    /**
     * 处理激光雷达数据
     * @param {string} rawData - 原始数据字符串
     * @returns {Array} 处理后的扫描数据
     */
    processLidarData(rawData) {
        try {
            this.log(`开始处理数据包，长度: ${rawData.length}`, 'info');
            
            // 验证数据长度
            if (rawData.length < LIDAR_CONSTANTS.EXPECTED_DATA_SIZE) {
                throw new Error(`数据包长度不足: ${rawData.length} < ${LIDAR_CONSTANTS.EXPECTED_DATA_SIZE}`);
            }
            
            // 按照MATLAB的LidarScan.m逻辑处理数据
            const data = rawData.substring(0, LIDAR_CONSTANTS.EXPECTED_DATA_SIZE);
            
            // 查找数据开始位置（对应MATLAB的 i=find(data==data(13))）
            const startChar = data[12]; // data(13)对应索引12
            this.log(`起始字符: '${startChar}' (ASCII: ${startChar.charCodeAt(0)})`, 'info');
            
            const startIndices = this.findStartIndices(data, startChar);
            this.log(`找到 ${startIndices.length} 个起始标识符，位置: [${startIndices.join(', ')}]`, 'info');
            
            if (startIndices.length < 3) {
                throw new Error(`数据格式错误：只找到 ${startIndices.length} 个起始标识符，期望至少3个`);
            }
            
            // 提取和重组距离数据
            const rangedata = data.substring(startIndices[2] + 1, data.length - 1);
            const onlyrangedata = this.reorganizeRangeData(rangedata);
            
            // 转换为编码距离数据
            const encodeddist = this.encodeDistanceData(onlyrangedata);
            
            // 解码距离数据
            const ranges = this.decodeDistanceData(encodeddist);
            
            // 统计距离数据
            this.logDistanceStats(ranges);
            
            // 转换为坐标
            const scanData = this.convertToCoordinates(ranges);
            
            this.log(`成功处理 ${scanData.length} 个扫描点`, 'success');
            return scanData;
            
        } catch (error) {
            this.log(`数据处理失败: ${error.message}`, 'error');
            this.log(`原始数据前100字符: ${rawData.substring(0, 100)}`, 'error');
            throw error;
        }
    }
    
    /**
     * 查找起始标识符位置
     * @param {string} data - 数据字符串
     * @param {string} startChar - 起始字符
     * @returns {Array<number>} 起始位置数组
     */
    findStartIndices(data, startChar) {
        const startIndices = [];
        for (let i = 0; i < data.length; i++) {
            if (data[i] === startChar) {
                startIndices.push(i);
            }
        }
        return startIndices;
    }
    
    /**
     * 重新组织距离数据
     * @param {string} rangedata - 原始距离数据
     * @returns {string} 重组后的距离数据
     */
    reorganizeRangeData(rangedata) {
        this.log(`提取距离数据长度: ${rangedata.length}`, 'info');
        
        let onlyrangedata = '';
        for (let j = 0; j < 32; j++) {
            const start = 1 + (66 * j);
            const end = 64 + (66 * j);
            if (end <= rangedata.length) {
                onlyrangedata += rangedata.substring(start, end);
            }
        }
        
        this.log(`重新组织后数据长度: ${onlyrangedata.length}`, 'info');
        return onlyrangedata;
    }
    
    /**
     * 编码距离数据
     * @param {string} onlyrangedata - 重组后的距离数据
     * @returns {Array<Array<string>>} 编码后的距离数据
     */
    encodeDistanceData(onlyrangedata) {
        const encodeddist = [];
        for (let i = 0; i < Math.floor(onlyrangedata.length / 3); i++) {
            const start = i * 3;
            encodeddist.push([
                onlyrangedata[start],
                onlyrangedata[start + 1],
                onlyrangedata[start + 2]
            ]);
        }
        
        this.log(`编码距离数据组数: ${encodeddist.length}`, 'info');
        return encodeddist;
    }
    
    /**
     * 解码距离数据
     * @param {Array<Array<string>>} encodeddist - 编码后的距离数据
     * @returns {Array<number>} 解码后的距离数组
     */
    decodeDistanceData(encodeddist) {
        const ranges = [];
        for (let k = 0; k < encodeddist.length; k++) {
            const distance = this.decodeSCIP(encodeddist[k]);
            ranges.push(distance);
        }
        return ranges;
    }
    
    /**
     * 记录距离统计信息
     * @param {Array<number>} ranges - 距离数组
     */
    logDistanceStats(ranges) {
        const validRanges = ranges.filter(r => r > 0 && r < LIDAR_CONSTANTS.MAX_RANGE);
        const avgDistance = validRanges.length > 0 ? validRanges.reduce((a, b) => a + b, 0) / validRanges.length : 0;
        this.log(`解码完成: ${ranges.length} 个距离值，有效值: ${validRanges.length}，平均距离: ${Math.round(avgDistance)}mm`, 'info');
    }
    
    /**
     * 转换为坐标
     * @param {Array<number>} ranges - 距离数组
     * @returns {Array<Object>} 扫描数据数组
     */
    convertToCoordinates(ranges) {
        const scanData = [];
        
        for (let i = 0; i < ranges.length; i++) {
            // 计算角度：完全按照MATLAB逻辑
            // MATLAB: min_angle=(-120:240/682:120-240/682)*pi/180;
            const angle = (LIDAR_CONSTANTS.START_ANGLE + (240 * i) / LIDAR_CONSTANTS.TOTAL_POINTS) * Math.PI / 180;
            const distance = ranges[i];
            
            scanData.push({
                angle: angle,
                distance: distance,
                x: distance * Math.cos(angle),
                y: distance * Math.sin(angle)
            });
        }
        
        return scanData;
    }
    
    decodeSCIP(rangeenc) {
        // 完全按照MATLAB的decodeSCIP.m逻辑
        if (rangeenc[0] === '0' && rangeenc[1] === '0' && rangeenc[2] === '0') {
            return 0;
        }
        
        if (rangeenc[0] === '0') {
            // 2字符编码
            const dig1 = ((rangeenc[1].charCodeAt(0) - '!'.charCodeAt(0)) + 33);
            const dig2 = ((rangeenc[2].charCodeAt(0) - '!'.charCodeAt(0)) + 33);
            const dig1sub = dig1 - 48;
            const dig2sub = dig2 - 48;
            const dig1bin = dig1sub.toString(2).padStart(6, '0');
            const dig2bin = dig2sub.toString(2).padStart(6, '0');
            const result = parseInt(dig1bin + dig2bin, 2);
            
            // 调试信息
            if (Math.random() < 0.01) { // 1%的概率输出调试信息
                this.log(`2字符解码: [${rangeenc.join('')}] -> dig1=${dig1sub}, dig2=${dig2sub} -> ${result}mm`, 'info');
            }
            
            return result;
        } else {
            // 3字符编码
            const dig1 = ((rangeenc[0].charCodeAt(0) - '!'.charCodeAt(0)) + 33);
            const dig2 = ((rangeenc[1].charCodeAt(0) - '!'.charCodeAt(0)) + 33);
            const dig3 = ((rangeenc[2].charCodeAt(0) - '!'.charCodeAt(0)) + 33);
            const dig1sub = dig1 - 48;
            const dig2sub = dig2 - 48;
            const dig3sub = dig3 - 48;
            const dig1bin = dig1sub.toString(2).padStart(6, '0');
            const dig2bin = dig2sub.toString(2).padStart(6, '0');
            const dig3bin = dig3sub.toString(2).padStart(6, '0');
            const result = parseInt(dig1bin + dig2bin + dig3bin, 2);
            
            // 调试信息
            if (Math.random() < 0.01) { // 1%的概率输出调试信息
                this.log(`3字符解码: [${rangeenc.join('')}] -> dig1=${dig1sub}, dig2=${dig2sub}, dig3=${dig3sub} -> ${result}mm`, 'info');
            }
            
            return result;
        }
    }
    
    /**
     * 更新可视化显示
     */
    updateVisualization() {
        // 完全重置Canvas状态
        this.ctx.setTransform(1, 0, 0, 1, 0, 0);
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 应用视图变换
        this.applyViewTransform();
        
        // 绘制背景网格
        if (this.viewSettings.showGrid) {
            this.drawGrid();
        }
        
        // 绘制扫描点
        this.drawScanPoints();
        
        // 绘制检测到的树木
        this.drawTrees();
        
        // 扫描线动画已移除
        
        // 重置视图变换
        this.resetViewTransform();
        
        // 更新扫描信息显示
        this.updateScanInfo();
    }
    
    /**
     * 应用视图变换
     */
    applyViewTransform() {
        const rect = this.canvas.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        
        this.ctx.save();
        // 先平移到中心点
        this.ctx.translate(centerX + this.viewSettings.offsetX, centerY + this.viewSettings.offsetY);
        // 再应用缩放
        this.ctx.scale(this.viewSettings.zoom, this.viewSettings.zoom);
        // 最后应用旋转（相对于中心点）
        this.ctx.rotate(this.viewSettings.rotation * Math.PI / 180);
        
        console.log('应用视图变换 - 旋转角度:', this.viewSettings.rotation, '度');
    }
    
    /**
     * 重置视图变换
     */
    resetViewTransform() {
        this.ctx.restore();
    }
    
    /**
     * 重置视图
     */
    resetView() {
        console.log('resetView方法被调用');
        this.log('=== 重置视图功能被调用 ===', 'info');
        
        // 重置所有视图参数
        this.viewSettings.zoom = 1.0;
        this.viewSettings.offsetX = 0;
        this.viewSettings.offsetY = 0;
        this.viewSettings.rotation = 0; // 确保旋转角度重置为0
        
        console.log('viewSettings已更新:', this.viewSettings);
        console.log('旋转角度已重置为:', this.viewSettings.rotation);
        
        this.updateVisualization();
        this.log('视图已重置 - 缩放:1.0, 偏移:(0,0), 旋转:0°', 'success');
        
        // 添加视觉反馈
        const btn = document.getElementById('resetViewBtn');
        if (btn) {
            btn.style.backgroundColor = '#48bb78';
            setTimeout(() => {
                btn.style.backgroundColor = '';
            }, 200);
        }
    }
    
    /**
     * 切换网格显示
     */
    toggleGrid() {
        console.log('toggleGrid方法被调用');
        this.log('=== 网格切换功能被调用 ===', 'info');
        this.viewSettings.showGrid = !this.viewSettings.showGrid;
        console.log('网格显示状态:', this.viewSettings.showGrid);
        const btn = document.getElementById('toggleGridBtn');
        if (btn) {
            btn.classList.toggle('active', this.viewSettings.showGrid);
            // 添加视觉反馈
            btn.style.backgroundColor = this.viewSettings.showGrid ? '#3182ce' : '#48bb78';
            setTimeout(() => {
                btn.style.backgroundColor = '';
            }, 200);
        }
        this.updateVisualization();
        this.log(`网格显示: ${this.viewSettings.showGrid ? '开启' : '关闭'}`, 'success');
    }
    
    /**
     * 切换标签显示
     */
    toggleLabels() {
        console.log('toggleLabels方法被调用');
        this.log('=== 标签切换功能被调用 ===', 'info');
        this.viewSettings.showLabels = !this.viewSettings.showLabels;
        console.log('标签显示状态:', this.viewSettings.showLabels);
        const btn = document.getElementById('toggleLabelsBtn');
        if (btn) {
            btn.classList.toggle('active', this.viewSettings.showLabels);
            // 添加视觉反馈
            btn.style.backgroundColor = this.viewSettings.showLabels ? '#3182ce' : '#48bb78';
            setTimeout(() => {
                btn.style.backgroundColor = '';
            }, 200);
        }
        this.updateVisualization();
        this.log(`标签显示: ${this.viewSettings.showLabels ? '开启' : '关闭'}`, 'success');
    }
    
    /**
     * 旋转视图90度
     */
    rotateView() {
        console.log('rotateView方法被调用 - 当前角度:', this.viewSettings.rotation);
        
        // 确保旋转角度是有效的数值
        if (typeof this.viewSettings.rotation !== 'number' || isNaN(this.viewSettings.rotation)) {
            this.viewSettings.rotation = 0;
            console.log('旋转角度被重置为0');
        }
        
        // 每次旋转90度
        this.viewSettings.rotation += 90;
        
        // 如果超过360度，重置为0度
        if (this.viewSettings.rotation >= 360) {
            this.viewSettings.rotation = 0;
        }
        
        console.log('旋转后角度:', this.viewSettings.rotation);
        
        // 更新可视化
        this.updateVisualization();
        this.log(`视图已旋转至: ${this.viewSettings.rotation}°`, 'success');
        
        // 添加视觉反馈
        const btn = document.getElementById('rotateViewBtn');
        if (btn) {
            btn.style.backgroundColor = '#ff6b6b';
            setTimeout(() => {
                btn.style.backgroundColor = '';
            }, 200);
        }
    }
    
    /**
     * 设置画布交互
     */
    setupCanvasInteraction() {
        let isDragging = false;
        let lastX = 0;
        let lastY = 0;
        
        // 鼠标滚轮缩放
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? 0.9 : 1.1;
            this.viewSettings.zoom *= delta;
            this.viewSettings.zoom = Math.max(0.1, Math.min(5.0, this.viewSettings.zoom));
            this.updateVisualization();
        });
        
        // 鼠标拖拽平移
        this.canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastX = e.clientX;
            lastY = e.clientY;
        });
        
        this.canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const deltaX = e.clientX - lastX;
                const deltaY = e.clientY - lastY;
                this.viewSettings.offsetX += deltaX;
                this.viewSettings.offsetY += deltaY;
                lastX = e.clientX;
                lastY = e.clientY;
                this.updateVisualization();
            }
        });
        
        this.canvas.addEventListener('mouseup', () => {
            isDragging = false;
        });
        
        this.canvas.addEventListener('mouseleave', () => {
            isDragging = false;
        });
    }
    
    /**
     * 更新扫描信息显示
     */
    updateScanInfo() {
        const currentAngleElement = document.getElementById('currentAngle');
        const scanProgressElement = document.getElementById('scanProgress');
        
        if (currentAngleElement) {
            currentAngleElement.textContent = `${this.currentScanAngle.toFixed(1)}°`;
        }
        
        if (scanProgressElement) {
            const progress = ((this.currentScanAngle - LIDAR_CONSTANTS.START_ANGLE) / 
                            (LIDAR_CONSTANTS.END_ANGLE - LIDAR_CONSTANTS.START_ANGLE)) * 100;
            scanProgressElement.textContent = `${Math.max(0, Math.min(100, progress)).toFixed(1)}%`;
        }
    }
    
    /**
     * 绘制网格
     */
    drawGrid() {
        const maxRange = this.scanParams.maxRange;
        const scale = 0.8; // 固定缩放比例
        
        // 绘制同心圆网格 - 根据缩放级别自动调整密度和线条粗细
        this.ctx.strokeStyle = 'rgba(240, 240, 240, 0.3)';
        
        // 根据缩放级别调整网格线条粗细
        let gridLineWidth = 1; // 默认线条宽度
        if (this.viewSettings.zoom > 3.0) {
            gridLineWidth = 2.5; // 超高缩放：粗线条
        } else if (this.viewSettings.zoom > 2.5) {
            gridLineWidth = 2.2;
        } else if (this.viewSettings.zoom > 2.0) {
            gridLineWidth = 2.0;
        } else if (this.viewSettings.zoom > 1.5) {
            gridLineWidth = 1.8;
        } else if (this.viewSettings.zoom > 1.0) {
            gridLineWidth = 1.5;
        } else if (this.viewSettings.zoom < 0.5) {
            gridLineWidth = 0.5; // 低缩放：细线条
        } else if (this.viewSettings.zoom < 0.8) {
            gridLineWidth = 0.8;
        }
        
        this.ctx.lineWidth = gridLineWidth;
        
        // 根据缩放级别确定网格密度 - 大幅增加密度
        let gridDensity = 12; // 默认12个圆圈
        if (this.viewSettings.zoom > 3.0) {
            gridDensity = 40; // 超高缩放：40个圆圈
        } else if (this.viewSettings.zoom > 2.5) {
            gridDensity = 32; // 很高缩放：32个圆圈
        } else if (this.viewSettings.zoom > 2.0) {
            gridDensity = 28; // 高缩放：28个圆圈
        } else if (this.viewSettings.zoom > 1.5) {
            gridDensity = 24; // 中高缩放：24个圆圈
        } else if (this.viewSettings.zoom > 1.0) {
            gridDensity = 20; // 中缩放：20个圆圈
        } else if (this.viewSettings.zoom < 0.5) {
            gridDensity = 8; // 低缩放：8个圆圈
        } else if (this.viewSettings.zoom < 0.8) {
            gridDensity = 10; // 中低缩放：10个圆圈
        }
        
        for (let i = 1; i <= gridDensity; i++) {
            const radius = (maxRange / gridDensity) * i * scale;
            this.ctx.beginPath();
            this.ctx.arc(0, 0, radius, 0, 2 * Math.PI);
            this.ctx.stroke();
        }
        
        // 绘制扫描范围阴影和边界
        this.drawScanRange(maxRange, scale);
        
        // 绘制角度线 - 根据缩放级别自动调整密度和线条粗细
        this.ctx.strokeStyle = 'rgba(208, 208, 208, 0.4)';
        
        // 根据缩放级别调整角度线粗细
        let angleLineWidth = 1; // 默认线条宽度
        if (this.viewSettings.zoom > 3.0) {
            angleLineWidth = 2.0; // 超高缩放：粗线条
        } else if (this.viewSettings.zoom > 2.5) {
            angleLineWidth = 1.8;
        } else if (this.viewSettings.zoom > 2.0) {
            angleLineWidth = 1.6;
        } else if (this.viewSettings.zoom > 1.5) {
            angleLineWidth = 1.4;
        } else if (this.viewSettings.zoom > 1.0) {
            angleLineWidth = 1.2;
        } else if (this.viewSettings.zoom < 0.5) {
            angleLineWidth = 0.5; // 低缩放：细线条
        } else if (this.viewSettings.zoom < 0.8) {
            angleLineWidth = 0.7;
        }
        
        this.ctx.lineWidth = angleLineWidth;
        
        // 根据缩放级别确定角度线密度 - 大幅增加密度
        let angleDensity = 12; // 默认12条线（每30°一条）
        if (this.viewSettings.zoom > 3.0) {
            angleDensity = 72; // 超高缩放：72条线（每5°一条）
        } else if (this.viewSettings.zoom > 2.5) {
            angleDensity = 48; // 很高缩放：48条线（每7.5°一条）
        } else if (this.viewSettings.zoom > 2.0) {
            angleDensity = 36; // 高缩放：36条线（每10°一条）
        } else if (this.viewSettings.zoom > 1.5) {
            angleDensity = 24; // 中高缩放：24条线（每15°一条）
        } else if (this.viewSettings.zoom > 1.0) {
            angleDensity = 18; // 中缩放：18条线（每20°一条）
        } else if (this.viewSettings.zoom < 0.5) {
            angleDensity = 6; // 低缩放：6条线（每60°一条）
        } else if (this.viewSettings.zoom < 0.8) {
            angleDensity = 8; // 中低缩放：8条线（每45°一条）
        }
        
        for (let i = 0; i < angleDensity; i++) {
            const angle = (2 * Math.PI / angleDensity) * i;
            const x = Math.cos(angle) * maxRange * scale;
            const y = Math.sin(angle) * maxRange * scale;
            
            this.ctx.beginPath();
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo(x, y);
            this.ctx.stroke();
        }
        
        // 绘制中心点 - 根据缩放级别调整大小
        this.ctx.fillStyle = '#666';
        
        // 根据缩放级别调整中心点大小
        let centerPointSize = 3; // 默认中心点大小
        if (this.viewSettings.zoom > 3.0) {
            centerPointSize = 6; // 超高缩放：大中心点
        } else if (this.viewSettings.zoom > 2.5) {
            centerPointSize = 5.5;
        } else if (this.viewSettings.zoom > 2.0) {
            centerPointSize = 5;
        } else if (this.viewSettings.zoom > 1.5) {
            centerPointSize = 4.5;
        } else if (this.viewSettings.zoom > 1.0) {
            centerPointSize = 4;
        } else if (this.viewSettings.zoom < 0.5) {
            centerPointSize = 2; // 低缩放：小中心点
        } else if (this.viewSettings.zoom < 0.8) {
            centerPointSize = 2.5;
        }
        
        this.ctx.beginPath();
        this.ctx.arc(0, 0, centerPointSize, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // 绘制距离标签 - 固定字体大小
        if (this.viewSettings.showLabels) {
            this.ctx.fillStyle = 'rgba(102, 102, 102, 0.8)';
            
            // 固定字体大小，不受缩放影响
            const fontSize = 12;
            this.ctx.font = `${fontSize}px Arial`;
            this.ctx.textAlign = 'center';
            
            // 根据缩放级别确定标签密度 - 大幅增加密度
            let labelDensity = 12; // 默认12个标签
            if (this.viewSettings.zoom > 3.0) {
                labelDensity = 40; // 超高缩放：40个标签
            } else if (this.viewSettings.zoom > 2.5) {
                labelDensity = 32; // 很高缩放：32个标签
            } else if (this.viewSettings.zoom > 2.0) {
                labelDensity = 28; // 高缩放：28个标签
            } else if (this.viewSettings.zoom > 1.5) {
                labelDensity = 24; // 中高缩放：24个标签
            } else if (this.viewSettings.zoom > 1.0) {
                labelDensity = 20; // 中缩放：20个标签
            } else if (this.viewSettings.zoom < 0.5) {
                labelDensity = 8; // 低缩放：8个标签
            } else if (this.viewSettings.zoom < 0.8) {
                labelDensity = 10; // 中低缩放：10个标签
            }
            
            // 绘制标签
            for (let i = 1; i <= labelDensity; i++) {
                const radius = (maxRange / labelDensity) * i * scale;
                const distance = (maxRange / labelDensity) * i;
                
                // 根据距离选择合适的单位
                let labelText;
                if (distance >= 1000) {
                    labelText = `${(distance/1000).toFixed(1)}m`;
                } else {
                    labelText = `${Math.round(distance)}mm`;
                }
                
                this.ctx.fillText(labelText, radius, -5);
            }
        }
    }
    
    /**
     * 绘制扫描范围（240度扫描区域和阴影）
     */
    drawScanRange(maxRange, scale) {
        // 使用真实的扫描范围：-120°到120°（240度）
        // 盲区在下方：120°到-120°（经过下方，120度范围）
        const startAngle = LIDAR_CONSTANTS.START_ANGLE * Math.PI / 180;  // -120度
        const endAngle = LIDAR_CONSTANTS.END_ANGLE * Math.PI / 180;      // 120度
        const scanRange = endAngle - startAngle;  // 240度
        
        // 1. 绘制扫描不到的区域的红色阴影（下方盲区）
        this.ctx.fillStyle = 'rgba(255, 0, 0, 0.15)'; // 半透明红色阴影
        
        // 绘制下方阴影区域（120度到-120度，经过下方）
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.arc(0, 0, maxRange * scale, endAngle, startAngle + 2 * Math.PI);
        this.ctx.closePath();
        this.ctx.fill();
        
        // 2. 绘制扫描范围边界线
        const scanArcLineWidth = this.getAdaptiveLineWidth(2, 4, 1);
        this.ctx.strokeStyle = 'rgba(59, 130, 246, 0.8)'; // 蓝色边界线
        this.ctx.lineWidth = scanArcLineWidth;
        
        // 绘制扫描范围弧线
        this.ctx.beginPath();
        this.ctx.arc(0, 0, maxRange * scale, startAngle, endAngle);
        this.ctx.stroke();
        
        // 3. 绘制扫描范围边界射线
        this.ctx.strokeStyle = 'rgba(59, 130, 246, 0.6)';
        this.ctx.lineWidth = scanArcLineWidth * 0.8;
        
        // 左边界射线（-120度）
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(
            Math.cos(startAngle) * maxRange * scale,
            Math.sin(startAngle) * maxRange * scale
        );
        this.ctx.stroke();
        
        // 右边界射线（120度）
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(
            Math.cos(endAngle) * maxRange * scale,
            Math.sin(endAngle) * maxRange * scale
        );
        this.ctx.stroke();
        
        // 4. 绘制扫描范围标签
        if (this.viewSettings.showLabels) {
            this.ctx.fillStyle = 'rgba(59, 130, 246, 0.9)';
            const fontSize = 12; // 固定字体大小
            this.ctx.font = `${fontSize}px Arial`;
            this.ctx.textAlign = 'center';
            
            // 在扫描范围中心显示"240°扫描范围"（上方）
            const centerAngle = (startAngle + endAngle) / 2;
            const labelRadius = maxRange * scale * 0.7;
            const labelX = Math.cos(centerAngle) * labelRadius;
            const labelY = Math.sin(centerAngle) * labelRadius;
            
            this.ctx.fillText('240°扫描范围', labelX, labelY);
            
            // 在下方阴影区域显示"盲区"
            const blindAngle = Math.PI; // 正下方（180度）
            
            this.ctx.fillStyle = 'rgba(255, 0, 0, 0.8)';
            this.ctx.font = `${fontSize}px Arial`; // 固定字体大小
            
            // 下方盲区标签
            const blindLabelX = Math.cos(blindAngle) * labelRadius * 0.6;
            const blindLabelY = Math.sin(blindAngle) * labelRadius * 0.6;
            this.ctx.fillText('盲区', blindLabelX, blindLabelY);
        }
    }
    
    /**
     * 获取自适应线条宽度
     */
    getAdaptiveLineWidth(defaultWidth, maxWidth, minWidth) {
        if (this.viewSettings.zoom > 3.0) return maxWidth;
        if (this.viewSettings.zoom > 2.5) return maxWidth * 0.9;
        if (this.viewSettings.zoom > 2.0) return maxWidth * 0.8;
        if (this.viewSettings.zoom > 1.5) return maxWidth * 0.7;
        if (this.viewSettings.zoom > 1.0) return defaultWidth;
        if (this.viewSettings.zoom < 0.5) return minWidth;
        if (this.viewSettings.zoom < 0.8) return minWidth * 1.5;
        return defaultWidth;
    }
    
    /**
     * 获取自适应字体大小
     */
    getAdaptiveFontSize(defaultSize, maxSize, minSize) {
        if (this.viewSettings.zoom > 3.0) return maxSize;
        if (this.viewSettings.zoom > 2.5) return maxSize * 0.9;
        if (this.viewSettings.zoom > 2.0) return maxSize * 0.8;
        if (this.viewSettings.zoom > 1.5) return maxSize * 0.7;
        if (this.viewSettings.zoom > 1.0) return defaultSize;
        if (this.viewSettings.zoom < 0.5) return minSize;
        if (this.viewSettings.zoom < 0.8) return minSize * 1.2;
        return defaultSize;
    }
    
    /**
     * 绘制扫描点
     */
    drawScanPoints() {
        if (!this.scanData.length) return;
        
        const scale = 0.8; // 与网格使用相同的缩放比例
        
        // 根据缩放级别调整点的显示密度和大小
        let pointStep = 3; // 默认每3个点显示1个
        let pointSize = 1.5; // 默认点大小
        
        if (this.viewSettings.zoom > 3.0) {
            pointStep = 1; // 超高缩放：显示所有点
            pointSize = 3.0; // 更大的点
        } else if (this.viewSettings.zoom > 2.5) {
            pointStep = 1; // 很高缩放：显示所有点
            pointSize = 2.5;
        } else if (this.viewSettings.zoom > 2.0) {
            pointStep = 2; // 高缩放：每2个点显示1个
            pointSize = 2.0;
        } else if (this.viewSettings.zoom > 1.5) {
            pointStep = 2; // 中高缩放：每2个点显示1个
            pointSize = 1.8;
        } else if (this.viewSettings.zoom > 1.0) {
            pointStep = 3; // 中缩放：每3个点显示1个
            pointSize = 1.5;
        } else if (this.viewSettings.zoom < 0.5) {
            pointStep = 5; // 低缩放：每5个点显示1个
            pointSize = 1.0;
        } else if (this.viewSettings.zoom < 0.8) {
            pointStep = 4; // 中低缩放：每4个点显示1个
            pointSize = 1.2;
        }
        
        // 绘制扫描点
        this.ctx.fillStyle = '#3b82f6';
        this.ctx.globalAlpha = 0.8;
        
        for (let i = 0; i < this.scanData.length; i += pointStep) {
            const point = this.scanData[i];
            const x = point.x * scale;
            const y = -point.y * scale; // 翻转Y轴
            
            this.ctx.beginPath();
            this.ctx.arc(x, y, pointSize, 0, 2 * Math.PI);
            this.ctx.fill();
        }
        
        this.ctx.globalAlpha = 1.0; // 重置透明度
    }
    
    /**
     * 绘制检测到的树木
     */
    drawTrees() {
        if (!this.trees.length) return;
        
        const scale = 0.8; // 与网格使用相同的缩放比例
        
        // 根据缩放级别调整树木显示
        let treeLineWidth = 2; // 默认线条宽度
        let centerPointSize = 3; // 默认中心点大小
        const fontSize = 12; // 固定字体大小，不受缩放影响
        
        if (this.viewSettings.zoom > 3.0) {
            treeLineWidth = 4;
            centerPointSize = 5;
        } else if (this.viewSettings.zoom > 2.5) {
            treeLineWidth = 3.5;
            centerPointSize = 4.5;
        } else if (this.viewSettings.zoom > 2.0) {
            treeLineWidth = 3;
            centerPointSize = 4;
        } else if (this.viewSettings.zoom > 1.5) {
            treeLineWidth = 2.5;
            centerPointSize = 3.5;
        } else if (this.viewSettings.zoom < 0.5) {
            treeLineWidth = 1;
            centerPointSize = 2;
        } else if (this.viewSettings.zoom < 0.8) {
            treeLineWidth = 1.5;
            centerPointSize = 2.5;
        }
        
        this.trees.forEach((tree, index) => {
            const x = tree.center.x * scale;
            const y = -tree.center.y * scale; // 翻转Y轴
            const radius = tree.radius * scale;
            
            // 绘制圆形
            this.ctx.strokeStyle = '#10b981';
            this.ctx.lineWidth = treeLineWidth;
            this.ctx.beginPath();
            this.ctx.arc(x, y, radius, 0, 2 * Math.PI);
            this.ctx.stroke();
            
            // 绘制中心点
            this.ctx.fillStyle = '#10b981';
            this.ctx.beginPath();
            this.ctx.arc(x, y, centerPointSize, 0, 2 * Math.PI);
            this.ctx.fill();
            
            // 绘制标签
            if (this.viewSettings.showLabels) {
                this.ctx.fillStyle = '#10b981';
                this.ctx.font = `${fontSize}px Arial`; // 使用固定字体大小
                this.ctx.textAlign = 'center';
                this.ctx.fillText(`树${index + 1}`, x, y - radius - 10);
                this.ctx.fillText(`${Math.round(tree.diameter)}mm`, x, y + radius + 20);
            }
        });
    }
    
    /**
     * 扫描动画
     */
    animateScan() {
        this.currentScanAngle += 2;
        if (this.currentScanAngle > LIDAR_CONSTANTS.END_ANGLE) {
            this.currentScanAngle = LIDAR_CONSTANTS.START_ANGLE;
        }
        
        // 绘制扫描线
        this.drawScanLine();
        
        this.scanAnimationFrame = requestAnimationFrame(() => {
            this.updateVisualization();
        });
    }
    
    /**
     * 绘制扫描线
     */
    drawScanLine() {
        const maxRange = this.scanParams.maxRange;
        const scale = 0.8;
        const currentAngle = this.currentScanAngle * Math.PI / 180;
        
        // 根据缩放级别调整扫描线显示
        let lineWidth = 3; // 默认线条宽度
        let pointSize = 4; // 默认扫描点大小
        
        if (this.viewSettings.zoom > 3.0) {
            lineWidth = 5;
            pointSize = 6;
        } else if (this.viewSettings.zoom > 2.5) {
            lineWidth = 4.5;
            pointSize = 5.5;
        } else if (this.viewSettings.zoom > 2.0) {
            lineWidth = 4;
            pointSize = 5;
        } else if (this.viewSettings.zoom > 1.5) {
            lineWidth = 3.5;
            pointSize = 4.5;
        } else if (this.viewSettings.zoom < 0.5) {
            lineWidth = 2;
            pointSize = 3;
        } else if (this.viewSettings.zoom < 0.8) {
            lineWidth = 2.5;
            pointSize = 3.5;
        }
        
        // 绘制扫描线
        this.ctx.strokeStyle = '#ff6b6b';
        this.ctx.lineWidth = lineWidth;
        this.ctx.globalAlpha = 0.8;
        
        const scanLineX = Math.cos(currentAngle) * maxRange * scale;
        const scanLineY = -Math.sin(currentAngle) * maxRange * scale;
        
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(scanLineX, scanLineY);
        this.ctx.stroke();
        
        // 绘制当前扫描点
        this.ctx.fillStyle = '#ff6b6b';
        this.ctx.beginPath();
        this.ctx.arc(scanLineX, scanLineY, pointSize, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // 绘制扫描扇形效果
        this.ctx.fillStyle = 'rgba(255, 107, 107, 0.1)';
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.arc(0, 0, maxRange * scale, currentAngle - 0.1, currentAngle + 0.1);
        this.ctx.closePath();
        this.ctx.fill();
        
        this.ctx.globalAlpha = 1.0; // 重置透明度
    }
    
    detectTrees() {
        if (!this.scanData.length) {
            this.log('没有扫描数据', 'warning');
            return;
        }
        
        this.log('开始检测树木...', 'info');
        
        // 过滤有效数据点
        const validPoints = this.scanData.filter(point => 
            point.distance > this.scanParams.minRange && 
            point.distance < this.scanParams.maxRange
        );
        
        if (validPoints.length < this.detectionParams.minPoints) {
            this.log('有效数据点不足，无法进行聚类', 'warning');
            return;
        }
        
        // DBSCAN聚类
        const clusters = this.dbscan(validPoints);
        
        // 对每个聚类进行圆形拟合
        this.trees = [];
        clusters.forEach((cluster, index) => {
            if (cluster.length >= this.detectionParams.minPoints) {
                const circle = this.fitCircle(cluster);
                if (circle && circle.radius >= this.detectionParams.minRadius && 
                    circle.radius <= this.detectionParams.maxRadius) {
                    this.trees.push({
                        center: circle.center,
                        radius: circle.radius,
                        diameter: circle.radius * 2,
                        points: cluster
                    });
                }
            }
        });
        
        this.updateTreeList();
        this.updateVisualization();
        this.updateStats(this.scanData, 0);
        
        this.log(`检测到 ${this.trees.length} 棵树`, 'success');
    }
    
    dbscan(points) {
        const clusters = [];
        const visited = new Set();
        const noise = new Set();
        
        points.forEach((point, index) => {
            if (visited.has(index)) return;
            
            visited.add(index);
            const neighbors = this.getNeighbors(points, index);
            
            if (neighbors.length < this.detectionParams.minPoints) {
                noise.add(index);
            } else {
                const cluster = [point];
                this.expandCluster(points, neighbors, cluster, visited, noise);
                clusters.push(cluster);
            }
        });
        
        return clusters;
    }
    
    getNeighbors(points, index) {
        const neighbors = [];
        const point = points[index];
        
        points.forEach((otherPoint, otherIndex) => {
            if (index === otherIndex) return;
            
            const distance = Math.sqrt(
                Math.pow(point.x - otherPoint.x, 2) + 
                Math.pow(point.y - otherPoint.y, 2)
            );
            
            if (distance <= this.detectionParams.eps) {
                neighbors.push(otherIndex);
            }
        });
        
        return neighbors;
    }
    
    expandCluster(points, neighbors, cluster, visited, noise) {
        let i = 0;
        while (i < neighbors.length) {
            const neighborIndex = neighbors[i];
            
            if (!visited.has(neighborIndex)) {
                visited.add(neighborIndex);
                const newNeighbors = this.getNeighbors(points, neighborIndex);
                if (newNeighbors.length >= this.detectionParams.minPoints) {
                    neighbors.push(...newNeighbors);
                }
            }
            
            if (!noise.has(neighborIndex)) {
                cluster.push(points[neighborIndex]);
            }
            
            i++;
        }
    }
    
    fitCircle(points) {
        if (points.length < 3) return null;
        
        // 使用最小二乘法拟合圆形
        let sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0;
        let sumX3 = 0, sumY3 = 0, sumXY2 = 0, sumX2Y = 0;
        
        points.forEach(point => {
            const x = point.x;
            const y = point.y;
            
            sumX += x;
            sumY += y;
            sumX2 += x * x;
            sumY2 += y * y;
            sumXY += x * y;
            sumX3 += x * x * x;
            sumY3 += y * y * y;
            sumXY2 += x * y * y;
            sumX2Y += x * x * y;
        });
        
        const n = points.length;
        const A = n * sumX2 - sumX * sumX;
        const B = n * sumXY - sumX * sumY;
        const C = n * sumY2 - sumY * sumY;
        const D = 0.5 * (n * sumXY2 - sumX * sumY2 + n * sumX3 - sumX * sumX2);
        const E = 0.5 * (n * sumX2Y - sumY * sumX2 + n * sumY3 - sumY * sumY2);
        
        const det = A * C - B * B;
        if (Math.abs(det) < 1e-10) return null;
        
        const centerX = (D * C - B * E) / det;
        const centerY = (A * E - B * D) / det;
        
        let radius = 0;
        points.forEach(point => {
            const dx = point.x - centerX;
            const dy = point.y - centerY;
            radius += Math.sqrt(dx * dx + dy * dy);
        });
        radius /= n;
        
        return {
            center: { x: centerX, y: centerY },
            radius: radius
        };
    }
    
    updateTreeList() {
        const treeList = document.getElementById('treeList');
        treeList.innerHTML = this.trees.map((tree, index) => `
            <div class="tree-item">
                <div class="tree-info">
                    <div class="tree-label">树 ${index + 1}</div>
                    <div class="tree-position">位置: (${Math.round(tree.center.x)}, ${Math.round(tree.center.y)})</div>
                </div>
                <div class="tree-diameter">${Math.round(tree.diameter)}mm</div>
            </div>
        `).join('');
    }
    
    clearResults() {
        this.trees = [];
        this.updateVisualization();
        this.updateTreeList();
        this.updateStats(this.scanData, 0);
        this.log('检测结果已清除', 'info');
    }
    
    debugData() {
        if (!this.scanData.length) {
            this.log('没有扫描数据可调试', 'warning');
            return;
        }
        
        this.log('=== 调试扫描数据 ===', 'info');
        this.log(`扫描点总数: ${this.scanData.length}`, 'info');
        
        // 统计距离分布
        const distances = this.scanData.map(point => point.distance);
        const validDistances = distances.filter(d => d > 0 && d < 5500);
        const invalidDistances = distances.filter(d => d <= 0 || d >= 5500);
        
        this.log(`有效距离点: ${validDistances.length}`, 'info');
        this.log(`无效距离点: ${invalidDistances.length}`, 'info');
        
        if (validDistances.length > 0) {
            const minDist = Math.min(...validDistances);
            const maxDist = Math.max(...validDistances);
            const avgDist = validDistances.reduce((a, b) => a + b, 0) / validDistances.length;
            
            this.log(`距离范围: ${Math.round(minDist)}mm - ${Math.round(maxDist)}mm`, 'info');
            this.log(`平均距离: ${Math.round(avgDist)}mm`, 'info');
        }
        
        // 显示前10个点的详细信息
        this.log('前10个扫描点详情:', 'info');
        for (let i = 0; i < Math.min(10, this.scanData.length); i++) {
            const point = this.scanData[i];
            this.log(`点${i+1}: 角度=${(point.angle * 180 / Math.PI).toFixed(1)}°, 距离=${point.distance}mm, 坐标=(${Math.round(point.x)}, ${Math.round(point.y)})`, 'info');
        }
        
        // 检查数据一致性
        const uniqueDistances = new Set(distances);
        this.log(`唯一距离值数量: ${uniqueDistances.size}`, 'info');
        
        if (uniqueDistances.size < distances.length * 0.1) {
            this.log('警告: 距离值重复率很高，可能数据有问题', 'warning');
        }
        
        this.log('=== 调试完成 ===', 'info');
    }
    
    calibrateDistance() {
        if (!this.scanData.length) {
            this.log('没有扫描数据可校准', 'warning');
            return;
        }
        
        this.log('=== 距离校准测试 ===', 'info');
        
        // 分析距离分布
        const distances = this.scanData.map(point => point.distance);
        const validDistances = distances.filter(d => d > 0 && d < 5500);
        
        if (validDistances.length === 0) {
            this.log('没有有效的距离数据', 'error');
            return;
        }
        
        // 统计距离分布
        const distanceStats = {
            min: Math.min(...validDistances),
            max: Math.max(...validDistances),
            avg: validDistances.reduce((a, b) => a + b, 0) / validDistances.length,
            median: this.getMedian(validDistances)
        };
        
        this.log(`距离统计: 最小=${Math.round(distanceStats.min)}mm, 最大=${Math.round(distanceStats.max)}mm, 平均=${Math.round(distanceStats.avg)}mm, 中位数=${Math.round(distanceStats.median)}mm`, 'info');
        
        // 检查距离分布是否合理
        const expectedRange = 2000; // 期望的典型测量范围
        if (distanceStats.avg < 500) {
            this.log('警告: 平均距离过小，可能解码有问题', 'warning');
        } else if (distanceStats.avg > 4000) {
            this.log('警告: 平均距离过大，可能解码有问题', 'warning');
        }
        
        // 分析角度分布
        const angles = this.scanData.map(point => point.angle * 180 / Math.PI);
        const angleStats = {
            min: Math.min(...angles),
            max: Math.max(...angles),
            range: Math.max(...angles) - Math.min(...angles)
        };
        
        this.log(`角度统计: 最小=${angleStats.min.toFixed(1)}°, 最大=${angleStats.max.toFixed(1)}°, 范围=${angleStats.range.toFixed(1)}°`, 'info');
        
        // 检查角度范围是否正确
        if (angleStats.range < 200) {
            this.log('警告: 角度范围过小，可能角度计算有问题', 'warning');
        }
        
        // 分析坐标分布
        const xCoords = this.scanData.map(point => point.x);
        const yCoords = this.scanData.map(point => point.y);
        
        const coordStats = {
            xMin: Math.min(...xCoords),
            xMax: Math.max(...xCoords),
            yMin: Math.min(...yCoords),
            yMax: Math.max(...yCoords)
        };
        
        this.log(`坐标范围: X=[${Math.round(coordStats.xMin)}, ${Math.round(coordStats.xMax)}], Y=[${Math.round(coordStats.yMin)}, ${Math.round(coordStats.yMax)}]`, 'info');
        
        // 检查坐标是否对称
        const xSymmetry = Math.abs(coordStats.xMin + coordStats.xMax);
        const ySymmetry = Math.abs(coordStats.yMin + coordStats.yMax);
        
        if (xSymmetry > 1000) {
            this.log(`警告: X坐标不对称 (差值=${Math.round(xSymmetry)})，可能角度计算有问题`, 'warning');
        }
        
        if (ySymmetry > 1000) {
            this.log(`警告: Y坐标不对称 (差值=${Math.round(ySymmetry)})，可能角度计算有问题`, 'warning');
        }
        
        // 显示前20个点的详细信息
        this.log('前20个点的详细信息:', 'info');
        for (let i = 0; i < Math.min(20, this.scanData.length); i++) {
            const point = this.scanData[i];
            this.log(`点${i+1}: 角度=${(point.angle * 180 / Math.PI).toFixed(1)}°, 距离=${point.distance}mm, 坐标=(${Math.round(point.x)}, ${Math.round(point.y)})`, 'info');
        }
        
        // 建议
        this.log('=== 校准建议 ===', 'info');
        if (distanceStats.avg < 500) {
            this.log('建议: 检查SCIP解码逻辑，距离值可能被错误解码', 'info');
        }
        if (angleStats.range < 200) {
            this.log('建议: 检查角度计算逻辑，确保从-120°到120°', 'info');
        }
        if (xSymmetry > 1000 || ySymmetry > 1000) {
            this.log('建议: 检查坐标转换逻辑，确保激光雷达在原点', 'info');
        }
        
        this.log('=== 校准测试完成 ===', 'info');
    }
    
    /**
     * 绘制高度图表
     */
    drawHeightChart(heightHistory = []) {
        if (!this.heightChartCtx || !this.heightChart) return;
        
        const ctx = this.heightChartCtx;
        const canvas = this.heightChart;
        
        // 重新获取画布尺寸，确保正确显示
        const rect = canvas.getBoundingClientRect();
        const width = rect.width;
        const height = rect.height;
        
        // 重新设置画布尺寸以匹配显示尺寸
        canvas.width = width * window.devicePixelRatio;
        canvas.height = height * window.devicePixelRatio;
        ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        
        // 设置画布显示尺寸
        canvas.style.width = width + 'px';
        canvas.style.height = height + 'px';
        
        // 清除画布
        ctx.clearRect(0, 0, width, height);
        
        if (heightHistory.length === 0) {
            // 绘制空状态
            ctx.fillStyle = '#666';
            ctx.font = '16px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('暂无高度数据', width / 2, height / 2);
            return;
        }
        
        // 计算数据范围
        const heights = heightHistory.map(h => h.height);
        const minHeight = Math.min(...heights);
        const maxHeight = Math.max(...heights);
        const heightRange = maxHeight - minHeight || 1000; // 避免除零
        
        // 设置边距
        const margin = { top: 20, right: 20, bottom: 40, left: 60 };
        const chartWidth = width - margin.left - margin.right;
        const chartHeight = height - margin.top - margin.bottom;
        
        // 绘制背景
        ctx.fillStyle = '#f8f9fa';
        ctx.fillRect(0, 0, width, height);
        
        // 绘制网格
        ctx.strokeStyle = '#e0e0e0';
        ctx.lineWidth = 1;
        
        // 水平网格线
        const gridLines = 5;
        for (let i = 0; i <= gridLines; i++) {
            const y = margin.top + (chartHeight / gridLines) * i;
            ctx.beginPath();
            ctx.moveTo(margin.left, y);
            ctx.lineTo(margin.left + chartWidth, y);
            ctx.stroke();
        }
        
        // 垂直网格线
        const timeGridLines = 10;
        for (let i = 0; i <= timeGridLines; i++) {
            const x = margin.left + (chartWidth / timeGridLines) * i;
            ctx.beginPath();
            ctx.moveTo(x, margin.top);
            ctx.lineTo(x, margin.top + chartHeight);
            ctx.stroke();
        }
        
        // 绘制高度曲线
        if (heightHistory.length > 0) {
            ctx.strokeStyle = '#3b82f6';
            ctx.lineWidth = 2;
            ctx.beginPath();
            
            heightHistory.forEach((point, index) => {
                const x = heightHistory.length > 1 ? 
                    margin.left + (chartWidth / (heightHistory.length - 1)) * index :
                    margin.left + chartWidth / 2; // 单点时居中显示
                const y = margin.top + chartHeight - ((point.height - minHeight) / heightRange) * chartHeight;
                
                if (index === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            });
            
            ctx.stroke();
            
            // 绘制数据点
            ctx.fillStyle = '#3b82f6';
            heightHistory.forEach((point, index) => {
                const x = heightHistory.length > 1 ? 
                    margin.left + (chartWidth / (heightHistory.length - 1)) * index :
                    margin.left + chartWidth / 2; // 单点时居中显示
                const y = margin.top + chartHeight - ((point.height - minHeight) / heightRange) * chartHeight;
                
                ctx.beginPath();
                ctx.arc(x, y, 4, 0, 2 * Math.PI);
                ctx.fill();
                
                // 绘制数据点标签（仅显示最近几个点）
                if (index >= heightHistory.length - 3) {
                    ctx.fillStyle = '#333';
                    ctx.font = '10px Arial';
                    ctx.textAlign = 'center';
                    ctx.fillText(`${Math.round(point.height)}mm`, x, y - 10);
                    ctx.fillStyle = '#3b82f6';
                }
            });
        }
        
        // 绘制Y轴标签
        ctx.fillStyle = '#666';
        ctx.font = '12px Arial';
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        
        for (let i = 0; i <= gridLines; i++) {
            const value = maxHeight - (heightRange / gridLines) * i;
            const y = margin.top + (chartHeight / gridLines) * i;
            ctx.fillText(`${Math.round(value)}mm`, margin.left - 10, y);
        }
        
        // 绘制X轴标签
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        
        if (heightHistory.length > 1) {
            const timeLabels = 5;
            for (let i = 0; i <= timeLabels; i++) {
                const index = Math.floor((heightHistory.length - 1) / timeLabels) * i;
                if (index < heightHistory.length) {
                    const x = margin.left + (chartWidth / timeLabels) * i;
                    const time = new Date(heightHistory[index].timestamp);
                    const timeStr = `${time.getHours().toString().padStart(2, '0')}:${time.getMinutes().toString().padStart(2, '0')}:${time.getSeconds().toString().padStart(2, '0')}`;
                    ctx.fillText(timeStr, x, margin.top + chartHeight + 10);
                }
            }
        }
        
        // 绘制标题
        ctx.fillStyle = '#333';
        ctx.font = 'bold 14px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('高度变化趋势', width / 2, 15);
        
        // 绘制图例
        ctx.fillStyle = '#3b82f6';
        ctx.fillRect(width - 120, 10, 15, 15);
        ctx.fillStyle = '#333';
        ctx.font = '12px Arial';
        ctx.textAlign = 'left';
        ctx.fillText('高度 (mm)', width - 100, 22);
    }
    
    /**
     * 导出高度数据
     */
    exportHeightData() {
        if (!this.stp23lSensor.heightHistory.length) {
            this.log('没有高度数据可导出', 'warning');
            return;
        }
        
        const data = this.stp23lSensor.heightHistory.map((point, index) => 
            `${index + 1}\t${point.height}\t${new Date(point.timestamp).toISOString()}\t${point.noise}\t${point.confidence}`
        ).join('\n');
        
        const header = '序号\t高度(mm)\t时间戳\t噪声\t置信度\n';
        const csvData = header + data;
        
        const blob = new Blob([csvData], { type: 'text/plain;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `height_data_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`;
        a.click();
        URL.revokeObjectURL(url);
        
        this.log('高度数据已导出', 'success');
    }
    
    /**
     * 重置高度图表
     */
    resetHeightChart() {
        this.stp23lSensor.clearHeightData();
        this.drawHeightChart();
        this.log('高度图表已重置', 'info');
    }
    
    /**
     * 生成测试高度数据（用于调试图表显示）
     */
    generateTestHeightData() {
        this.log('生成测试高度数据...', 'info');
        
        // 清除现有数据
        this.stp23lSensor.clearHeightData();
        
        // 生成模拟数据
        const baseHeight = 1500; // 基础高度1.5米
        const dataPoints = 20;
        
        for (let i = 0; i < dataPoints; i++) {
            const height = baseHeight + Math.sin(i * 0.5) * 200 + Math.random() * 50;
            const timestamp = Date.now() - (dataPoints - i) * 1000; // 每秒一个数据点
            
            this.stp23lSensor.heightHistory.push({
                height: height,
                timestamp: timestamp,
                noise: Math.random() * 10,
                confidence: 80 + Math.random() * 20
            });
        }
        
        // 更新显示
        this.stp23lSensor.updateDisplay();
        this.drawHeightChart(this.stp23lSensor.heightHistory);
        
        this.log(`已生成 ${dataPoints} 个测试数据点`, 'success');
    }
    
    getMedian(arr) {
        const sorted = [...arr].sort((a, b) => a - b);
        const mid = Math.floor(sorted.length / 2);
        return sorted.length % 2 !== 0 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
    }

    async importData() {
        const fileInput = document.getElementById('importFile');
        const file = fileInput.files[0];
        
        if (!file) {
            this.log('请选择要导入的文件', 'warning');
            return;
        }
        
        try {
            const text = await file.text();
            const lines = text.trim().split('\n');
            
            this.scanData = [];
            lines.forEach((line, index) => {
                const values = line.trim().split(/\s+/);
                if (values.length >= 2) {
                    const distance = parseFloat(values[0]);
                    const angle = parseFloat(values[1]) * Math.PI / 180;
                    
                    this.scanData.push({
                        angle: angle,
                        distance: distance,
                        x: distance * Math.cos(angle),
                        y: distance * Math.sin(angle)
                    });
                }
            });
            
            this.updateVisualization();
            this.updateButtons();
            this.log(`成功导入 ${this.scanData.length} 个数据点`, 'success');
            
        } catch (error) {
            this.log(`导入失败: ${error.message}`, 'error');
        }
    }
    
    exportData() {
        if (!this.scanData.length) {
            this.log('没有数据可导出', 'warning');
            return;
        }
        
        const data = this.scanData.map(point => 
            `${point.distance}\t${point.angle * 180 / Math.PI}\t${point.x}\t${point.y}`
        ).join('\n');
        
        const blob = new Blob([data], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `scan_data_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`;
        a.click();
        URL.revokeObjectURL(url);
        
        this.log('数据已导出', 'success');
    }
    
    exportTrees() {
        if (!this.trees.length) {
            this.log('没有树木数据可导出', 'warning');
            return;
        }
        
        const data = this.trees.map((tree, index) => 
            `树${index + 1}\t${Math.round(tree.center.x)}\t${Math.round(tree.center.y)}\t${Math.round(tree.diameter)}`
        ).join('\n');
        
        const blob = new Blob([data], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `trees_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`;
        a.click();
        URL.revokeObjectURL(url);
        
        this.log('树木数据已导出', 'success');
    }
    
    updateStats(scanData, scanTime) {
        this.log(`开始更新统计信息: scanData.length=${scanData ? scanData.length : 'null'}, scanTime=${scanTime}`, 'info');
        
        // 更新各个统计元素
        const treeCountElement = document.getElementById('treeCount');
        const scanPointsElement = document.getElementById('scanPoints');
        const avgDiameterElement = document.getElementById('avgDiameter');
        const scanTimeElement = document.getElementById('scanTime');
        const avgDistanceElement = document.getElementById('avgDistance');
        const maxDistanceElement = document.getElementById('maxDistance');
        
        this.log(`找到DOM元素: treeCount=${!!treeCountElement}, scanPoints=${!!scanPointsElement}, avgDiameter=${!!avgDiameterElement}, scanTime=${!!scanTimeElement}, avgDistance=${!!avgDistanceElement}, maxDistance=${!!maxDistanceElement}`, 'info');
        
        if (treeCountElement) {
            treeCountElement.textContent = this.trees.length;
            this.log(`更新树木数量: ${this.trees.length}`, 'info');
        }
        
        if (scanPointsElement) {
            scanPointsElement.textContent = scanData ? scanData.length : 0;
            this.log(`更新扫描点数: ${scanData ? scanData.length : 0}`, 'info');
        }
        
        if (scanTimeElement) {
            scanTimeElement.textContent = scanTime.toFixed(1);
            this.log(`更新扫描时间: ${scanTime.toFixed(1)}ms`, 'info');
        }
        
        // 计算平均直径
        if (avgDiameterElement) {
            if (this.trees.length > 0) {
                const totalDiameter = this.trees.reduce((sum, tree) => sum + tree.diameter, 0);
                const avgDiameter = totalDiameter / this.trees.length;
                avgDiameterElement.textContent = Math.round(avgDiameter);
                this.log(`更新平均直径: ${Math.round(avgDiameter)}mm`, 'info');
            } else {
                avgDiameterElement.textContent = '0';
                this.log(`更新平均直径: 0mm (无树木)`, 'info');
            }
        }
        
        // 计算距离统计
        if (scanData && scanData.length > 0) {
            const validPoints = scanData.filter(point => 
                point.distance > this.scanParams.minRange && 
                point.distance < this.scanParams.maxRange
            );
            
            if (avgDistanceElement) {
                if (validPoints.length > 0) {
                    const totalDistance = validPoints.reduce((sum, point) => sum + point.distance, 0);
                    const avgDistance = totalDistance / validPoints.length;
                    avgDistanceElement.textContent = Math.round(avgDistance);
                    this.log(`更新平均距离: ${Math.round(avgDistance)}mm`, 'info');
                } else {
                    avgDistanceElement.textContent = '0';
                    this.log(`更新平均距离: 0mm (无有效点)`, 'info');
                }
            }
            
            if (maxDistanceElement) {
                if (validPoints.length > 0) {
                    const maxDistance = Math.max(...validPoints.map(point => point.distance));
                    maxDistanceElement.textContent = Math.round(maxDistance);
                    this.log(`更新最大距离: ${Math.round(maxDistance)}mm`, 'info');
                } else {
                    maxDistanceElement.textContent = '0';
                    this.log(`更新最大距离: 0mm (无有效点)`, 'info');
                }
            }
            
            // 记录统计信息到日志
            this.log(`统计更新完成: 扫描点${scanData.length}, 有效点${validPoints.length}, 树木${this.trees.length}`, 'success');
        } else {
            this.log(`统计更新失败: scanData为空或无效`, 'error');
        }
    }
    
    updateStatus(text, type) {
        const statusText = document.getElementById('statusText');
        const statusDot = document.getElementById('statusDot');
        
        if (statusText) {
            statusText.textContent = text;
        }
        
        if (statusDot) {
            statusDot.className = `status-dot status-${type}`;
        }
    }

    updateButtons() {
        const connectBtn = document.getElementById('connectBtn');
        const disconnectBtn = document.getElementById('disconnectBtn');
        const startScanBtn = document.getElementById('startScanBtn');
        const pauseScanBtn = document.getElementById('pauseScanBtn');
        const singleScanBtn = document.getElementById('singleScanBtn');
        const detectTreesBtn = document.getElementById('detectTreesBtn');
        const debugBtn = document.getElementById('debugBtn');
        const calibrateBtn = document.getElementById('calibrateBtn');
        
        if (connectBtn) connectBtn.disabled = this.isConnected;
        if (disconnectBtn) disconnectBtn.disabled = !this.isConnected;
        if (startScanBtn) startScanBtn.disabled = !this.isConnected || this.isScanning;
        if (pauseScanBtn) pauseScanBtn.disabled = !this.isConnected || !this.isScanning;
        if (singleScanBtn) singleScanBtn.disabled = !this.isConnected;
        
        // 树木检测按钮：只要有扫描数据就可以使用
        const hasScanData = this.scanData && this.scanData.length > 0;
        if (detectTreesBtn) {
            detectTreesBtn.disabled = !hasScanData;
            console.log(`树木检测按钮状态更新: ${hasScanData ? '启用' : '禁用'} (数据点数: ${this.scanData ? this.scanData.length : 0})`);
        }
        
        if (debugBtn) debugBtn.disabled = !hasScanData;
        if (calibrateBtn) calibrateBtn.disabled = !hasScanData;
    }

    log(message, type = 'info') {
        const logArea = document.getElementById('logArea');
        if (!logArea) return;
        
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;
        logEntry.innerHTML = `<span class="log-time">[${timestamp}]</span> <span class="log-message">${message}</span>`;
        
        logArea.appendChild(logEntry);
        logArea.scrollTop = logArea.scrollHeight;
        
        // 限制日志条数
        const maxLogs = 100;
        while (logArea.children.length > maxLogs) {
            logArea.removeChild(logArea.firstChild);
        }
    }
}

// 初始化系统
document.addEventListener('DOMContentLoaded', () => {
    console.log('DOM已加载，开始初始化激光雷达系统...');
    // 延迟初始化，确保所有DOM元素都已加载
    setTimeout(() => {
        try {
            window.lidarSystem = new LidarSystem();
            console.log('激光雷达系统初始化完成:', window.lidarSystem);
        } catch (error) {
            console.error('激光雷达系统初始化失败:', error);
        }
    }, 100);
});