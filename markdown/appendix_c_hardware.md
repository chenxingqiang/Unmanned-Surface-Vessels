# 附录C：硬件规格

本附录提供了无人水面艇可视化系统开发和测试所用的硬件规格，以及推荐的运行环境配置。

## C.1 开发环境硬件规格

### C.1.1 开发工作站配置

| 组件     | 规格                                     | 说明                       |
|---------|----------------------------------------|----------------------------|
| 处理器   | Intel Core i7-11700K (8核16线程, 3.6GHz) | 支持高性能计算与多线程模拟   |
| 内存     | 32GB DDR4 3200MHz                      | 足够处理大规模模拟数据       |
| 显卡     | NVIDIA GeForce RTX 3070 (8GB GDDR6)    | 支持3D可视化和GPU加速计算   |
| 存储     | 1TB NVMe SSD                           | 高速文件读写                |
| 操作系统 | Ubuntu 20.04 LTS / Windows 10 Pro       | 双系统支持不同开发场景      |
| 显示器   | 27英寸 4K显示器 (3840x2160)              | 提供高分辨率可视化界面      |
| 网络     | 1Gbps以太网 + WiFi 6                    | 支持远程操作和高速数据传输   |

### C.1.2 便携式开发设备

| 组件     | 规格                                      | 说明                       |
|---------|------------------------------------------|----------------------------|
| 型号     | MacBook Pro 16" (2021)                   | 便携式开发和演示            |
| 处理器   | Apple M1 Pro (10核CPU)                   | 高性能低功耗                |
| 内存     | 32GB 统一内存                             | 支持多任务开发环境          |
| 存储     | 1TB SSD                                  | 存储代码和测试数据          |
| 显示器   | 16英寸Liquid Retina XDR (3456 x 2234)    | 高分辨率显示                |
| 操作系统 | macOS Monterey                           | 支持Unix开发环境            |
| 网络     | WiFi 6 + Bluetooth 5.0                   | 无线连接与测试              |

## C.2 服务器环境规格

### C.2.1 开发服务器

| 组件     | 规格                                      | 说明                         |
|---------|------------------------------------------|------------------------------|
| 处理器   | Intel Xeon E5-2680 v4 (14核28线程, 2.4GHz)| 多核心支持并行计算与多用户访问 |
| 内存     | 64GB DDR4 ECC                            | 支持多个并行模拟实例           |
| 存储     | 2TB NVMe SSD + 8TB HDD RAID 5            | 系统与数据分离存储            |
| 网络     | 10Gbps以太网 (双端口)                      | 高带宽数据传输                |
| 操作系统 | Ubuntu Server 20.04 LTS                   | 稳定的服务器环境              |

### C.2.2 云部署规格

| 服务提供商    | 实例类型                 | 规格                                        |
|--------------|--------------------------|---------------------------------------------|
| AWS          | EC2 c5.2xlarge           | 8 vCPU, 16GB RAM, 高达10Gbps网络            |
| Google Cloud | n2-standard-8            | 8 vCPU, 32GB RAM, 10Gbps网络                |
| Azure        | Standard_F8s_v2          | 8 vCPU, 16GB RAM, 高级SSD, 最大磁盘吞吐量     |
| 阿里云        | ecs.c6.2xlarge          | 8 vCPU, 16GB RAM, 2.5Gbps带宽               |

## C.3 测试环境规格

### C.3.1 客户端测试设备

| 设备类型      | 型号                           | 操作系统              | 浏览器               |
|--------------|--------------------------------|----------------------|---------------------|
| 台式计算机    | Dell XPS 8940                  | Windows 11 Pro       | Chrome 96, Edge 96  |
| 笔记本电脑    | Lenovo ThinkPad X1 Carbon     | Windows 10 Pro       | Firefox 95, Edge 90 |
| 苹果台式机    | iMac 27" (2020)               | macOS Big Sur        | Safari 15, Chrome 95|
| 苹果笔记本    | MacBook Air M1                | macOS Monterey       | Safari 15           |
| Linux工作站   | System76 Thelio               | Ubuntu 20.04 LTS     | Firefox 94          |

### C.3.2 移动设备测试

| 设备类型      | 型号                           | 操作系统              | 屏幕分辨率           |
|--------------|--------------------------------|----------------------|---------------------|
| iOS手机      | iPhone 13 Pro                  | iOS 15.1             | 1170 x 2532         |
| iOS平板      | iPad Pro 12.9" (2021)          | iPadOS 15.1          | 2732 x 2048         |
| Android手机  | Samsung Galaxy S21 Ultra       | Android 12           | 1440 x 3200         |
| Android平板  | Samsung Galaxy Tab S7+         | Android 11           | 2800 x 1752         |
| Windows平板  | Microsoft Surface Pro 8        | Windows 11           | 2880 x 1920         |

## C.4 推荐运行环境

### C.4.1 最低硬件要求

| 组件     | 最低规格                                 | 建议规格                                |
|---------|------------------------------------------|---------------------------------------|
| 处理器   | 双核CPU, 2.0GHz以上                      | 四核CPU, 2.5GHz以上                    |
| 内存     | 4GB RAM                                  | 8GB RAM                               |
| 显卡     | 集成显卡, 支持WebGL                       | 独立显卡, 2GB显存以上                   |
| 存储     | 100MB可用空间                            | 500MB可用空间                          |
| 网络     | 10Mbps互联网连接                         | 50Mbps互联网连接                       |
| 显示器   | 1366 x 768分辨率                         | 1920 x 1080或更高分辨率                |

### C.4.2 软件环境要求

| 组件     | 最低版本                                 | 建议版本                                |
|---------|------------------------------------------|---------------------------------------|
| Python  | Python 3.7+                              | Python 3.9+                           |
| 操作系统 | Windows 8.1, macOS Catalina, Ubuntu 18.04| Windows 10/11, macOS Big Sur, Ubuntu 20.04 |
| 浏览器   | Chrome 80+, Firefox 80+, Safari 14+      | 最新版Chrome, Firefox, Edge或Safari    |
| 网络     | 支持WebSocket                            | 支持WebSocket和HTTP/2                  |

### C.4.3 移动设备要求

| 组件     | 最低规格                                 | 建议规格                                |
|---------|------------------------------------------|---------------------------------------|
| iOS     | iOS 13+, iPhone 8或更新机型               | iOS 15+, iPhone 11或更新机型            |
| Android | Android 9.0+, 4GB RAM                    | Android 11.0+, 6GB RAM                |
| 平板     | iPad Air 2, Galaxy Tab S5e或同等设备      | iPad Pro, Galaxy Tab S7或更高端设备     |
| 网络     | WiFi 802.11n                            | WiFi 802.11ac或WiFi 6                 |

## C.5 硬件性能对可视化系统的影响

硬件性能对无人水面艇可视化系统有显著影响，特别是以下几个方面：

1. **3D渲染性能**：GPU性能直接影响3D场景渲染的流畅度和细节水平。高端GPU可以支持更复杂的水面效果和更多的场景对象。

2. **模拟计算速度**：CPU性能影响物理模拟计算速度，特别是在复杂环境或多艇协同模拟场景中。

3. **响应时间**：服务器硬件性能和网络带宽影响从模拟计算到前端显示的延迟。

4. **并发连接能力**：在多用户使用场景下，服务器内存和CPU核心数决定了系统可以同时支持的连接数量。

5. **移动设备兼容性**：移动设备的GPU性能和屏幕分辨率影响移动端可视化体验质量。

综合测试表明，推荐配置下的系统可以实现60FPS的稳定帧率，即使在复杂场景中也能保持30FPS以上的性能，保证良好的用户体验。 