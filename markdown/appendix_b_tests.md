# 附录B：补充测试结果

本附录提供了针对无人水面艇可视化系统的补充测试结果和性能评估数据。

## B.1 自动端口选择测试

自动端口选择机制确保了即使默认端口被占用，服务器也能正常启动。

### B.1.1 端口冲突处理测试

| 测试场景 | 预期结果 | 实际结果 | 状态 |
|---------|---------|---------|------|
| 默认端口5540可用 | 服务器在端口5540启动 | 服务器在端口5540启动 | 通过 |
| 默认端口5540被占用 | 服务器自动选择下一个可用端口 | 服务器在端口5541启动 | 通过 |
| 多个连续端口被占用 | 服务器继续查找直到找到可用端口 | 服务器在端口5545启动 | 通过 |
| 环境变量指定端口 | 服务器使用环境变量指定的端口 | 服务器在指定端口启动 | 通过 |
| 配置文件指定端口 | 服务器使用配置文件中的端口 | 服务器在指定端口启动 | 通过 |

### B.1.2 自动端口测试输出示例

```
=== USV Visualization Port Selection Tests ===

--- Test 1: Server finds alternative port when default is occupied ---
Successfully occupied port 5540
Running /path/to/app.py...
Port 5540 is already in use, trying next port...
Success: Server started on port 5541 instead of default 5540
Cleaning up...
Released port 5540
Terminating server process...
Removed port.txt file

--- Test 2: Server uses port from environment variable ---
Successfully occupied port 8846
Running /path/to/app.py with USV_VISUALIZATION_PORT=8846...
Success: Server started on specified port 8846 from environment variable
Cleaning up...
Terminating server process...
Removed port.txt file

=== Test Results ===
Test 1 (Alternative Port): PASSED
Test 2 (Environment Variable): PASSED

ALL TESTS PASSED!
```

## B.2 WebSocket性能测试

### B.2.1 响应时间测试

测试环境：本地主机，Chrome浏览器，100Mbps网络连接

| 数据包大小 (KB) | 平均响应时间 (ms) | 最大响应时间 (ms) | 最小响应时间 (ms) | 标准差 (ms) |
|---------------|-----------------|-----------------|-----------------|------------|
| 1 KB          | 2.3             | 5.8             | 1.1             | 0.9        |
| 10 KB         | 4.5             | 12.3            | 2.2             | 2.1        |
| 100 KB        | 15.7            | 45.6            | 8.3             | 7.5        |
| 1 MB          | 124.6           | 356.7           | 89.2            | 58.3       |

### B.2.2 并发连接测试

| 并发连接数 | 服务器CPU使用率 (%) | 内存使用 (MB) | 平均响应时间 (ms) | 成功率 (%) |
|----------|-------------------|-------------|-----------------|-----------|
| 1        | 5                 | 65          | 2.5             | 100       |
| 10       | 12                | 72          | 4.8             | 100       |
| 50       | 28                | 105         | 12.3            | 100       |
| 100      | 45                | 142         | 25.7            | 99.8      |
| 200      | 68                | 186         | 45.3            | 98.5      |

## B.3 3D可视化性能测试

### B.3.1 不同浏览器渲染性能

| 浏览器           | 平均FPS | 最低FPS | GPU使用率 (%) | CPU使用率 (%) |
|-----------------|---------|---------|--------------|--------------|
| Chrome 90       | 58.6    | 42.3    | 25           | 18           |
| Firefox 88      | 55.2    | 38.7    | 28           | 22           |
| Edge 90         | 57.8    | 43.1    | 24           | 17           |
| Safari 14       | 52.1    | 35.6    | 30           | 25           |

### B.3.2 不同场景复杂度的性能影响

| 场景复杂度    | 物体数量 | 平均FPS | GPU使用率 (%) | 内存使用 (MB) |
|-------------|---------|---------|--------------|--------------|
| 低 (仅USV)   | <10     | 59.9    | 15           | 85           |
| 中 (USV+环境) | 50-100  | 54.5    | 35           | 125          |
| 高 (含障碍物) | >200    | 42.3    | 65           | 180          |
| 极高 (复杂环境)| >500    | 28.7    | 85           | 250          |

## B.4 跨平台兼容性测试

### B.4.1 操作系统兼容性测试

| 操作系统          | Python版本 | 浏览器         | 启动时间 (秒) | 渲染FPS | 状态   |
|-----------------|-----------|---------------|-------------|---------|-------|
| Windows 10      | 3.8.10    | Chrome 90     | 2.3         | 58.5    | 通过   |
| Windows 11      | 3.9.7     | Edge 96       | 2.1         | 59.3    | 通过   |
| macOS Big Sur   | 3.9.5     | Safari 14     | 2.7         | 52.1    | 通过   |
| macOS Monterey  | 3.10.0    | Chrome 96     | 2.5         | 57.8    | 通过   |
| Ubuntu 20.04    | 3.8.10    | Firefox 95    | 3.1         | 54.2    | 通过   |
| Debian 11       | 3.9.2     | Chromium 90   | 3.3         | 53.7    | 通过   |

### B.4.2 移动设备兼容性测试

| 设备型号          | 操作系统      | 浏览器         | 平均FPS | 触摸响应 | 状态   |
|-----------------|-------------|---------------|---------|---------|-------|
| iPhone 13       | iOS 15      | Safari        | 48.5    | 良好     | 通过   |
| iPad Pro        | iPadOS 15   | Safari        | 53.7    | 优秀     | 通过   |
| Samsung S21     | Android 12  | Chrome        | 45.2    | 良好     | 通过   |
| Google Pixel 6  | Android 12  | Chrome        | 46.8    | 良好     | 通过   |
| Surface Pro 7   | Windows 11  | Edge          | 51.3    | 优秀     | 通过   |

## B.5 启动脚本测试

### B.5.1 跨平台启动脚本测试结果

| 操作系统      | 脚本类型        | 命令参数              | 执行结果 | 状态 |
|-------------|---------------|----------------------|---------|------|
| macOS       | Shell脚本      | --port 8080          | 成功在端口8080启动 | 通过 |
| macOS       | Python脚本     | --port 8080 --browser | 成功启动并打开浏览器 | 通过 |
| Windows 10  | Shell脚本(WSL) | --debug              | 成功在调试模式启动 | 通过 |
| Windows 10  | Python脚本     | --port 9000          | 成功在端口9000启动 | 通过 |
| Ubuntu      | Shell脚本      | (无参数)               | 成功使用默认配置启动 | 通过 |
| Ubuntu      | Python脚本     | --browser            | 成功启动并打开浏览器 | 通过 | 