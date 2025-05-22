# 附录A：代码实现

本附录提供了无人水面艇智能导航与控制系统的核心算法Python实现。

## A.1 运动学模型实现

```python
def update_kinematics(self, u: np.ndarray) -> np.ndarray:
    """
    基于运动学模型更新USV状态。

    参数:
        u: 控制输入向量 [u, v, r] - 纵向速度、横向速度和偏航角速度

    返回:
        更新后的状态向量 [x, y, psi]
    """
    # 提取当前状态
    x, y, psi = self.state[:3]
    
    # 构建旋转矩阵
    R = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    
    # 计算状态导数
    state_dot = R @ u
    
    # 使用欧拉法积分
    new_state = self.state[:3] + state_dot * self.dt
    
    return new_state
```

## A.2 动力学模型实现

```python
def update_dynamics(self, F: np.ndarray) -> np.ndarray:
    """
    基于动力学模型更新USV速度。

    参数:
        F: 控制力向量 [X, Y, N] - 纵向力、横向力和偏航力矩

    返回:
        更新后的速度向量 [u, v, r]
    """
    # 提取当前速度
    u, v, r = self.vel
    
    # 计算科氏力和向心力
    C = np.array([
        [0, -self.m * r, -self.m * v],
        [self.m * r, 0, self.m * u],
        [self.m * v, -self.m * u, 0]
    ])
    
    # 计算阻尼力
    D = np.array([
        [self.Du, 0, 0],
        [0, self.Dv, 0],
        [0, 0, self.Dr]
    ])
    
    # 构建质量和惯性矩阵
    M = np.array([
        [self.m, 0, 0],
        [0, self.m, 0],
        [0, 0, self.Iz]
    ])
    
    # 计算加速度: M^(-1) * (F - C*v - D*v)
    Minv = np.linalg.inv(M)
    vel_dot = Minv @ (F - C @ self.vel - D @ self.vel)
    
    # 使用欧拉法积分
    new_vel = self.vel + vel_dot * self.dt
    
    return new_vel
```

## A.3 PID控制器实现

```python
def pid_control(self, desired_state: np.ndarray) -> np.ndarray:
    """
    使用PID控制器计算控制输入。

    参数:
        desired_state: 期望状态 [x_d, y_d, psi_d]

    返回:
        控制输入向量 [F_surge, F_sway, M_yaw]
    """
    # 提取当前状态和速度
    x, y, psi = self.state[:3]
    u, v, r = self.vel
    
    # 提取目标状态
    x_d, y_d, psi_d = desired_state
    
    # 计算位置误差
    pos_error = np.array([x_d - x, y_d - y])
    
    # 转换到船体坐标系
    R_inv = np.array([
        [np.cos(psi), np.sin(psi)],
        [-np.sin(psi), np.cos(psi)]
    ])
    
    pos_error_body = R_inv @ pos_error
    
    # 计算航向误差，确保在 -pi 到 pi 之间
    psi_error = np.arctan2(np.sin(psi_d - psi), np.cos(psi_d - psi))
    
    # 更新积分误差
    self.pos_error_int += pos_error_body * self.dt
    self.psi_error_int += psi_error * self.dt
    
    # 计算微分误差
    pos_error_dot = (pos_error_body - self.prev_pos_error) / self.dt
    psi_error_dot = (psi_error - self.prev_psi_error) / self.dt
    
    # 保存当前误差以供下次使用
    self.prev_pos_error = pos_error_body.copy()
    self.prev_psi_error = psi_error
    
    # 计算前向力（PID控制）
    F_surge = (self.Kp_surge * pos_error_body[0] + 
               self.Ki_surge * self.pos_error_int[0] + 
               self.Kd_surge * pos_error_dot[0])
    
    # 计算侧向力（PD控制，通常不直接控制）
    F_sway = 0.0  # USV通常是欠驱动的，没有侧向推进器
    
    # 计算偏航力矩（PID控制）
    M_yaw = (self.Kp_yaw * psi_error + 
             self.Ki_yaw * self.psi_error_int + 
             self.Kd_yaw * psi_error_dot)
    
    # 组合控制输入
    control_input = np.array([F_surge, F_sway, M_yaw])
    
    return control_input
```

## A.4 改进A*算法实现

```python
class ImprovedAStar:
    """改进的A*路径规划算法"""
    
    def __init__(self, map_data, resolution=1.0, safety_distance=5.0):
        """
        初始化路径规划器。
        
        参数:
            map_data: 地图数据，包含障碍物信息
            resolution: 网格分辨率（米/网格）
            safety_distance: 与障碍物的安全距离（米）
        """
        self.map_data = map_data
        self.resolution = resolution
        self.safety_distance = safety_distance
        self.grid = self._create_grid()
        
    def _create_grid(self):
        """创建网格地图，包含障碍物和安全距离"""
        # 地图尺寸
        width = int(self.map_data.width / self.resolution)
        height = int(self.map_data.height / self.resolution)
        
        # 初始化网格
        grid = np.zeros((height, width))
        
        # 标记障碍物和安全区域
        for obstacle in self.map_data.obstacles:
            x, y = obstacle.position
            radius = obstacle.size + self.safety_distance
            
            # 障碍物坐标转网格索引
            grid_x = int(x / self.resolution)
            grid_y = int(y / self.resolution)
            
            # 标记障碍物及周围安全区域
            safety_cells = int(radius / self.resolution)
            for i in range(-safety_cells, safety_cells + 1):
                for j in range(-safety_cells, safety_cells + 1):
                    if (0 <= grid_y + i < height and 0 <= grid_x + j < width):
                        # 计算到障碍物中心的距离
                        dist = np.sqrt(i**2 + j**2) * self.resolution
                        if dist <= radius:
                            # 设置成本值（距离障碍物越近，成本越高）
                            cost = 1.0 - dist/radius if dist < radius else 0
                            grid[grid_y + i, grid_x + j] = max(grid[grid_y + i, grid_x + j], cost)
        
        return grid
    
    def plan_path(self, start, goal):
        """
        使用改进的A*算法规划路径
        
        参数:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
            
        返回:
            路径点列表，从起点到终点
        """
        # 将坐标转换为网格索引
        start_grid = (int(start[1] / self.resolution), int(start[0] / self.resolution))
        goal_grid = (int(goal[1] / self.resolution), int(goal[0] / self.resolution))
        
        # 检查起点和终点是否可达
        if self.grid[start_grid] > 0.8 or self.grid[goal_grid] > 0.8:
            print("起点或终点在障碍物内！")
            return None
        
        # 初始化开集和闭集
        open_set = PriorityQueue()
        closed_set = set()
        
        # 路径跟踪字典
        came_from = {}
        
        # 初始化成本
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        # 将起点加入开集
        open_set.put((f_score[start_grid], start_grid))
        
        while not open_set.empty():
            # 获取当前f值最小的节点
            _, current = open_set.get()
            
            # 如果到达目标，重建路径
            if current == goal_grid:
                path = self._reconstruct_path(came_from, current)
                return self._smooth_path(path)
            
            # 将当前节点加入闭集
            closed_set.add(current)
            
            # 探索相邻节点
            for next_node in self._get_neighbors(current):
                # 如果相邻节点在闭集中，跳过
                if next_node in closed_set:
                    continue
                
                # 计算到达相邻节点的成本
                tentative_g_score = g_score[current] + self._movement_cost(current, next_node)
                
                # 如果找到了更好的路径或者这个节点是新的
                if next_node not in g_score or tentative_g_score < g_score[next_node]:
                    # 记录新的最佳路径和成本
                    came_from[next_node] = current
                    g_score[next_node] = tentative_g_score
                    f_score[next_node] = tentative_g_score + self._heuristic(next_node, goal_grid)
                    
                    # 将相邻节点加入开集
                    open_set.put((f_score[next_node], next_node))
        
        # 如果开集为空但未找到路径，返回None
        return None
    
    def _heuristic(self, a, b):
        """计算两点间的启发式距离（欧几里得距离）"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _movement_cost(self, a, b):
        """计算从a移动到b的成本，考虑障碍物成本"""
        # 基础移动成本（欧几里得距离）
        base_cost = self._heuristic(a, b)
        
        # 障碍物成本（a和b的平均值）
        obstacle_cost = (self.grid[a] + self.grid[b]) / 2
        
        # 总成本 = 基础成本 + 障碍物权重 * 障碍物成本
        return base_cost + 10.0 * obstacle_cost
    
    def _get_neighbors(self, node):
        """获取节点的相邻节点（8个方向）"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # 跳过自身
                
                neighbor = (node[0] + dy, node[1] + dx)
                
                # 检查边界
                if (0 <= neighbor[0] < self.grid.shape[0] and 
                    0 <= neighbor[1] < self.grid.shape[1]):
                    # 检查是否为障碍物（障碍物成本阈值设为0.8）
                    if self.grid[neighbor] < 0.8:
                        neighbors.append(neighbor)
        
        return neighbors
    
    def _reconstruct_path(self, came_from, current):
        """从came_from字典中重建路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        # 将路径从网格索引转换回实际坐标
        path = [(node[1] * self.resolution, node[0] * self.resolution) for node in reversed(path)]
        return path
    
    def _smooth_path(self, path):
        """使用B样条平滑路径"""
        if path is None or len(path) < 3:
            return path
        
        # 创建B样条
        x = np.array([p[0] for p in path])
        y = np.array([p[1] for p in path])
        
        # 参数化路径点
        t = np.zeros(len(path))
        for i in range(1, len(path)):
            t[i] = t[i-1] + np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
        
        t = t / t[-1]  # 归一化参数
        
        # 创建B样条拟合
        tck_x = interpolate.splrep(t, x, s=0)
        tck_y = interpolate.splrep(t, y, s=0)
        
        # 生成平滑路径
        t_new = np.linspace(0, 1, num=100)
        x_smooth = interpolate.splev(t_new, tck_x, der=0)
        y_smooth = interpolate.splev(t_new, tck_y, der=0)
        
        smooth_path = list(zip(x_smooth, y_smooth))
        return smooth_path
```

## A.5 动态窗口法实现

```python
class DynamicWindowApproach:
    """动态窗口避障算法"""
    
    def __init__(self, config):
        """
        初始化DWA算法。
        
        参数:
            config: 配置参数字典
        """
        # 机器人参数
        self.max_speed = config['max_speed']         # 最大速度 [m/s]
        self.min_speed = config['min_speed']         # 最小速度 [m/s]
        self.max_yawrate = config['max_yawrate']     # 最大偏航角速度 [rad/s]
        self.max_accel = config['max_accel']         # 最大加速度 [m/s^2]
        self.max_dyawrate = config['max_dyawrate']   # 最大偏航角加速度 [rad/s^2]
        self.v_reso = config['v_reso']               # 速度分辨率 [m/s]
        self.yawrate_reso = config['yawrate_reso']   # 偏航角速度分辨率 [rad/s]
        self.dt = config['dt']                       # 时间步长 [s]
        self.predict_time = config['predict_time']   # 轨迹预测时间 [s]
        self.to_goal_cost_gain = config['to_goal_cost_gain']  # 目标代价增益
        self.speed_cost_gain = config['speed_cost_gain']      # 速度代价增益
        self.obstacle_cost_gain = config['obstacle_cost_gain'] # 障碍物代价增益
        self.robot_radius = config['robot_radius']            # 机器人半径 [m]
    
    def plan(self, x, goal, ob):
        """
        DWA算法规划路径。
        
        参数:
            x: 当前状态 [x(m), y(m), yaw(rad), v(m/s), yaw_rate(rad/s)]
            goal: 目标点 [x(m), y(m)]
            ob: 障碍物列表 [[x(m), y(m)], ...]
            
        返回:
            最优速度控制 [v(m/s), yaw_rate(rad/s)]，轨迹
        """
        # 动态窗口计算
        dw = self._calc_dynamic_window(x)
        
        # 轨迹评估
        u, trajectory = self._eval_trajectory(x, dw, goal, ob)
        
        return u, trajectory
    
    def _calc_dynamic_window(self, x):
        """
        计算动态窗口。
        
        参数:
            x: 当前状态 [x(m), y(m), yaw(rad), v(m/s), yaw_rate(rad/s)]
            
        返回:
            动态窗口 [v_min, v_max, yawrate_min, yawrate_max]
        """
        # 机器人运动学约束窗口
        Vs = [self.min_speed, self.max_speed, -self.max_yawrate, self.max_yawrate]
        
        # 动力学约束窗口
        Vd = [x[3] - self.max_accel * self.dt,
              x[3] + self.max_accel * self.dt,
              x[4] - self.max_dyawrate * self.dt,
              x[4] + self.max_dyawrate * self.dt]
        
        # 动态窗口
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return dw
    
    def _eval_trajectory(self, x, dw, goal, ob):
        """
        评估所有可能的轨迹并选择最优的。
        
        参数:
            x: 当前状态 [x(m), y(m), yaw(rad), v(m/s), yaw_rate(rad/s)]
            dw: 动态窗口 [v_min, v_max, yawrate_min, yawrate_max]
            goal: 目标点 [x(m), y(m)]
            ob: 障碍物列表 [[x(m), y(m)], ...]
            
        返回:
            最优速度控制 [v(m/s), yaw_rate(rad/s)]，最优轨迹
        """
        x_init = x[:]
        min_cost = float('inf')
        best_u = [0.0, 0.0]
        best_trajectory = None
        
        # 评估所有可能的速度组合
        for v in np.arange(dw[0], dw[1], self.v_reso):
            for y in np.arange(dw[2], dw[3], self.yawrate_reso):
                # 预测轨迹
                trajectory = self._predict_trajectory(x_init, v, y)
                
                # 计算轨迹成本
                to_goal_cost = self._calc_to_goal_cost(trajectory, goal)
                speed_cost = self._calc_speed_cost(v)
                ob_cost = self._calc_obstacle_cost(trajectory, ob)
                
                # 总成本
                final_cost = (self.to_goal_cost_gain * to_goal_cost + 
                             self.speed_cost_gain * speed_cost + 
                             self.obstacle_cost_gain * ob_cost)
                
                # 更新最优控制
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
        
        return best_u, best_trajectory
    
    def _predict_trajectory(self, x_init, v, y):
        """
        预测给定控制输入下的轨迹。
        
        参数:
            x_init: 初始状态 [x(m), y(m), yaw(rad), v(m/s), yaw_rate(rad/s)]
            v: 速度输入 [m/s]
            y: 偏航角速度输入 [rad/s]
            
        返回:
            预测的轨迹 [[x(m), y(m)], ...]
        """
        x = np.array(x_init)
        trajectory = []
        time = 0
        
        while time <= self.predict_time:
            x = self._motion(x, [v, y], self.dt)
            trajectory.append([x[0], x[1]])
            time += self.dt
        
        return trajectory
    
    def _motion(self, x, u, dt):
        """
        运动模型。
        
        参数:
            x: 状态 [x(m), y(m), yaw(rad), v(m/s), yaw_rate(rad/s)]
            u: 控制输入 [v(m/s), yaw_rate(rad/s)]
            dt: 时间步长 [s]
            
        返回:
            新状态 [x(m), y(m), yaw(rad), v(m/s), yaw_rate(rad/s)]
        """
        x[0] += x[3] * math.cos(x[2]) * dt  # x
        x[1] += x[3] * math.sin(x[2]) * dt  # y
        x[2] += u[1] * dt  # yaw
        x[3] = u[0]  # v
        x[4] = u[1]  # yaw_rate
        
        return x
    
    def _calc_to_goal_cost(self, trajectory, goal):
        """计算轨迹到目标的代价"""
        dx = goal[0] - trajectory[-1][0]
        dy = goal[1] - trajectory[-1][1]
        dist = math.sqrt(dx**2 + dy**2)
        return dist
    
    def _calc_speed_cost(self, v):
        """计算速度代价（鼓励高速）"""
        return self.max_speed - v
    
    def _calc_obstacle_cost(self, trajectory, ob):
        """计算障碍物代价"""
        min_dist = float('inf')
        
        for ii in range(0, len(trajectory), 5):  # 减少计算
            for obstacle in ob:
                dx = trajectory[ii][0] - obstacle[0]
                dy = trajectory[ii][1] - obstacle[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist <= self.robot_radius:  # 碰撞
                    return float('inf')
                
                min_dist = min(min_dist, dist)
        
        return 1.0 / min_dist  # 距离越小，代价越高