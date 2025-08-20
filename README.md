## About

这个算法背景是用于 **无人机群避障并让一架“补位/加入”的无人机安全并入编队**。整体有两条思路：

1. **加入者主动规避**（当前已实现）  
   - **已知信息**：原编队的运动轨迹 F(x, y, z, t)（编队中每架飞机给出随时间的三维位置）。  
   - **输入**：编队每架飞机在各时间帧的对应位置集合，即 F(x, y, z, t|per-plane)。  
   - **输出**：补位无人机的一条可行且安全的**规划路径**。  
   - **算法思想**：采用 **三维 A\*** + **分段局部规划**
   - **算法核心思想**是把动态障碍物转换为静态障碍物的连通域，短时局部规划以应对集群编队的时变障碍约束，本库算是一种A*算法的三维改写和工程应用实现。

2. **编队微调避让**（后续可能会实现第二种）  
   - **目标**：在不偏离原始轨迹太多的前提下，让编队为加入者让出“通道”。  
   - **边界条件**：编队内每架无人机的规避机动限制在其原轨迹周围（例如“规避机动范围 < Δv 球”）。  
   - **输出**：需要进行避碰机动的**原编队中每架无人机在每个时刻的机动向量**（使加入者能穿越空隙到达目标位置）。

> 🔴 在线演示（GitHub Pages）：https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/

---

## <span style="color:#6B2E2E">❖ 算法历程</span>  

<video
  src="https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/testlog/three_drone_test.mp4"
  type="video/mp4"
  width="720"
  controls
  playsinline
  preload="metadata">
  您的浏览器不支持 video 标签。可直接下载：
  <a href="https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/testlog/three_drone_test.mp4">
    three_drone_test.mp4
  </a>
</video>

<p align="center">
  <img src="https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/testlog/picture1.jpg" width="720" alt="局部3D A-STAR路径规划示意">
  <br><sub>局部3D A*路径搜索示意</sub>
</p>

<p align="center">
  <img src="https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/testlog/picture2.jpg" width="720" alt="多机路径互斥 任意位置起飞">
  <br><sub>多机路径互斥 任意位置起飞</sub>
</p>


---

<video
  src="https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/testlog/group_drone_test.mp4"
  type="video/mp4"
  width="720"
  controls
  playsinline
  preload="metadata">
  您的浏览器不支持 video 标签。可直接下载：
  <a href="https://agathazhang.github.io/Drone-swarm-avoidance-cooperative-target/testlog/group_drone_test.mp4">
    group_drone_test.mp4
  </a>
</video>

**一些解释**：  
- 体素扩增预览图 && 单架次全局路径图：  
<table align="center">
  <tr>
    <td align="center">
      <img src="https://agathazhang.github.io//Drone-swarm-avoidance-cooperative-target/testlog/picture5.jpg" width="360" alt="体素扩增预览图"><br>
      <sub>体素扩增预览图</sub>
    </td>
    <td align="center">
      <img src="https://agathazhang.github.io//Drone-swarm-avoidance-cooperative-target/testlog/picture6.jpg" width="360" alt="单架次全局路径图"><br>
      <sub>单架次全局路径图</sub>
    </td>
  </tr>
</table>

- 参数仿真：  
<table align="center">
  <tr>
    <td align="center">
      <img src="https://agathazhang.github.io//Drone-swarm-avoidance-cooperative-target/testlog/unity1.jpg" width="360" alt="Unity仿真测试"><br>
      <sub>Unity刚体碰撞检测 修正体素扩增半径</sub>
    </td>
    <td align="center">
      <img src="https://agathazhang.github.io//Drone-swarm-avoidance-cooperative-target/testlog/picture3.jpg" width="360" alt="三维A*调参"><br>
      <sub>Matlab下三维 A* 搜索尺度调参</sub>
    </td>
  </tr>
</table>
---

## How to use

仓库中的三个主要目录：

1. **初期仿真**：用于在 Unity 中验证思路与做 Demo，记录早期的设计与灵感启发。  
2. **PC实现**：基于地面站一次性计算多架次路径，生成综合路径（包含多补位飞机间的路径互斥约束）；后续将加入并行计算优化。  
3. **Linux实现**：按项目需求使用 Mavlink 通信，在机载模块 RV1103 上做嵌入式计算，实现算法的跨平台迁移与部署。

  - **鸣谢**：本文的A* 库是发展自Damian Barczyński的二维平面A* 算法https://github.com/daancode/a-star 本人将原作的2D平面A* 改写为了3D空间A* 这个部分的脚本欢迎大家复用 至此再次鸣谢Barczyński。


后续将部署带飞机间通信的集群编队算法，采用一种基于闵可夫斯基空间中时空事件F(x, y, z, t)的最优化思想，做**最短电池能耗**的控制优化；并把**动力学模型**纳入为子优化目标。

