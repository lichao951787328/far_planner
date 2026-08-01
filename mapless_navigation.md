<!--
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-23 11:15:48
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2026-08-01 14:38:48
 * @FilePath: /far_planner/mapless_navigation.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
graph可以保留，全地图（自由、障碍、可通行）给我的高程图我还当作全局地图用吗？还是只当局部地图用，全局地图还是依旧保留这直接使用点云的方案，累积误差影响大吗？地图端会维护一个全局的点云和语义地图吗？除了全局地图需要维护几何信息之外，建议维护一个和高程相同维度的代价地图，这个代价地图统一把语义代价和可通行性代价揉在一起，注意代价的卡尔曼更新与动态物体的去除。waypoint可以结合之前的方案，然后根据布局代价来确定，这个确定可能会有一些工作量。
在确定waypoint之后，使用带代价的地图进行规划路径，这里在规划时可以考虑梯度的方向及各种代价。可以先写一个保底的，后面可以使用自监督，我觉得这个来做自监督应该可以。




里程计回环与代价地图累积的代价地图更新，可以使用一个代价地图就完成障碍+代价的维护吗？不考虑别的区域。如果有回环，可通行代价和语义代价可以怎么维护。


先不考虑里程计的误差+自己维护代价地图+传统的局部规划器。


把 Util/dyosb_update_thred 从 4 降到 2（先试）
把 Util/dynamic_obs_dacay_time 从 10 降到 3-5
把 CDetector/filter_count_value 从 3 提到 4 或 5（减少稀疏伪轮廓）
若仍有“厚边”，再评估 Util/obs_inflate_size 是否过于保守

1、在不考虑语义区域渐变式自适应可通行区域的前提下
重心1：维护一个具有切实指导意义的graph，也就是要维护一个全局的二维概率栅格地图。
如果上层直接给语义分割障碍区域和动态障碍，确定是动态障碍的才当作动态障碍，例如人、狗等移动生命活体；把建筑等固定的当作确定性的静态障碍；把车当作初步静态障碍，在传输点云时，要筛选出非动态点云的物体，然后维护这个全局的二维概率栅格地图。

重心2：
二维栅格地图直接转img、进行过滤操作之后即可进行维护graph，graph可能需要不会整体性的维护，只维护局部，然后再拼成全局。

2、局部规划层面，依靠地形高程图，叠加当前的全部障碍和地形可通行性代价，进行规划。或者自行构建一个随着机器人坐标更新的https://voxblox.readthedocs.io/en/latest/index.html,https://github.com/ethz-asl/voxblox.git

Voxblox 坐标系：固定不动（与世界坐标系平行）。
滚动窗口：仅指空间位置随机器人平移，不包含旋转。
对规划的影响：无负面影响。规划器通过 tf 变换将世界坐标系的障碍物转换到机器人坐标系下进行计算。
性能优势：这种设计避免了昂贵的网格旋转重采样，保证了实时性。

实现思路：非动态点云加在到octreemap，维护一个全局暂静态的三维地图，当作全局地图使用。同时，截取机器人附近区域部分，添加上动态障碍，并添加上可通行性代价和障碍。

第二步：octreemap在转二维图维护障碍地图时，如果地面是三维场景，是不是应该考虑梯度场，


第三步：给每个栅格补上语义标签参考ssmi@github