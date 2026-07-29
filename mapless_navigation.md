<!--
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-23 11:15:48
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-23 13:24:16
 * @FilePath: /far_planner/mapless_navigation.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
graph可以保留，全地图（自由、障碍、可通行）给我的高程图我还当作全局地图用吗？还是只当局部地图用，全局地图还是依旧保留这直接使用点云的方案，累积误差影响大吗？地图端会维护一个全局的点云和语义地图吗？除了全局地图需要维护几何信息之外，建议维护一个和高程相同维度的代价地图，这个代价地图统一把语义代价和可通行性代价揉在一起，注意代价的卡尔曼更新与动态物体的去除。waypoint可以结合之前的方案，然后根据布局代价来确定，这个确定可能会有一些工作量。
在确定waypoint之后，使用带代价的地图进行规划路径，这里在规划时可以考虑梯度的方向及各种代价。可以先写一个保底的，后面可以使用自监督，我觉得这个来做自监督应该可以。




里程计回环与代价地图累积的代价地图更新，可以使用一个代价地图就完成障碍+代价的维护吗？不考虑别的区域。如果有回环，可通行代价和语义代价可以怎么维护。


先不考虑里程计的误差+自己维护代价地图+传统的局部规划器。