# 河北科技大学RobotMaster2025自瞄算法

## 开发步骤

先克隆到本地仓库 `git clone https://github.com/AovoT/2025_rm_auto_aim.git`cd到本地仓库下执行下面命令

```shell
#切换到dev
git checkout dev
# 拉取dev分支最新代码
git pull origin dev
#创建一个本地的分支然后切换到这个分支
git checkout -b feature/your-feature-name 
```

然后就可以基于刚克隆下来的仓库进行开发，开发完成想要合并的时候，执行以下指令拉取到远程仓库

```shell
git add .
git commit -m "提交的备注"
git push origin feature/your-feature-name
```

然后进入仓库页面创建PR就可以

## 各个坐标系以及怎么转换
### 转换过程
- 先从相机坐标系通过静态变化转换到imu坐标系下,转换矩阵通过imu和相机的刚性连接机构来决定。
- 接收下位机传来的imu发的四元数，发布动态变换,从imu坐标系转换到惯性系。
### 固定坐标系惯性系
相机坐标系 镜头对着的方向为+z  右边为+x  下边为+y
世界坐标系 看向装甲板的的方向为+z  右边为+x  下边为+y
### 步兵陀螺仪惯性系
imu坐标系 镜头对着的方向方向为-y  左边为+x  上边为+z
惯性坐标系 镜头对着的方向为-y  左边为+x  上边为+z
### 英雄陀螺仪惯性系

### 哨兵陀螺仪

### 注意事项
坐标系转换的时候，动态变换和静态变换的事件戳必须一样，在ros2静态变换当中, 填写参数的时候面向轴的正方向顺时针旋转为+,反之为-;

## 检查坐标变换
ros2 run tf2_ros tf2_echo imu_frame camera_frame
ros2 run tf2_ros tf2_echo odom_frame imu_frame 

## 上位机下位机通信流程
![RUNOOB 图标](./images/Untitled diagram-2024-09-10-072612.png)
