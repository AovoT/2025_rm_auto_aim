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

##
相机坐标系 镜头对着的方向为z  右边为x  下边为y
世界坐标系 看向装甲板的的方向为z  右边为x  下边为y

## 检查坐标变换
ros2 run tf2_ros tf2_echo imu_frame camera_frame
ros2 run tf2_ros tf2_echo odom_frame imu_frame 




## 上位机下位机通信流程
![RUNOOB 图标](./images/Untitled diagram-2024-09-10-072612.png)
