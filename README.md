# raspimouse_odometory_tuning_ros

https://github.com/YusukeKato/raspimouse_odometry_tuning_unity

このUnityアプリケーションと共に使用する

## キャリブレーションの流れ
1. ロボットを一辺L[m]の正方形を描くように走らせる
2. 4つの角におけるロボットの位置姿勢を取得
3. 目標とする位置姿勢と比較して補正値を計算
4. モータの出力を補正

## 導入方法
```
cd ~/catkin_ws/src
git clone https://github.com/YusukeKato/raspimouse_odometory_tuning_ros.git
cd ~/catkin_ws
catkin_make
```

## motors.cpp の置き換え
https://github.com/ryuichiueda/raspimouse_ros_2

このパッケージ内にある src/motors.cpp を置き換える

## 実行

### rosbridge_server
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

### RasPiMouse
```
roslaunch raspimouse_ros_2 raspimouse.launch
```

### 正方形を描くように走行
右まわり
```
rosparam set /motion_mode 1
rosrun raspimouse_odometory_tuning_ros square_cmdvel.py
```

左まわり
```
rosparam set /motion_mode 2
rosrun raspimouse_odometory_tuning_ros square_cmdvel.py
```

### キャリブレーション
```
rosrun raspimouse_odometory_tuning_ros square_raspimouse_calibration.py
```

Unityアプリケーションにおける"Signal"ボタンを押すことで
キャリブレーションを行う