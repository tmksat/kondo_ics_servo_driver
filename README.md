# kondo-ics-servo-driver

## 概要


## 動作確認環境

- Ubuntu 22.04.1 arm64
- ROS2 Humble
- KONDO KRS-5054HV 


## ビルド

ワークスペースに移動
```
$ cd ~/ros2_ws
```

パッケージのビルド
```
$ colcon build --packages-select kondo_ics_servo_driver
```

環境の設定
```
$ source install/setup.bash
```

## 実行

### ノードの起動

デフォルトのシリアルポート(/dev/ttyUSB0)を使用する場合
```
$ ros2 run kondo_ics_servo_driver kondo_ics_servo_driver_node
```

シリアルポートを指定する場合
```
$ ros2 run kondo_ics_servo_driver kondo_ics_servo_driver_node --ros-args -p device:=/dev/ttyUSB1
```

### 各種サービスの呼び出し

__サーボ角度の指令（ID:1のサーボを45度=0.785rad）__
```
$ ros2 service call /set_position kondo_ics_servo_msgs/srv/SetPosition "{serve_id: 1, angle: 0.785}"
```

__現在角度の取得（ID:1のサーボ）__
```
$ ros2 service call /get_position kondo_ics_servo_msgs/srv/GetPosition "{serve_id: 1}"
```

__サーボIDの設定（ID:2に設定）__
```
$ ros2 service call /set_id kondo_ics_servo_msgs/srv/SetId "{new_id: 2}"
```

__現在のサーボIDの取得（サーボ1台のみ接続してください）__
```
$ ros2 service call /get_id kondo_ics_servo_msgs/srv/GetId "{}"
```