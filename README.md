# kondo-ics-servo-driver

## build
ワークスペースに移動
```
$ cd ~/ros2_ws
```

パッケージのビルド
```
$ colcon build --packages-select kondo_servo_driver kondo_servo_msgs
```

環境の設定
```
$ source install/setup.bash
```

## 実行

### ノードの起動

デフォルトのシリアルポート(/dev/ttyUSB0)を使用する場合
```
$ ros2 run kondo_servo_driver kondo_servo_driver_node
```

シリアルポートを指定する場合
```
$ ros2 run kondo_servo_driver kondo_servo_driver_node --ros-args -p device:=/dev/ttyUSB1
```

 ### 各種サービスの呼び出し

サーボ角度の指令（ID:1のサーボを45度=0.785rad）
ros2 service call /set_position kondo_servo_msgs/srv/SetPosition "{id: 1, position: 0.785}"

現在角度の取得（ID:1のサーボ）
ros2 service call /get_position kondo_servo_msgs/srv/GetPosition "{id: 1}"

サーボIDの設定（ID:2に設定）
ros2 service call /set_id kondo_servo_msgs/srv/SetId "{id: 2}"

現在のサーボIDの取得
ros2 service call /get_id kondo_servo_msgs/srv/GetId "{}"