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

 