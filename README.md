# kondo-ics-servo-driver

## 概要

このリポジトリは、KONDO製のICSプロトコルを使用するサーボモーターをROS2環境で制御するためのドライバです。ビルド手順や実行方法、各種サービスの呼び出し方法、パラメータについての詳細な説明が含まれています。


## 動作確認環境

- Ubuntu 22.04.1 arm64
- ROS2 Humble
- [KONDO KRS-5054HV ICS](https://kondo-robot.com/product/03180)を用いてテスト
- 通信速度 115200[bps]


## サービス

このROSノードは以下のサービスを提供します：

1. `/set_position`
    - **説明**: 指定したサーボモーターの角度を設定します。
    - **リクエスト**: `kondo_ics_servo_msgs/srv/SetPosition`
      - `servo_id` (int): サーボモーターのID
      - `angle` (float): 設定する角度[rad]
    - **レスポンス**: 
      - `success` (bool): 角度設定が成功したかどうか

2. `/get_position`
    - **説明**: 指定したサーボモーターの現在の角度を取得します。
    - **リクエスト**: `kondo_ics_servo_msgs/srv/GetPosition`
      - `servo_id` (int): サーボモーターのID
    - **レスポンス**: 
      - `angle` (float): 現在の角度[rad]

3. `/set_id`
    - **説明**: サーボモーターのIDを設定します。
    - **リクエスト**: `kondo_ics_servo_msgs/srv/SetId`
      - `new_id` (int): 新しいサーボモーターのID
    - **レスポンス**: 
      - `success` (bool): ID設定が成功したかどうか

4. `/get_id`
    - **説明**: 接続されているサーボモーターのIDを取得します。
    - **リクエスト**: `kondo_ics_servo_msgs/srv/GetId`
      - リクエストは空のメッセージです。
    - **レスポンス**: 
      - `servo_id` (int): サーボモーターのID

5. `/free`
    - **説明**: 指定したサーボモーターのトルクをオフにします。
    - **リクエスト**: `kondo_ics_servo_driver/srv/Free`
      - `servo_id` (int): サーボモーターのID
    - **レスポンス**: 
      - `success` (bool): トルクオフが成功したかどうか



## パラメータ

このROSノードに対して設定できるパラメータは以下の通りです：

1. `port`
    - **説明**: サーボモーターと通信するためのシリアルポートのデバイスファイルを指定します。
    - **デフォルト値**: `/dev/ttyUSB0`
    - **使用例**: `/dev/ttyUSB1`

これらのパラメータは、ノードの起動時に`--ros-args -p <parameter_name>:=<value>`の形式で指定することができます。


## ビルド

1. ワークスペースに移動します。  
2. パッケージのビルドをビルドします。
3. 環境変数等の設定します。  
```shell
$ cd ~/ros2_ws
$ colcon build --packages-select kondo_ics_servo_driver
$ source install/setup.bash
```

## 実行

### ノードの起動

デフォルトのシリアルポート(/dev/ttyUSB0)を使用する場合
```shell
$ ros2 run kondo_ics_servo_driver kondo_ics_servo_driver_node
```

シリアルポートを指定する場合
```shell
$ ros2 run kondo_ics_servo_driver kondo_ics_servo_driver_node --ros-args -p port:=/dev/ttyUSB1
```

### 各種サービスの呼び出し

__サーボ角度の指令（ID:1のサーボを45度=0.785rad）__
```shell
$ ros2 service call /set_position kondo_ics_servo_msgs/srv/SetPosition "{serve_id: 1, angle: 0.785}"
```

__現在角度の取得（ID:1のサーボ）__
```shell
$ ros2 service call /get_position kondo_ics_servo_msgs/srv/GetPosition "{serve_id: 1}"
```

__サーボIDの設定（ID:2に設定）__
```shell
$ ros2 service call /set_id kondo_ics_servo_msgs/srv/SetId "{new_id: 2}"
```

__現在のサーボIDの取得（サーボ1台のみ接続してください）__
```shell
$ ros2 service call /get_id kondo_ics_servo_msgs/srv/GetId "{}"
```

__サーボのトルクオフ（ID:1）__
```shell
$ ros2 service call /free kondo_ics_servo_driver/srv/Free "{servo_id: 1}"
```