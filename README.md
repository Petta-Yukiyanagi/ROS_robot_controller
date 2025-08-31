# ROS Robot Controller with Flutter + Roomba(Create2)

このリポジトリは、**Flutter アプリ**から **iRobot Create2 (Roomba系) を USB 接続した Raspberry Pi** 経由で操作する手順をまとめたものです。  
通信には **rosbridge_server (WebSocket)** を使用し、アプリから `/cmd_vel` を publish することでロボットを動かします。

---

## システム構成

```
Flutter App (PC/Android/iOS)
    ↓ ws://<PC or RPi>:9090 (WebSocket)
rosbridge_server (Raspberry Pi 上)
    ↓ ROS 2 Topic (/cmd_vel)
create_driver (Raspberry Pi 上, USB経由でRoombaと接続)
    ↓ UART (115200 baud)
Roomba / iRobot Create2
```

---

## 事前準備

### 1. Raspberry Pi (Ubuntu 24.04 / ROS 2 Jazzy)

#### ROS 2 環境セットアップ
```bash
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep git
sudo rosdep init || true
rosdep update
```

#### Create2 ドライバのビルド
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/AutonomyLab/create_robot.git
git clone https://github.com/AutonomyLab/libcreate.git
rosdep install --from-paths . --ignore-src -r -y

cd ~/ros2_ws
colcon build --symlink-install
```

#### 環境設定
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2. デバイス設定 (USB接続)

USB接続を確認：
```bash
ls -l /dev/ttyUSB* /dev/serial/by-id/
```

例：
```
/dev/ttyUSB0
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AV0L2E1F-if00-port0
```

推奨は **/dev/serial/by-id/...** を使用。

ユーザーを `dialout` グループに追加（再ログイン必須）：
```bash
sudo usermod -a -G dialout $USER
```

---

### 3. Create2 ドライバ起動

```bash
ros2 launch create_bringup create_2.launch   port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AV0L2E1F-if00-port0   baud:=115200
```

---

### 4. rosbridge_server 起動

別ターミナルで：
```bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

---

### 5. 接続確認

```bash
ros2 topic info /cmd_vel --verbose
```

- Publisher: `rosbridge_websocket`
- Subscriber: `create_driver`

となっていれば準備OK。

動作テスト：
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist   "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

---

## Flutter アプリ側

### 1. pubspec.yaml 依存関係
```yaml
dependencies:
  flutter:
    sdk: flutter
  flutter_joystick: ^0.2.1
  roslibdart:
    git:
      url: https://github.com/tmtong/roslibdart.git
      ref: main
  provider: ^6.1.5
  shared_preferences: ^2.5.3
```

### 2. 接続設定
アプリ起動 → 上部の **ROS Bridge IP Address** に接続先を入力。

#### PCで動かす場合（SSHトンネルあり）
```bash
ssh -L 9090:localhost:9090 user@<raspi-ip>
```
アプリの接続先は：
```
ws://127.0.0.1:9090
```

#### スマホで動かす場合
- Termius等で「Local Port Forward 9090 → localhost:9090」を設定し、同じく
```
ws://127.0.0.1:9090
```

---

## 操作方法

- **Linear Speed / Angular Speed** のスライダで速度を調整
- **D-pad**  
  - 押している間 `/cmd_vel` を連続送信  
- **ジョイスティック**  
  - 倒している間 `/cmd_vel` を連続送信  
  - 離すと停止
- **Emergency Stop** ボタンで即停止

---

## 注意事項

- 充電ドックに載っている場合や、セーフティ（クリフ/バンパ/ESTOP）が反応している場合は `/cmd_vel` を無視します。
- 安全な環境で動作確認してください。
- 初回は必ず `ros2 topic pub` コマンドで動作確認してからアプリを使用してください。

---

## トラブルシュート

- `/cmd_vel` に Subscriber がいない → ドライバ起動コマンドを確認
- Permission denied → `dialout` グループに追加したか確認
- 動かないがSubscriberはいる → ドックから外す / 速度を0.25以上に上げる
- 接続できない → SSHトンネル設定を再確認 (`ssh -L 9090:localhost:9090 ...`)

---

## ライセンス
各依存リポジトリ（`create_robot`, `libcreate`, `roslibdart` など）のライセンスに準じます。


---

## お手軽起動方法（ドライバや環境構築済み前提）

### 1. Raspberry Pi 側（SSHログイン後）

#### ROS環境の読み込み
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### Create2 ドライバ起動
```bash
ros2 launch create_bringup create_2.launch   port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AV0L2E1F-if00-port0   baud:=115200
```

#### rosbridge_server 起動（別ターミナル）
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

---

### 2. Windows PC 側（PowerShell）

#### SSHトンネルを張る
```powershell
ssh -L 9090:localhost:9090 user@<raspi-ip>
```
- `<raspi-ip>` はラズパイのIPアドレスに置き換えてください。  
- 接続後は **PCローカルの127.0.0.1:9090 がラズパイのrosbridgeに直結**します。

---

### 3. Flutter アプリ

- 接続先を **`ws://127.0.0.1:9090`** に設定して **Connect** を押す。  
- AppBar が緑色「Connected」になれば操作可能です。  
- D-pad長押し、またはジョイスティックを倒すと `/cmd_vel` が送られてルンバが動きます。  
- 停止は **Emergency Stop ボタン**。

---


---

## 応用: スマホがラズパイと別LANにいる場合（PCと同じLANにいる場合）

### 想定構成
- **ラズパイ** … LAN A に接続、rosbridge_server を起動中  
- **PC** … LAN A に接続してラズパイに SSH 可能  
- **スマホ** … LAN B に接続（ラズパイには直接つながらない）、ただし PC と同じLAN内にいる  

### 解決策: PC を中継する

#### 1. PCでSSHトンネルを張る
```powershell
ssh -L 9090:localhost:9090 user@<raspi-ip>
```
これにより **PCのポート9090 → ラズパイのrosbridge(9090)** へ転送される。

#### 2. PCのLAN内IPを調べる
```powershell
ipconfig
```
例: `192.168.20.50`

#### 3. スマホのFlutterアプリで接続
```
ws://192.168.20.50:9090
```
を指定。  
スマホ → PC:9090 → SSHトンネル → ラズパイ:9090 → rosbridge に接続できる。

### 注意点
- PCのファイアウォールでポート9090を開放する必要がある。  
- SSHトンネルを張っているPowerShellは閉じないこと。  
- PCが落ちると接続も切れる。

---
