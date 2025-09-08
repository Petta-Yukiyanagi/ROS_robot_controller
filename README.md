# ROS Robot Controller（Roomba + ROS 2 Humble）

Raspberry Pi 上の Docker コンテナで Roomba (Create 2) を ROS 2 Humble から制御するための一式です。  
コンテナ起動後は **Launch が自動で走る**ため、手で `ros2 launch` を叩く必要はありません。  
Flutter 製のコントローラーから `rosbridge_websocket` に接続し、`/cmd_vel` を送ると、スタックがゲートして `/cmd_vel_out` → `create_driver` に中継します。バッテリーは `/battery_state` で配信します。

---

## Docker Image

- パッケージページ：  
  👉 **ghcr.io/petta-yukiyanagi/ros_humble_lab**  
  https://github.com/Petta-Yukiyanagi/ROS_robot_controller/pkgs/container/ros_humble_lab

### Pull
```bash
docker pull ghcr.io/petta-yukiyanagi/ros_humble_lab:latest
# 必要に応じて日付/バージョンタグも利用
# docker pull ghcr.io/petta-yukiyanagi/ros_humble_lab:2025-09-07
```

### Run（お試し）
```bash
docker run -it --rm   --net=host   -v /run/udev:/run/udev:ro   --device /dev/ttyUSB0:/dev/roomba   ghcr.io/petta-yukiyanagi/ros_humble_lab:latest
```

> 運用時は下記の **systemd 常駐**を使うと、電源投入 → 自動起動までノータッチになります。

---

## クイックスタート（Raspberry Pi 常駐）

### 1) コンテナ常駐（ホスト OS の systemd）

`/etc/systemd/system/create-stack.service`：
```ini
[Unit]
Description=Roomba create-stack container
After=docker.service
Requires=docker.service

[Service]
ExecStart=/usr/bin/docker run --rm --name create-stack   --net=host   -v /run/udev:/run/udev:ro   -v /opt/roomba-stack:/opt/roomba-stack:rw   -v /opt/overlay_ws:/opt/overlay_ws:rw   --device /dev/ttyUSB0:/dev/roomba   ghcr.io/petta-yukiyanagi/ros_humble_lab:latest
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

### 2) コンテナ内で Launch を常駐

`/etc/systemd/system/create-bringup.service`：
```ini
[Unit]
Description=Roomba bringup (ros2 launch inside container)
After=create-stack.service
Requires=create-stack.service

[Service]
ExecStart=/usr/bin/docker exec create-stack bash -lc "\
  . /opt/ros/humble/setup.bash; \
  . /opt/overlay_ws/install/setup.bash; \
  ros2 launch /opt/roomba-stack/launch/create_stack.launch.py \
"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

適用：
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now create-stack.service create-bringup.service
```

> **旧式の個別サービス**（`create-driver.service` / `create-stack-manager.service`）は使用しません。重複起動の原因になります。

---

## モード（PASSIVE / SAFE）の基礎知識（初心者向け）

Roomba Create 2 の公式 Open Interface には代表的に次のモードがあります。

- **PASSIVE（パッシブ）**  
  センサー取得や LED 制御は可能ですが、**走行コマンドには応答しません**。
- **SAFE（セーフ）**  
  **走行が可能**。段差や車輪落下などの危険検知でモーターが自動停止する安全機構が働きます。
- **FULL（フル）**  
  安全機構を無効化した完全制御（危険）。**本スタックでは使用しません**。

本スタックでは **PASSIVE ↔ SAFE** を自動で切り替えます：

- 起動直後は **PASSIVE**。  
- `/cmd_vel` を受け取ると **非同期で SAFE 要求**を送信し、**待たずに即パススルー**で `/cmd_vel_out` を publish（操作の体感を最短化）。  
- 入力が途切れて `idle_timeout_sec` を越えると **PASSIVE** に戻し、安全側に倒します。

### supervisor トピック
- `/supervisor/mode` (`std_msgs/UInt8`)：**2=SAFE, 3=PASSIVE**。**Transient Local**（最新値をラッチ配信）  
- `/supervisor/active` (`std_msgs/Bool`)：**今まさに走行中か**のフラグ。**Transient Local**

> Flutter 側でこの 2 つを表示しておくと、状態が一目で分かります。

---

## アーキテクチャ / ノード構成

- **ホスト OS (Raspberry Pi)**  
  - `create-stack.service`：Docker コンテナ常駐  
  - `create-bringup.service`：コンテナ内で `ros2 launch` 実行

- **コンテナ内（ghcr.io/petta-yukiyanagi/ros_humble_lab）**  
  - ROS 2 Humble  
  - **起動ノード**（Launch で一括起動）  
    - `/create_driver`（C++：Roomba ドライバ）  
    - `/create_stack_manager`（Python：統合・監督ノード）  
    - `/rosbridge_websocket`（WS: 9090）  
    - （任意）`/robot_state_publisher`

### 主なトピック / サービス

- 入力: `/cmd_vel`（Flutter → rosbridge → ここ）  
- 中継: `/cmd_vel_out`（stack_manager が publish → create_driver が subscribe）  
- 監督: `/supervisor/mode`, `/supervisor/active`（**TL**）  
- バッテリー: `/battery_state`（`sensor_msgs/BatteryState`、**TL**）  
- Services（driver 側）：  
  - `/create/set_safe`（`std_srvs/Trigger`）  
  - `/create/set_passive`（`std_srvs/Trigger`）

### QoS（要点）
- `/cmd_vel`：**BEST_EFFORT / VOLATILE**（低遅延重視）  
- `/cmd_vel_out`：**RELIABLE / VOLATILE**  
- `/supervisor/*` と `/battery_state`：**RELIABLE / TRANSIENT_LOCAL (depth=1)**

---

## create_stack_manager の挙動（流れ）

1. `/cmd_vel` を受信 → **即** `/cmd_vel_out` へ中継（レスポンス重視）  
2. 同時に **非同期で SAFE 要求**（サービス応答は待たない）  
3. SAFE 確立時に **最後に受けた Twist を 1 回だけ再送**（単発入力でも動きやすく）  
4. 入力が一定時間（`idle_timeout_sec`）ない → **PASSIVE 要求**（必要なら停止 Twist を一度だけ送出）  
5. バッテリー `/battery/*` を合成して `/battery_state`（TL）を配信

### 主なパラメータ（Launch 抜粋）
```python
ExecuteProcess(
  cmd=[
    'python3', '/opt/roomba-stack/nodes/create_stack_manager.py',
    '--ros-args',
    '-p', 'cmd_vel_in:=/cmd_vel',
    '-p', 'cmd_vel_out:=/cmd_vel_out',
    '-p', 'idle_timeout_sec:=3.0',
    '-p', 'battery_publish_hz:=1.0',
  ],
  output='screen',
)
```
- `idle_timeout_sec` … 途切れてから PASSIVE へ落とす秒数（**連続運転を重視**するなら大きめ推奨）  
- `publish_zero_once_on_idle` … PASSIVE 移行時に停止 Twist を一度だけ送出  
- `battery_publish_hz` … `/battery_state` 発行周期（既定 1.0 Hz）

---

## Flutter コントローラー（使い方）

- 接続先：`ws://<raspi-ip>:9090`  
- Publish：`/cmd_vel`（`geometry_msgs/Twist`）  
  - **連続入力のときは 10–30 Hz** を目安に送信（操作が途切れにくい）  
- Subscribe（UI に表示するのに便利）：  
  - `/supervisor/mode`（2=SAFE, 3=PASSIVE）  
  - `/supervisor/active`（走行中かどうか）  
  - `/battery_state`（残量・電圧など）

> “一瞬だけ動いて止まる” 場合は、1) アプリ側の送信周期が十分か、2) `idle_timeout_sec` が短すぎないか、を確認してください。

---

## よく使う確認コマンド

```bash
# コンテナ稼働
docker ps --format "table {{.Names}}	{{.Status}}"

# デバイス（ホスト / コンテナ）
ls -l /dev/ttyUSB0
docker exec -it create-stack ls -l /dev/roomba

# ノード / トピック
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 node list"
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 topic list"

# QoS（TL の確認）
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 topic info /supervisor/mode -v"
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 topic info /supervisor/active -v"

# rosbridge 9090
docker exec -it create-stack bash -lc "ss -lntp | grep 9090 || echo 'no rosbridge'"

# 経路：/cmd_vel → /cmd_vel_out
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 topic info /cmd_vel -v"
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 topic info /cmd_vel_out -v"

# バッテリー
docker exec -it create-stack bash -lc ". /opt/ros/humble/setup.bash; ros2 topic echo /battery_state --once"
```

---

## ライセンス
（ここにライセンス表記を記載）
