# kuroko_gazebo

ROS 2 Humble + Gazebo Classic 用の Kuroko 起動パッケージです。

- Gazebo Classic を起動
- `kuroko_description/xacro/kuroko/kuroko.xacro` を `gazebo:=true` で展開して `robot_description` を生成
- 床（ground plane）を `z=0` に spawn
- Kuroko を `z=0.35`（デフォルト）に spawn
- `controller_manager.yaml` + `joint_state_broadcaster.yaml` を常に読み込み、
  残りのコントローラ yaml を 1 つ選択して spawner で起動

---

## 前提

- ROS 2 Humble
- Gazebo Classic（`gazebo_ros`）
- `kuroko_description` パッケージが存在すること
  - `kuroko_description/xacro/kuroko/kuroko.xacro`
  - xacro 引数:
    - `gazebo` (default: false)
    - `gz_sim` (default: false)
    - `command_interface` (default: position)

---

## パッケージ構成

```
kuroko_gazebo/
  config/
    controller_manager.yaml
    joint_state_broadcaster.yaml
    joint_group_position_controller.yaml
    joint_group_position_pid_controller.yaml
    joint_trajectory_controller.yaml
    joint_trajectory_pid_controller.yaml
  launch/
    spawn_kuroko_gazebo_classic.launch.py
    spawn_controllers.launch.py
    bringup_gazebo.launch.py
```

---

## ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select kuroko_gazebo
source install/setup.bash
```

---

## 起動方法

### Gazebo + ロボット + コントローラ（統合）

```bash
ros2 launch kuroko_gazebo bringup_gazebo.launch.py   controller:=joint_trajectory_controller
```

effort ベース JTC の例：

```bash
ros2 launch kuroko_gazebo bringup_gazebo.launch.py   controller:=joint_trajectory_effort_controller
```

### スポーン高さを変更

```bash
ros2 launch kuroko_gazebo bringup_gazebo.launch.py robot_z:=0.35
```

---

## コントローラ選択について

- 常に読み込まれる yaml:
  - `controller_manager.yaml`
  - `joint_state_broadcaster.yaml`
- 追加で 1 つ選択（launch 引数 `controller`）:
  - `joint_group_position_controller`
  - `joint_group_position_pid_controller`
  - `joint_trajectory_controller`
  - `joint_trajectory_effort_controller`

`spawn_controllers.launch.py` は、
`controller_manager.ros__parameters` 内で `type` を持つエントリを自動検出し、
すべて spawner により起動します。

---

## トラブルシュート

### コントローラが spawn されない

Gazebo Classic + `gazebo_ros2_control` 構成では、
`/controller_manager` は Gazebo plugin 側で起動します。

URDF / xacro 内の `gazebo_ros2_control` plugin に、
以下の yaml が渡されていることを確認してください。

- `controller_manager.yaml`
- `joint_state_broadcaster.yaml`
- 選択した controller yaml
