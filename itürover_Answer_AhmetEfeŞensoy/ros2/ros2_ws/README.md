# ITU Rover Engel Kaçınma Ödevi — ROS 2 Portu

ROS 1 (Noetic) kod tabanının tamamının ROS 2 (Humble, Gazebo Classic)
karşılığı. `itürover_Answer_AhmetEfeŞensoy/` altındaki orijinal ROS 1
çözümlerin birebir işlevsel eşdeğeridir.

## Paketler

| Paket | Tür | İçerik |
|---|---|---|
| `reactive_robot_sim` | ament_cmake | URDF/xacro, worlds, models, sim `launch.py`, Gazebo Classic plugin tanımları |
| `reactive_robot_common` | ament_python | Solution'lar arası paylaşılan saf-Python (`feature_utils`, `nn`, `barn_sim`) |
| `solution_duz_fgm` | ament_python | Follow the Gap Method (FGM) + disparity extender |
| `solution_fgm_dwa` | ament_python | FGM + Dynamic Window Approach (DWA) |
| `solution_kompleks_fgm_dwa` | ament_python | Genişletilmiş FGM+DWA (obstacle memory, door-commit state machine) |
| `solution_ml` | ament_python | Behavioural Cloning + DQN politikası, BARN veri toplama, Nav2 tabanlı expert navigasyon |

## Kurulum

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs \
     ros-humble-turtlebot3 ros-humble-turtlebot3-simulations \
     ros-humble-nav2-bringup ros-humble-teb-local-planner \
     ros-humble-xacro python3-colcon-common-extensions

mkdir -p ~/rover_ws/src
cp -r ros2_ws/src/* ~/rover_ws/src/
cd ~/rover_ws
colcon build --symlink-install
source install/setup.bash
```

## Çalıştırma

Her solution kendi launch dosyasıyla simülasyonu + node'u aynı anda başlatır
(ödev gereği):

```bash
# Çözüm 1 — düz FGM
ros2 launch solution_duz_fgm reactive_robot.launch.py

# Çözüm 2 — FGM + DWA
ros2 launch solution_fgm_dwa reactive_robot.launch.py

# Çözüm 3 — genişletilmiş FGM+DWA (obstacle memory, door-commit)
ros2 launch solution_kompleks_fgm_dwa reactive_robot.launch.py

# Çözüm 4 — ML (Behavioural Cloning politikası)
ros2 launch solution_ml ml_robot.launch.py weights_path:=/path/to/weights.npz
```

Parametreler launch argümanı olarak override edilebilir, örn.:

```bash
ros2 launch solution_duz_fgm reactive_robot.launch.py max_vel:=0.5 run_duration:=90.0
```

## ROS 1 → ROS 2 dönüşümünde önemli noktalar

- `rospy` → `rclpy.node.Node`; `get_param("~x")` → `declare_parameter`/`get_parameter`.
- `.launch` (XML) → `.launch.py` (Python launch API).
- Gazebo plugin'leri (`libgazebo_ros_diff_drive.so`, `_laser`, `_imu`,
  `_openni_kinect`, `_p3d`) ROS 2 `gazebo_ros` eşdeğerleriyle
  (`_ray_sensor`, `_imu_sensor`, `_camera`) değiştirildi; `<ros><remapping>`
  bloğu eklendi.
- `gazebo_msgs/DeleteLight` servisi `create_client`+`call_async` ile senkron
  çağrılıyor (ödev gereği koşu sonunda 'sun' ışığı siliniyor).
- `gazebo_msgs/SetModelState` → ROS 2'de **`SetEntityState`** (alan adı
  `model_name`→`name`); `dqn_finetune.py` episode reset'inde kullanılıyor.
- ROS 1 `move_base` + `teb_local_planner` + ayrı costmap YAML'ları → tek
  `solution_ml/config/nav2_params.yaml` (Nav2 `controller_server` /
  `planner_server` / `bt_navigator` / `behavior_server`).
- `actionlib.SimpleActionClient('move_base', ...)` → `barn_goal_publisher.py`
  içinde Nav2 `navigate_to_pose` action arayüzüne taşındı.
- `solution_ml/datasets_external/drlnav` zaten ROS 2 (rclpy/ament) olduğu
  için dokunulmadı — orijinal haliyle vendored.

## Bilinen sınırlamalar

- Bu dönüşüm Windows geliştirme makinesinde (ROS 2 kurulu değil) yapıldı;
  Python dosyaları `py_compile` ile sözdizimi doğrulandı, XML/xacro/world/SDF
  dosyaları well-formedness için doğrulandı, YAML `yaml.safe_load` ile
  doğrulandı. Gerçek bir ROS 2 + Gazebo Classic ortamında `colcon build` ve
  canlı bir koşu ile uçtan uca doğrulama henüz yapılmadı.
- Nav2 parametreleri (`nav2_params.yaml`) ROS 1 move_base/TEB/costmap
  değerlerinin en yakın eşdeğeridir; TEB local planner'ın Nav2 plugin
  arayüzü (`teb_local_planner::TebLocalPlannerROS`) sürüme göre küçük
  isim farklılıkları gösterebilir — kurulu `teb_local_planner` sürümüne
  göre doğrulanmalıdır.
