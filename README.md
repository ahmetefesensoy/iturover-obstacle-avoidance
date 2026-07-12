# ITU Rover — Obstacle Avoidance Assignment

Turtlebot simülasyonu üzerinde reaktif engelden kaçınma probleminin farklı
yaklaşımlarla çözümü. Görev, robotun `/scan` topic'inden gelen
[LaserScan](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
verisini kullanarak koridordaki engellerden kaçınıp başlangıç noktasından
mümkün olduğunca uzaklaşmasıdır.

`itürover_Answer_AhmetEfeŞensoy/` klasörü altında iki paralel çalışma alanı içerir:

| Klasör | İçerik |
|---|---|
| `itürover_Answer_AhmetEfeŞensoy/ros1/` | **ROS 1** (Noetic, catkin) tabanlı orijinal çözümler |
| `itürover_Answer_AhmetEfeŞensoy/ros2/` | Aynı çözümlerin **ROS 2** (rclpy, colcon) portu |

Her iki çalışma alanının da içinde `iturover_obstacle_avoidance_assignment-main/`
adında ödevin orijinal simülasyon paketi (world, urdf, launch dosyaları) bulunur;
çözümler bu paketin üzerine yazılmıştır.

## Solution'lar (kronolojik sıra)

Her çalışma alanı içinde aynı 4 çözüm bulunur. İlk üçü kronolojik olarak
birbirinin **versiyonudur** — her biri bir öncekinin geliştirilmiş halidir:

1. **`solution_FGM_DWA/`** — İlk versiyon. Follow-the-Gap Method (FGM) ile
   Dynamic Window Approach (DWA)'nın basit birleşimi.
2. **`solution_kompleks_FGM_DWA/`** — İkinci versiyon. FGM + DWA'nın çok daha
   gelişmiş/parametrik hâli (door-commit durum makinesi, obstacle memory,
   corridor compass). `docs/` altında geliştirme süreci notları
   (`denemegünlük.md`) bulunur.
3. **`solution_düz_FGM/`** — Üçüncü ve son versiyon. Kompleks kontrolcüden
   damıtılmış, sadeleştirilmiş saf FGM yaklaşımı: karmaşık versiyondan yalnızca
   gerçekten işe yarayan door-commit mekanizması korundu (`batch_run.py` ile
   toplu simülasyon/telemetri desteği eklendi).

Ayrı bir dördüncü koldaki:

4. **`solution_ml/`** — Klasik reaktif yöntemler yerine öğrenme tabanlı
   (behavioral cloning / DAgger) yaklaşım. `core/`, `training/`, `testing/`,
   `ros_nodes/` alt modülleriyle ayrı bir pipeline oluşturur; `datasets_external/`
   altında referans olarak incelenen DRL-navigation örnek ağırlıkları yer alır.

## Depo notları

- Eğitim verisi (`datasets*/`), model ağırlıkları (`*.npz`, `*.pt`) ve çalışma
  zamanı çıktıları (telemetri/log/scan dump CSV'leri, `batch_*/` klasörleri)
  boyut nedeniyle `.gitignore` ile hariç tutulmuştur; bu repo **kod, config ve
  launch dosyalarını** içerir. Verileri/ağırlıkları yeniden üretmek için ilgili
  solution altındaki `training/` / `batch_run.py` betiklerini çalıştırın.
- Build çıktıları (`build/`, `devel/`, `install/`, `log/`, `__pycache__/`) da
  repoya dahil değildir; her workspace kendi ortamında yeniden derlenmelidir.

## Kurulum ve Çalıştırma

Ayrıntılı kurulum adımları için ilgili çalışma alanındaki
`iturover_obstacle_avoidance_assignment-main/README.md` dosyasına bakın
(ROS 1: catkin_ws, ROS 2: colcon `ros2_ws`).

Genel akış:

```bash
# ROS 1
catkin_make   # veya catkin build
source devel/setup.bash
roslaunch <solution_klasörü> reactive_robot.launch

# ROS 2
colcon build
source install/setup.bash
ros2 launch <solution_klasörü> reactive_robot.launch.py
```

## Yazar

Ahmet Efe Şensoy
