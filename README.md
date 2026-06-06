# 🚗 TEKNOFEST 2026 — İnsansız Kara Aracı (İKA)

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-blue?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10%2B-yellow?logo=python&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Khadas%20VIM3-orange)
![OpenCV](https://img.shields.io/badge/OpenCV-4.8%2B-green?logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/Lisans-Apache%202.0-lightgrey)

**TEKNOFEST 2026 İnsansız Kara Aracı Yarışması için geliştirilmiş tam otonom / uzaktan kumandalı araç yazılımı.**  
ROS 2 tabanlı modüler mimari · Görüntü işleme (OpenCV + Tesseract) · PyQt5 Yer İstasyonu (GCS)

</div>

---

## 📋 İçindekiler

- [Proje Hakkında](#-proje-hakkında)
- [Sistem Mimarisi](#-sistem-mimarisi)
- [ROS 2 Node'ları](#-ros-2-nodeları)
- [Donanım](#-donanım)
- [Kurulum](#-kurulum)
- [Çalıştırma](#-çalıştırma)
- [Yer İstasyonu (GCS)](#-yer-i̇stasyonu-gcs)
- [Yapılandırma](#-yapılandırma)
- [Proje Yapısı](#-proje-yapısı)
- [Ekip](#-ekip)

---

## 🎯 Proje Hakkında

Bu proje, TEKNOFEST 2026 İKA (İnsansız Kara Aracı) kategorisinde yarışmak üzere geliştirilmiş tam kapsamlı bir robotik yazılım sistemidir. Araç; şerit takibi, koni/engel tespiti, tabela okuma (OCR) ve otonom sürüş gibi görevleri gerçekleştirirken Windows üzerinde çalışan bir **Yer Kontrol İstasyonu (GCS)** ile gerçek zamanlı izlenip kumanda edilebilmektedir.

### ✨ Öne Çıkan Özellikler

| Özellik | Detay |
|---------|-------|
| 🧠 **Otonom Karar Mekanizması** | `BrainNode` — MANUAL / AUTONOMOUS mod geçişi |
| 👁️ **Gerçek Zamanlı Görüntü İşleme** | Şerit takibi, koni tespiti, OCR (Tesseract) |
| 🛡️ **Donanım Güvenlik Katmanı** | Failsafe, anti-rollover, IMU tabanlı eğim kontrolü |
| ⚙️ **PID Motor Kontrolü** | Anti-windup, türev filtresi, thread-safe setpoint |
| 📡 **ROS 2 Haberleşmesi** | GCS ↔ Robot tam entegrasyon (ROS topic'ler üzerinden) |
| 🖥️ **PyQt5 GCS Arayüzü** | Canlı video, telemetri, manuel kumanda paneli |

---

## 🏗️ Sistem Mimarisi

```
┌──────────────────────────────────────────────────────────────────┐
│                      KHADAS VIM3 (Robot)                         │
│                                                                  │
│   ┌─────────────┐    ┌──────────────┐    ┌──────────────────┐   │
│   │ SensorNode  │───▶│  BrainNode   │───▶│   MotorNode      │   │
│   │  (50 Hz)    │    │ (Karar Mrk.) │    │  (PID + GPIO)    │   │
│   └─────────────┘    └──────┬───────┘    └──────────────────┘   │
│          ▲                  │                                    │
│   ┌──────┴──────┐    ┌──────▼───────┐                           │
│   │ IMU/UART    │    │PerceptionNode│                            │
│   │ (MPU6050)   │    │ (25 FPS Cam) │                           │
│   └─────────────┘    └──────────────┘                           │
│                                                                  │
└─────────────────────────┬────────────────────────────────────────┘
                          │ ROS 2 (Jazzy)
                          │ /telemetry · /cmd_vel · /drive_mode
                          │ /processed_frame · /imu_data
┌─────────────────────────▼────────────────────────────────────────┐
│               GCS — Windows (PyQt5 + ROS 2)                      │
│      gcs_ros_node.py · MainWindow · ManualDrivePanel             │
└──────────────────────────────────────────────────────────────────┘
```

---

## 🤖 ROS 2 Node'ları

### 🧭 `SensorNode` — `/sensor_node`
IMU (MPU6050) ve UART verilerini okuyarak ROS 2 ağına yayar.

| Topic | Tip | Açıklama |
|-------|-----|---------|
| `/imu_data` | `Float32MultiArray` | Roll, Pitch açıları (50 Hz) |
| `/system_status` | `String` | Kritik eğim uyarıları |
| `/telemetry` | `String` | GCS için JSON telemetri paketi |

### 👁️ `PerceptionNode` — `/perception_node`
USB kameradan frame alır, şerit/koni/tabela analizi yapar ve kararı Beyin'e iletir.

| Topic | Tip | Açıklama |
|-------|-----|---------|
| `/autonomy_cmd` | `Twist` | Throttle + steer kararı |
| `/processed_frame` | `Image` | İşlenmiş görüntü (GCS'e) |

### 🧠 `BrainNode` — `/brain_node`
Tüm veri akışlarını birleştirir. Mod yönetimi ve acil durum kontrolü yapar.

| Topic (Abone) | Açıklama |
|--------------|---------|
| `/autonomy_cmd` | Otonom modda PerceptionNode komutları |
| `/imu_data` | Eğim güvenlik kontrolü (>35° uyarı) |
| `/system_status` | Kritik hata → tüm motorlar kilit |
| `/drive_mode` | GCS'den mod değişikliği (MANUAL/AUTONOMOUS) |

| Topic (Yayın) | Açıklama |
|--------------|---------|
| `/cmd_vel` | Nihai hız/yön komutu (MotorNode'a) |

### ⚙️ `MotorNode` — `/motor_node`
Arka planda PID ve Safety thread'lerini çalıştırır, `/cmd_vel` komutlarını GPIO PWM'e çevirir.

---

## 🔧 Donanım

| Bileşen | Model / Detay |
|---------|--------------|
| **Ana Bilgisayar** | Khadas VIM3 (Ubuntu 24.04) |
| **IMU** | MPU6050 — I2C Bus 3 (`/dev/i2c-3`) |
| **Kamera** | USB Kamera — `/dev/video0` (640×480 @ 30 FPS) |
| **Motor Sürücü** | GPIO PWM — Sol: `pwmchip0`, Sağ: `pwmchip4` |
| **UART** | `/dev/ttyS3` @ 115200 baud |
| **Sol Motor GPIO** | Forward: Pin 35, Backward: Pin 37 |
| **Sağ Motor GPIO** | Forward: Pin 15, Backward: Pin 16 |

### VIM3 Overlay Ayarları (`/boot/env.txt`)
```
overlays=uart3 pwm_ao_a pwm_f i2c3
```

---

## ⚙️ Kurulum

### Gereksinimler

- **Robot (VIM3):** Ubuntu 24.04 + ROS 2 Jazzy Jalisco
- **GCS (PC):** Windows 10/11 + ROS 2 Jazzy Jalisco (binary)

### 1. Robot (Khadas VIM3)

```bash
# Sistem paketleri
sudo apt update && sudo apt install -y \
  python3-gpiod python3-smbus2 python3-serial \
  python3-opencv tesseract-ocr python3-pytesseract \
  ros-jazzy-rclpy ros-jazzy-cv-bridge \
  ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs ros-jazzy-launch ros-jazzy-launch-ros

# Python bağımlılıkları
pip install -r requirements_robot.txt

# ROS 2 paketini derle
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teknofest_ika
source install/setup.bash
```

### 2. GCS — Yer İstasyonu (Windows)

```bat
:: ROS 2 ortamını aktive et
call C:\dev\ros2_jazzy\setup.bat

:: Python bağımlılıkları
pip install -r requirements_gcs.txt
```

---

## 🚀 Çalıştırma

### Robot (VIM3)

```bash
# ROS 2 ortamını aktive et
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Tüm sistemi başlat (önerilen)
python start_robot.py

# Sistemi durdur
python start_robot.py stop

# Veya tek tek node başlatmak için:
ros2 run teknofest_ika sensor_node
ros2 run teknofest_ika perception_node
ros2 run teknofest_ika brain_node
ros2 run teknofest_ika motor_node
```

### GCS — Yer İstasyonu (Windows)

```bat
call C:\dev\ros2_jazzy\setup.bat
python ika_gui/ika_gcs/main.py
```

### Yararlı ROS 2 Komutları

```bash
# Aktif topic'leri listele
ros2 topic list

# Telemetri verisini izle
ros2 topic echo /telemetry

# IMU verisini izle
ros2 topic echo /imu_data

# Manuel mod geçişi
ros2 topic pub /drive_mode std_msgs/msg/String "data: 'AUTONOMOUS'"
ros2 topic pub /drive_mode std_msgs/msg/String "data: 'MANUAL'"
```

---

## 🖥️ Yer İstasyonu (GCS)

PyQt5 tabanlı GCS arayüzü şu özellikleri sunar:

- 📹 **Canlı Video** — ROS `/processed_frame` topic üzerinden
- 📊 **Telemetri Paneli** — Roll, Pitch, Batarya, Sıcaklık, Durum
- 🕹️ **Manuel Kumanda** — Klavye/joystick ile WASD kontrolü
- 🔄 **Mod Geçişi** — MANUAL ↔ AUTONOMOUS anlık değiştirme
- 🔌 **ROS 2 Entegrasyonu** — `gcs_ros_node.py` ile tam entegre

---

## 🔩 Yapılandırma

Tüm donanım ve yazılım parametreleri [`config/robot_params.yaml`](config/robot_params.yaml) dosyasında toplanmıştır:

```yaml
pid:
  kp: 1.2
  ki: 0.4
  kd: 0.15
  max_pwm: 70.0   # Güvenlik sınırı

failsafe:
  roll_stop_deg:  45.0   # Bu açı aşılırsa acil stop
  pitch_stop_deg: 45.0
  comm_timeout_s: 2.0

perception:
  target_number: "8"     # Hedef tabela numarası
  lane_steer_gain: 0.15  # Şerit P kazancı
```

---

## 📁 Proje Yapısı

```
TEKNOFEST_IKA_2026/
│
├── teknofest_ika/               # Ana ROS 2 paketi (Robot)
│   ├── nodes/
│   │   ├── brain_node.py        # 🧠 Karar mekanizması
│   │   ├── perception_node.py   # 👁️ Görüntü işleme
│   │   ├── sensor_node.py       # 🧭 IMU & UART
│   │   └── motor_node.py        # ⚙️ Motor kontrolü
│   ├── modules/
│   │   ├── perception.py        # OpenCV + Tesseract algoritmaları
│   │   ├── pid_thread.py        # PID + Failsafe thread'leri
│   │   ├── motor_driver.py      # GPIO PWM motor sürücüsü
│   │   └── serial_comms.py      # IMU + UART haberleşmesi
│   ├── core/
│   │   ├── vehicle_manager.py   # Araç durum yöneticisi
│   │   └── failsafe.py          # Güvenlik katmanı
│   └── utils/
│       └── camera_handler.py    # Kamera yönetimi
│
├── ika_gui/ika_gcs/             # Yer İstasyonu (GCS)
│   ├── main.py                  # Giriş noktası
│   ├── workers/
│   │   └── gcs_ros_node.py      # ROS 2 ↔ GCS köprüsü
│   ├── ui/
│   │   ├── main_window.py       # Ana pencere
│   │   ├── panels/
│   │   │   └── manual_drive_panel.py
│   │   └── widgets/
│   │       └── video_view.py    # Canlı video widget
│   └── app/
│       ├── config.py
│       └── constants.py
│
├── config/
│   └── robot_params.yaml        # Tüm donanım/yazılım parametreleri
│
├── start_robot.py               # Tek komutla sistem başlatıcı
├── requirements_robot.txt       # Robot Python bağımlılıkları
├── requirements_gcs.txt         # GCS Python bağımlılıkları
├── package.xml                  # ROS 2 paket tanımı
└── setup.py                     # ament_python kurulum betiği
```

---

## 👥 Ekip

| İsim | Rol |
|------|-----|
| **Yusuf Tufan** | Yazılım Geliştirici |
| **Muhammed Ulusoy** | Yazılım Kaptanı |
| **Zeynep** | Yazılım Geliştirici |
| **Hatice** | Yazılım Geliştirici |

---

<div align="center">

**TEKNOFEST 2026** &nbsp;|&nbsp; İnsansız Kara Aracı Kategorisi  
Türkiye'nin En Büyük Teknoloji Yarışması

</div>
