#!/usr/bin/env python3
"""
Khadas VIM3 — Sistem Kontrol Betiği
====================================
Bu betik, robot sistemini tek komutla başlatır veya durdurur.
ROS 2 kurulumunun aktif olduğunu varsayar (source /opt/ros/jazzy/setup.bash)
"""

import subprocess
import sys
import os

def check_ros():
    result = subprocess.run(["ros2", "topic", "list"], capture_output=True, timeout=3)
    return result.returncode == 0

def start_robot():
    print("=" * 50)
    print("  TEKNOFEST IKA 2026 — BAŞLATILIYOR")
    print("=" * 50)

    if not check_ros():
        print("[HATA] ROS 2 ortamı bulunamadı!")
        print("[İPUCU] Önce şunu çalıştırın: source /opt/ros/jazzy/setup.bash")
        sys.exit(1)

    print("[OK] ROS 2 ortamı hazır.")
    print("[BAŞLATILIYOR] ros2 launch teknofest_ika robot_launch.py")

    os.execlp(
        "ros2", "ros2", "launch", "teknofest_ika", "robot_launch.py"
    )

def stop_robot():
    print("[DURDUR] Tüm IKA node'ları kapatılıyor...")
    subprocess.run(["pkill", "-f", "perception_node"])
    subprocess.run(["pkill", "-f", "sensor_node"])
    subprocess.run(["pkill", "-f", "motor_node"])
    subprocess.run(["pkill", "-f", "brain_node"])
    print("[OK] Tüm node'lar kapatıldı.")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "stop":
        stop_robot()
    else:
        start_robot()
