from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt


def build_status_bar(main_window):
    sb = main_window.statusBar()

    # TCP/UDP durum
    main_window._status_conn = QLabel("❌ TCP: Bağlı Değil   |   📡 UDP: Kapalı")
    main_window._status_conn.setAlignment(Qt.AlignLeft)

    # Mod
    main_window._status_mode = QLabel("Mod: Manuel")
    main_window._status_mode.setAlignment(Qt.AlignLeft)

    # Batarya
    main_window._status_batt = QLabel("Batarya: --%")
    main_window._status_batt.setAlignment(Qt.AlignLeft)

    # Status bar'a ekle
    sb.addPermanentWidget(main_window._status_conn)
    sb.addPermanentWidget(main_window._status_mode)
    sb.addPermanentWidget(main_window._status_batt)

    sb.showMessage("Hazır")