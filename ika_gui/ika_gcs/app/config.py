"""
config.py — IKA GCS Konfigürasyonu
Uygulama genelinde ayarlar bu singleton üzerinden erişilir.
"""
from app.constants import (
    DEFAULT_TCP_HOST, DEFAULT_TCP_PORT, DEFAULT_UDP_PORT,
    WINDOW_MIN_W, WINDOW_MIN_H
)


class AppConfig:
    """Uygulama konfigürasyon singleton'ı."""
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init_defaults()
        return cls._instance

    def _init_defaults(self):
        self.tcp_host  = DEFAULT_TCP_HOST
        self.tcp_port  = DEFAULT_TCP_PORT
        self.udp_port  = DEFAULT_UDP_PORT
        self.theme     = "dark"
        self.min_w     = WINDOW_MIN_W
        self.min_h     = WINDOW_MIN_H


# Modül seviyesi erişim kolaylığı
config = AppConfig()
