# -*- coding: utf-8 -*-
import os
import logging
from datetime import datetime


def get_log_path() -> str:
    """logs/ klasörünü oluşturur ve log dosyasının yolunu döner."""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    logs_dir = os.path.join(base_dir, "logs")
    os.makedirs(logs_dir, exist_ok=True)
    filename = datetime.now().strftime("gcs_%Y-%m-%d_%H-%M.log")
    return os.path.join(logs_dir, filename)


def setup_file_logger(name: str = "gcs") -> logging.Logger:
    """Dosyaya yazan logger oluşturur."""
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger  # zaten kurulu

    logger.setLevel(logging.DEBUG)

    handler = logging.FileHandler(get_log_path(), encoding="utf-8")
    handler.setLevel(logging.DEBUG)

    formatter = logging.Formatter(
        fmt="[%(asctime)s] %(levelname)-8s %(message)s",
        datefmt="%H:%M:%S"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger


# Uygulama genelinde tek logger instance
_logger = setup_file_logger("gcs")


def log(message: str, level: str = "INFO"):
    """LogPanel'den çağrılan ana fonksiyon."""
    level = level.upper()
    if level == "DEBUG":
        _logger.debug(message)
    elif level == "WARNING":
        _logger.warning(message)
    elif level == "ERROR":
        _logger.error(message)
    elif level == "SUCCESS":
        _logger.info(f"[SUCCESS] {message}")
    else:
        _logger.info(message)