from pathlib import Path
from ui.theme.palette import PALETTE


class ThemeManager:
    @staticmethod
    def apply(app, mode: str = "dark"):
        mode = (mode or "dark").lower()
        if mode not in PALETTE:
            mode = "dark"

        p = PALETTE[mode]

        # QSS içine değişken basmak için :root benzeri yaklaşım
        qss_path = Path(__file__).with_name("styles.qss")
        qss = qss_path.read_text(encoding="utf-8")

        # CSS var(--x) yerine değerleri gömeceğiz
        # Basit replace: var(--bg) -> p["bg"]
        for k, v in p.items():
            qss = qss.replace(f"var(--{k})", v)

        app.setStyleSheet(qss)
