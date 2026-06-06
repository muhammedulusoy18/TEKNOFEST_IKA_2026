import sys
from PyQt5.QtWidgets import QApplication
from ui.theme.theme_manager import ThemeManager
from ui.main_window import MainWindow
from splash_screen import SplashScreen
from app.constants import APP_NAME, WINDOW_TITLE


def main():
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)

    ThemeManager.apply(app, mode="dark")

    splash = SplashScreen(vehicle_image_path="assets/vehicle.jpg")

    def on_login():
        win = MainWindow(app=app)
        win.show()

    splash.login_success.connect(on_login)
    splash.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()