import sys
import threading
from PySide6.QtWidgets import QApplication

import main_ui as ceo

def init_ceo_ui():
    app = QApplication(sys.argv)
    win = ceo.MainWindow()
    win.show()
    sys.exit(app.exec())

t1 = threading.Thread(target=init_ceo_ui)
t1.start()