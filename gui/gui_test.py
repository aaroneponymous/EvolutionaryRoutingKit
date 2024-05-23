import sys
from PyQt5.QtWidgets import QApplication, QMainWindow


def window():
    app = QApplication(sys.argv)
    widget = QMainWindow()
    widget.setWindowTitle('Test')
    widget.setGeometry(100, 100, 600, 400)
    widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    window()