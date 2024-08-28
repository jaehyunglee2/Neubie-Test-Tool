from PyQt5.QtWidgets import QApplication
import sys
from app.neubie_test_tool import MainWindow

def main():
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
