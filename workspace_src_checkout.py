import sys
import os
import shutil
import subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QMessageBox

class GitManagerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Git Manager GUI')
        self.setGeometry(100, 100, 500, 300)

        layout = QVBoxLayout()

        self.folderLabel = QLabel('Select Working Folder:', self)
        layout.addWidget(self.folderLabel)

        self.folderButton = QPushButton('Browse Folder', self)
        self.folderButton.clicked.connect(self.selectFolder)
        layout.addWidget(self.folderButton)

        self.fileLabel = QLabel('Select git information.txt:', self)
        layout.addWidget(self.fileLabel)

        self.fileButton = QPushButton('Browse File', self)
        self.fileButton.clicked.connect(self.selectFile)
        layout.addWidget(self.fileButton)

        self.processButton = QPushButton('Process', self)
        self.processButton.clicked.connect(self.processRepositories)
        layout.addWidget(self.processButton)

        self.setLayout(layout)

    def selectFolder(self):
        options = QFileDialog.Options()
        folderPath = QFileDialog.getExistingDirectory(self, "Select Working Folder", options=options)
        if folderPath:
            self.folderPath = folderPath
            self.folderLabel.setText(f'Selected Folder: {folderPath}')

    def selectFile(self):
        options = QFileDialog.Options()
        filePath, _ = QFileDialog.getOpenFileName(self, "Select git information.txt", "", "Text Files (*.txt);;All Files (*)", options=options)
        if filePath:
            self.filePath = filePath
            self.fileLabel.setText(f'Selected File: {filePath}')

    def processRepositories(self):
        if not hasattr(self, 'folderPath') or not hasattr(self, 'filePath'):
            QMessageBox.warning(self, 'Error', 'Please select both a folder and a file.')
            return

        srcDir = os.path.join(self.folderPath, 'src')
        if not os.path.isdir(srcDir):
            QMessageBox.warning(self, 'Error', 'src directory does not exist in the selected folder.')
            return

        try:
            with open(self.filePath, 'r') as file:
                git_info = {}
                for line in file:
                    if line.strip():
                        name, url = line.split()
                        git_info[name] = url

            for folder_name in os.listdir(srcDir):
                folder_path = os.path.join(srcDir, folder_name)
                if os.path.isdir(folder_path):
                    if folder_name in git_info:
                        subprocess.run(['git', '-C', folder_path, 'checkout', git_info[folder_name]])
                    else:
                        shutil.rmtree(folder_path)
            
            # colcon build 실행
            self.runColconBuild()

            QMessageBox.information(self, 'Success', 'Repositories processed and colcon build executed successfully.')
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'An error occurred: {e}')

    def runColconBuild(self):
        try:
            subprocess.run(['colcon', 'build'], cwd=self.folderPath, check=True)
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, 'Error', f'An error occurred while running colcon build: {e}')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = GitManagerApp()
    ex.show()
    sys.exit(app.exec_())


# 빌드 중 퍼셉션 2d 에러날 때 perception_2d / perception_2d_core 안의 CMakeLists.txt package.xml를 삭제 후 빌드
# 사용방법
# 1. 기존 워스페이스 복사/붙여넣기
# 2. 복사한 웤스페이스/에서 install, build, log 폴더 삭제
# 3. 프로그램 실행 시킨 뒤 복사한 웤스페이스 선택
# 4. repo-git-information.txt 선택
# 5. proceed 버튼 클릭
# 6. colcon build 터미널 진행은 되나 에러 뜸 (build 중 에러라 모름)
# 필요할 수 있는 내용 : git id/token 정보 (현재는 글로벌이라 상관없이 되지만 나중에 토큰 변경되면 에러 뜰거임 이거 대비 필요)
# 수찬님이 만든 참고 링크 : https://neubility.atlassian.net/wiki/spaces/AUT/pages/766083179/work+space