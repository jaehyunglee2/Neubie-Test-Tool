import sys
import json
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QFileDialog, QMessageBox
from PyQt5.QtCore import QThread, pyqtSignal
from geojson import Point, LineString, Feature, FeatureCollection

class ConverterThread(QThread):
    finished = pyqtSignal(str)
    
    def __init__(self, file_path):
        super().__init__()
        self.file_path = file_path

    def run(self):
        try:
            with open(self.file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
                geojson_data = self.convert_to_geojson(data)
                output_path = self.file_path.replace('.json', '.geojson')
                with open(output_path, 'w', encoding='utf-8') as outfile:
                    json.dump(geojson_data, outfile, ensure_ascii=False, indent=2)
                self.finished.emit(f"GeoJSON 변환이 완료되었습니다: {output_path}")
        except Exception as e:
            self.finished.emit(f"오류 발생: {str(e)}")

    def convert_to_geojson(self, data):
        features = []
        for item in data['DATA']:
            if item['node_type'] == 'LINK' and item['lnkg_wkt']:
                coords = [
                    [float(coord.split()[0]), float(coord.split()[1])]
                    for coord in item['lnkg_wkt'].replace('LINESTRING(', '').replace(')', '').split(',')
                ]
                geometry = LineString(coords)
            elif item['node_type'] == 'NODE' and item['node_wkt']:
                coord = item['node_wkt'].replace('POINT(', '').replace(')', '').split()
                geometry = Point((float(coord[0]), float(coord[1])))
            else:
                continue

            feature = Feature(geometry=geometry, properties={key: item[key] for key in item if key not in ['lnkg_wkt', 'node_wkt']})
            features.append(feature)

        return FeatureCollection(features)


class App(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('JSON to GeoJSON Converter')
        
        self.layout = QVBoxLayout()

        self.convert_button = QPushButton('JSON 파일 불러오기', self)
        self.convert_button.clicked.connect(self.load_json_file)
        
        self.layout.addWidget(self.convert_button)
        
        self.setLayout(self.layout)
        self.resize(300, 100)

    def load_json_file(self):
        file_path = QFileDialog.getOpenFileName(self, 'JSON 파일 선택', '', 'JSON files (*.json)')[0]
        if file_path:
            self.convert_button.setEnabled(False)
            self.thread = ConverterThread(file_path)
            self.thread.finished.connect(self.on_conversion_finished)
            self.thread.start()

    def on_conversion_finished(self, message):
        self.convert_button.setEnabled(True)
        QMessageBox.information(self, '완료', message)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    ex.show()
    sys.exit(app.exec_())
