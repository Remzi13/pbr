import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QPushButton, QLabel,
    QFileDialog
)
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt

class PPMViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 PPM Viewer")
        self.setGeometry(100, 100, 800, 600)

        # 1. Создание центрального виджета и компоновки
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        self.layout = QVBoxLayout(central_widget)

        # 2. Метка для отображения изображения
        self.image_label = QLabel("Нажмите 'Открыть файл', чтобы загрузить PPM изображение.")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("border: 1px solid #ccc;")
        
        # Разрешаем метке расширяться, чтобы показать изображение
        self.image_label.setScaledContents(True)

        self.layout.addWidget(self.image_label)

        # 3. Кнопка для открытия файла
        self.open_button = QPushButton("📂 Открыть PPM файл")
        self.open_button.clicked.connect(self.open_image_file)
        self.layout.addWidget(self.open_button)

    def open_image_file(self):
        # Открытие диалогового окна для выбора файла
        file_path, _ = QFileDialog.getOpenFileName(
            self,                                          # Родительское окно
            "Открыть PPM Изображение",                     # Заголовок
            "",                                            # Начальная директория
            "PPM Files (*.ppm);;All Files (*)"             # Фильтр файлов
        )

        if file_path:
            # Загрузка изображения в QPixmap
            pixmap = QPixmap(file_path)
            
            if pixmap.isNull():
                # Проверка на ошибку загрузки (например, если файл поврежден)
                self.image_label.setText(
                    f"⚠️ Не удалось загрузить изображение из файла: {file_path}. Возможно, файл поврежден или не является действительным PPM."
                )
            else:
                # 4. Отображение изображения
                
                # Устанавливаем QPixmap в QLabel. 
                # QPixmap автоматически масштабируется, так как у QLabel установлен setScaledContents(True)
                self.image_label.setPixmap(pixmap)
                self.setWindowTitle(f"PyQt6 PPM Viewer - {file_path}")
                self.image_label.setText("") # Очистка текста-заглушки

if __name__ == '__main__':
    # 5. Запуск приложения
    app = QApplication(sys.argv)
    viewer = PPMViewer()
    viewer.show()
    sys.exit(app.exec())