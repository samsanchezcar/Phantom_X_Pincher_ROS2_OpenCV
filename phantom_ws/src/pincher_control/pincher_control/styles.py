"""
Estilos CSS para la interfaz gráfica del PhantomX Pincher.
"""

MODERN_STYLESHEET = """
* {
    font-family: 'Segoe UI', 'Ubuntu', sans-serif;
}

QMainWindow {
    background-color: #1e1e2e;
}

QWidget#centralWidget {
    background-color: #1e1e2e;
}

QWidget#contentArea {
    background-color: #2a2a3e;
    border-radius: 15px;
}

QFrame#sidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                stop:0 #0f0f1e, stop:1 #1a1a2e);
    border-right: 2px solid #00d9ff;
}

QPushButton#menuButton {
    background-color: transparent;
    color: #b0b0c0;
    text-align: left;
    padding: 18px 20px;
    border: none;
    border-left: 4px solid transparent;
    font-size: 13pt;
    font-weight: 500;
}

QPushButton#menuButton:hover {
    background-color: #2a2a3e;
    color: #00d9ff;
    border-left: 4px solid #00d9ff;
}

QPushButton#menuButton:checked {
    background-color: #0f4c75;
    color: #00d9ff;
    border-left: 4px solid #00d9ff;
    font-weight: bold;
}

QPushButton#homeButtonSidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #6a11cb, stop:1 #2575fc);
    color: white;
    padding: 15px 20px;
    border: none;
    border-radius: 8px;
    font-size: 12pt;
    font-weight: bold;
    margin: 10px;
}

QPushButton#homeButtonSidebar:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #7f1ed6, stop:1 #3d87ff);
}

QPushButton#emergencyButtonSidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #ff416c, stop:1 #ff4b2b);
    color: white;
    padding: 15px 20px;
    border: none;
    border-radius: 8px;
    font-size: 12pt;
    font-weight: bold;
    margin: 10px;
}

QPushButton#emergencyButtonSidebar:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #ff5a7f, stop:1 #ff6747);
}

QLabel#pageTitle {
    color: #00d9ff;
    font-size: 24pt;
    font-weight: bold;
    padding: 10px;
}

QLabel#sectionTitle {
    color: #00d9ff;
    font-size: 16pt;
    font-weight: bold;
    padding: 8px;
}

QLabel#subtitle {
    color: #b0b0c0;
    font-size: 11pt;
    padding: 5px;
}

QFrame#card {
    background-color: #252538;
    border-radius: 12px;
    border: 1px solid #3a3a4e;
    padding: 15px;
}

QFrame#card:hover {
    border: 1px solid #00d9ff;
}

QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    color: #ffffff;
    border: none;
    padding: 12px 25px;
    border-radius: 8px;
    font-weight: bold;
    font-size: 11pt;
}

QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00d9ff, stop:1 #00e5ff);
}

QPushButton:pressed {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #008ba3, stop:1 #00a8cc);
}

QPushButton:disabled {
    background-color: #3a3a4e;
    color: #6a6a7e;
}

QPushButton#presetButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #667eea, stop:1 #764ba2);
    padding: 20px;
    font-size: 12pt;
    min-height: 80px;
}

QPushButton#presetButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #7b92f5, stop:1 #8a5fb5);
}

QLabel {
    color: #e0e0e0;
    font-size: 10pt;
}

QLabel#statusLabel {
    background-color: #252538;
    border: 2px solid #00d9ff;
    border-radius: 8px;
    padding: 10px;
    font-weight: bold;
    font-size: 11pt;
    color: #00d9ff;
}

QLabel#motorLabel {
    color: #00d9ff;
    font-weight: bold;
    font-size: 11pt;
}

QLabel#valueLabel {
    background-color: #1a1a2e;
    border-radius: 5px;
    padding: 8px;
    font-weight: bold;
    color: #00d9ff;
    min-width: 60px;
}

QLineEdit {
    background-color: #1a1a2e;
    border: 2px solid #3a3a4e;
    border-radius: 6px;
    padding: 10px;
    color: #e0e0e0;
    font-size: 11pt;
}

QLineEdit:focus {
    border: 2px solid #00d9ff;
}

QSlider::groove:horizontal {
    border: none;
    height: 10px;
    background: #1a1a2e;
    border-radius: 5px;
}

QSlider::handle:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    border: 2px solid #00d9ff;
    width: 22px;
    margin: -6px 0;
    border-radius: 11px;
}

QSlider::handle:horizontal:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00d9ff, stop:1 #00e5ff);
}

QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    border-radius: 5px;
}

QScrollArea {
    border: none;
    background-color: transparent;
}

QScrollBar:vertical {
    background-color: #1a1a2e;
    width: 12px;
    border-radius: 6px;
}

QScrollBar::handle:vertical {
    background-color: #00d9ff;
    border-radius: 6px;
    min-height: 20px;
}

QScrollBar::handle:vertical:hover {
    background-color: #00e5ff;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
"""
