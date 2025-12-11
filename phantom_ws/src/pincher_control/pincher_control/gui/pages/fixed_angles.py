"""
Página para movimientos a ángulos predefinidos.
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QFrame, QPushButton,
    QGridLayout, QMessageBox
)
from PyQt5.QtCore import QTimer

from ...config import PRESET_POSITIONS


class FixedAnglesPage(QWidget):
    """Página para movimientos a ángulos predefinidos."""
    
    def __init__(self, controller, update_status_func):
        super().__init__()
        self.controller = controller
        self.update_status = update_status_func
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        title = QLabel("Movimientos a Ángulos Predefinidos")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Posiciones predefinidas con un solo clic")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        grid = QGridLayout()
        grid.setSpacing(20)
        
        for i, (name, angles, desc) in enumerate(PRESET_POSITIONS):
            card = QFrame()
            card.setObjectName("card")
            card_layout = QVBoxLayout(card)
            
            title_label = QLabel(name)
            title_label.setObjectName("sectionTitle")
            card_layout.addWidget(title_label)
            
            desc_label = QLabel(desc)
            desc_label.setStyleSheet("color: #b0b0c0; padding: 5px;")
            desc_label.setWordWrap(True)
            card_layout.addWidget(desc_label)
            
            angles_str = f"[{', '.join(f'{a}°' for a in angles)}]"
            angles_label = QLabel(angles_str)
            angles_label.setStyleSheet(
                "color: #00d9ff; font-weight: bold; padding: 10px; font-size: 12pt;"
            )
            card_layout.addWidget(angles_label)
            
            btn = QPushButton("▶ Ejecutar Movimiento")
            btn.setObjectName("presetButton")
            btn.clicked.connect(
                lambda checked, a=angles: self.move_to_preset(a)
            )
            card_layout.addWidget(btn)
            
            row = i // 2
            col = i % 2
            grid.addWidget(card, row, col)
        
        layout.addLayout(grid)
        layout.addStretch()
    
    def move_to_preset(self, angles):
        """Mueve el robot a una posición predefinida."""
        if self.controller.emergency_stop_activated:
            QMessageBox.warning(
                self, "Advertencia", "Sistema en parada de emergencia"
            )
            return
        
        success = self.controller.move_to_angles_degrees(angles)
        
        if success:
            self.update_status(f"● Movido a: {angles}")
            QTimer.singleShot(3000, lambda: self.update_status("● Sistema Listo"))
        else:
            QMessageBox.critical(
                self, "Error", "No se pudo completar el movimiento"
            )
