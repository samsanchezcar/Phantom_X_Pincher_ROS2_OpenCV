"""
Página para mover motores a valores fijos.
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QLineEdit, QPushButton, QScrollArea, QMessageBox
)
from PyQt5.QtCore import QTimer

from ...config import DEFAULT_GOAL, DXL_MAX_VALUE


class FixedValuesPage(QWidget):
    """Página para mover motores a valores específicos."""
    
    def __init__(self, controller, get_speed_func, update_status_func):
        super().__init__()
        self.controller = controller
        self.get_speed = get_speed_func
        self.update_status = update_status_func
        
        self.value_entries = {}
        self.value_entry_labels = {}
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        title = QLabel("Mover a Valores Fijos")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Ingresa valores específicos para cada motor")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        for motor_id in self.controller.dxl_ids:
            motor_card = QFrame()
            motor_card.setObjectName("card")
            motor_layout = QHBoxLayout(motor_card)
            
            motor_title = QLabel(f"Motor {motor_id}")
            motor_title.setObjectName("motorLabel")
            motor_title.setMinimumWidth(100)
            motor_layout.addWidget(motor_title)
            
            entry_label = QLabel(f"Valor (0-{DXL_MAX_VALUE}):")
            motor_layout.addWidget(entry_label)
            
            entry = QLineEdit(str(DEFAULT_GOAL))
            entry.setMaximumWidth(120)
            entry.returnPressed.connect(
                lambda mid=motor_id: self.move_single_motor(mid)
            )
            motor_layout.addWidget(entry)
            
            move_btn = QPushButton("Mover")
            move_btn.clicked.connect(
                lambda checked, mid=motor_id: self.move_single_motor(mid)
            )
            motor_layout.addWidget(move_btn)
            
            status_label = QLabel("Listo")
            status_label.setStyleSheet("color: #00d9ff; font-weight: bold;")
            motor_layout.addWidget(status_label)
            
            motor_layout.addStretch()
            
            scroll_layout.addWidget(motor_card)
            
            self.value_entries[motor_id] = entry
            self.value_entry_labels[motor_id] = status_label
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        # Botón mover todos
        move_all_btn = QPushButton("▶ MOVER TODOS LOS MOTORES")
        move_all_btn.setStyleSheet("font-size: 13pt; padding: 15px;")
        move_all_btn.clicked.connect(self.move_all_motors)
        layout.addWidget(move_all_btn)
    
    def move_single_motor(self, motor_id):
        """Mueve un motor individual al valor especificado."""
        try:
            value = self.value_entries[motor_id].text()
            position = int(value)
            
            if 0 <= position <= DXL_MAX_VALUE:
                speed = self.get_speed()
                
                if speed == 0:
                    self._set_status(motor_id, "Velocidad 0", "orange")
                elif self.controller.emergency_stop_activated:
                    self._set_status(motor_id, "EMERGENCIA", "red")
                else:
                    self.controller.move_motor(motor_id, position)
                    self._set_status(motor_id, "Enviado", "blue")
                    self.update_status(f"● Motor {motor_id} → {position}")
                    QTimer.singleShot(2000, lambda: self._reset_status(motor_id))
            else:
                self._set_status(motor_id, f"Error: 0-{DXL_MAX_VALUE}", "red")
        except ValueError:
            self._set_status(motor_id, "Error: Número", "red")
    
    def move_all_motors(self):
        """Mueve todos los motores a sus valores especificados."""
        speed = self.get_speed()
        
        if speed == 0:
            self.update_status("● Velocidad 0: Los motores no se moverán")
            return
        
        if self.controller.emergency_stop_activated:
            QMessageBox.warning(self, "Advertencia", "Sistema en parada de emergencia")
            return
        
        success_count = 0
        for motor_id in self.controller.dxl_ids:
            try:
                value = self.value_entries[motor_id].text()
                position = int(value)
                
                if 0 <= position <= DXL_MAX_VALUE:
                    self.controller.move_motor(motor_id, position)
                    self._set_status(motor_id, "Enviado", "blue")
                    success_count += 1
            except ValueError:
                self._set_status(motor_id, "Error", "red")
        
        self.update_status(
            f"● {success_count}/{len(self.controller.dxl_ids)} motores movidos"
        )
    
    def _set_status(self, motor_id, text, color):
        """Establece el estado de un motor."""
        self.value_entry_labels[motor_id].setText(text)
        self.value_entry_labels[motor_id].setStyleSheet(
            f"color: {color}; font-weight: bold;"
        )
    
    def _reset_status(self, motor_id):
        """Restablece el estado de un motor."""
        self._set_status(motor_id, "Listo", "#00d9ff")
