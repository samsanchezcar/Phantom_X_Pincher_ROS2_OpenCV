"""
Página del panel principal (Dashboard).
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QGridLayout, QSlider
)
from PyQt5.QtCore import Qt

from ..widgets import create_card
from ...config import MAX_SPEED


class DashboardPage(QWidget):
    """Página del panel de control principal."""
    
    def __init__(self, controller, on_speed_change):
        super().__init__()
        self.controller = controller
        self.on_speed_change = on_speed_change
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Título
        title = QLabel("Panel de Control Principal")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Vista general del estado del robot")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        # Grid de tarjetas
        cards_layout = QGridLayout()
        cards_layout.setSpacing(20)
        
        # Tarjeta de posición
        pos_card = create_card("Posición Actual", self._create_position_widget())
        cards_layout.addWidget(pos_card, 0, 0)
        
        # Tarjeta de estado de motores
        motors_card = create_card("Estado de Motores", self._create_motors_status_widget())
        cards_layout.addWidget(motors_card, 0, 1)
        
        # Tarjeta de velocidad
        speed_card = create_card("Control de Velocidad", self._create_speed_widget())
        cards_layout.addWidget(speed_card, 1, 0, 1, 2)
        
        layout.addLayout(cards_layout)
        layout.addStretch()
    
    def _create_position_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.dash_x_label = QLabel("X: 0.000 m")
        self.dash_x_label.setStyleSheet("font-size: 13pt; color: #00d9ff; font-weight: bold;")
        layout.addWidget(self.dash_x_label)
        
        self.dash_y_label = QLabel("Y: 0.000 m")
        self.dash_y_label.setStyleSheet("font-size: 13pt; color: #00d9ff; font-weight: bold;")
        layout.addWidget(self.dash_y_label)
        
        self.dash_z_label = QLabel("Z: 0.000 m")
        self.dash_z_label.setStyleSheet("font-size: 13pt; color: #00d9ff; font-weight: bold;")
        layout.addWidget(self.dash_z_label)
        
        return widget
    
    def _create_motors_status_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        for motor_id in self.controller.dxl_ids:
            motor_label = QLabel(f"Motor {motor_id}: ● Activo")
            motor_label.setStyleSheet("color: #00d9ff; padding: 5px;")
            layout.addWidget(motor_label)
        
        return widget
    
    def _create_speed_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        slider_layout = QHBoxLayout()
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(MAX_SPEED)
        self.speed_slider.setValue(100)
        self.speed_slider.valueChanged.connect(self._on_speed_slider_change)
        slider_layout.addWidget(self.speed_slider, 3)
        
        self.speed_value_label = QLabel("100")
        self.speed_value_label.setObjectName("valueLabel")
        self.speed_value_label.setStyleSheet("font-size: 16pt;")
        slider_layout.addWidget(self.speed_value_label)
        
        layout.addLayout(slider_layout)
        
        return widget
    
    def _on_speed_slider_change(self, value):
        self.speed_value_label.setText(str(value))
        self.on_speed_change(value)
    
    def update_position(self, x, y, z):
        """Actualiza la visualización de posición."""
        self.dash_x_label.setText(f"X: {x:.3f} m")
        self.dash_y_label.setText(f"Y: {y:.3f} m")
        self.dash_z_label.setText(f"Z: {z:.3f} m")
    
    def get_speed(self):
        """Retorna el valor actual del slider de velocidad."""
        return self.speed_slider.value()
