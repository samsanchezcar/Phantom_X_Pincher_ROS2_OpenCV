"""
Página de control manual por sliders.
"""
import time
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QSlider, QScrollArea
)
from PyQt5.QtCore import Qt

from ...config import DEFAULT_GOAL, DXL_MAX_VALUE


class ManualControlPage(QWidget):
    """Página de control manual mediante sliders."""
    
    def __init__(self, controller, get_speed_func, update_status_func):
        super().__init__()
        self.controller = controller
        self.get_speed = get_speed_func
        self.update_status = update_status_func
        
        self.sliders = {}
        self.slider_labels = {}
        self.last_motor_update = {mid: 0 for mid in controller.dxl_ids}
        self.update_interval = 0.05
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        title = QLabel("Control Manual por Sliders")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        for motor_id in self.controller.dxl_ids:
            motor_card = QFrame()
            motor_card.setObjectName("card")
            motor_layout = QVBoxLayout(motor_card)
            
            motor_title = QLabel(f"Motor {motor_id}")
            motor_title.setObjectName("motorLabel")
            motor_layout.addWidget(motor_title)
            
            slider_layout = QHBoxLayout()
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(DXL_MAX_VALUE)
            slider.setValue(DEFAULT_GOAL)
            slider.valueChanged.connect(
                lambda v, mid=motor_id: self.on_slider_change(mid)
            )
            slider_layout.addWidget(slider, 3)
            
            value_label = QLabel(f"{DEFAULT_GOAL}")
            value_label.setObjectName("valueLabel")
            value_label.setAlignment(Qt.AlignCenter)
            slider_layout.addWidget(value_label)
            
            motor_layout.addLayout(slider_layout)
            scroll_layout.addWidget(motor_card)
            
            self.sliders[motor_id] = slider
            self.slider_labels[motor_id] = value_label
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
    
    def on_slider_change(self, motor_id):
        """Maneja el cambio de valor en un slider."""
        current_time = time.time()
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            position = self.sliders[motor_id].value()
            speed = self.get_speed()
            
            self.slider_labels[motor_id].setText(str(position))
            
            if speed > 0 and not self.controller.emergency_stop_activated:
                self.controller.move_motor(motor_id, position)
                self.update_status(f"● Motor {motor_id} → {position}")
                self.last_motor_update[motor_id] = current_time
    
    def on_position_changed(self, motor_id, position):
        """Actualiza los sliders cuando la posición cambia externamente."""
        if motor_id in self.sliders:
            self.sliders[motor_id].blockSignals(True)
            self.sliders[motor_id].setValue(position)
            self.slider_labels[motor_id].setText(str(position))
            self.sliders[motor_id].blockSignals(False)
