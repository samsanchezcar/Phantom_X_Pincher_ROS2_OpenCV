"""
Widgets comunes reutilizables para la GUI.
"""
from PyQt5.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, 
    QPushButton, QSlider, QLineEdit
)
from PyQt5.QtCore import Qt, pyqtSignal


class PositionSignal(QWidget):
    """Widget para emitir señales de cambio de posición."""
    changed = pyqtSignal(int, int)  # motor_id, position


def create_card(title, content_widget):
    """
    Crea una tarjeta con título y contenido.
    
    Args:
        title: Título de la tarjeta
        content_widget: Widget con el contenido
    
    Returns:
        QFrame: Tarjeta estilizada
    """
    card = QFrame()
    card.setObjectName("card")
    layout = QVBoxLayout(card)
    
    title_label = QLabel(title)
    title_label.setObjectName("sectionTitle")
    layout.addWidget(title_label)
    
    layout.addWidget(content_widget)
    
    return card


def create_motor_slider(motor_id, default_value, max_value, on_change_callback):
    """
    Crea un control deslizante para un motor.
    
    Args:
        motor_id: ID del motor
        default_value: Valor inicial
        max_value: Valor máximo
        on_change_callback: Función a llamar cuando cambia el valor
    
    Returns:
        tuple: (QFrame card, QSlider slider, QLabel value_label)
    """
    motor_card = QFrame()
    motor_card.setObjectName("card")
    motor_layout = QVBoxLayout(motor_card)
    
    motor_title = QLabel(f"Motor {motor_id}")
    motor_title.setObjectName("motorLabel")
    motor_layout.addWidget(motor_title)
    
    slider_layout = QHBoxLayout()
    
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(0)
    slider.setMaximum(max_value)
    slider.setValue(default_value)
    slider.valueChanged.connect(lambda v: on_change_callback(motor_id))
    slider_layout.addWidget(slider, 3)
    
    value_label = QLabel(f"{default_value}")
    value_label.setObjectName("valueLabel")
    value_label.setAlignment(Qt.AlignCenter)
    slider_layout.addWidget(value_label)
    
    motor_layout.addLayout(slider_layout)
    
    return motor_card, slider, value_label


def create_xyz_input(label_text, default_val):
    """
    Crea un campo de entrada para coordenadas XYZ.
    
    Args:
        label_text: Texto de la etiqueta
        default_val: Valor por defecto
    
    Returns:
        tuple: (QVBoxLayout layout, QLineEdit entry)
    """
    v_layout = QVBoxLayout()
    
    lbl = QLabel(label_text)
    lbl.setStyleSheet("font-weight: bold; color: #00d9ff;")
    v_layout.addWidget(lbl)
    
    entry = QLineEdit(default_val)
    entry.setFixedWidth(120)
    v_layout.addWidget(entry)
    
    return v_layout, entry
