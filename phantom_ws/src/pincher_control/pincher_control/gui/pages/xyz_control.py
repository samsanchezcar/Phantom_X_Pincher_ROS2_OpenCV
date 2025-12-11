"""
Página de control por posición XYZ.
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, 
    QLineEdit, QPushButton, QGridLayout, QMessageBox, QApplication
)

from ...config import XYZ_PRESETS
from ..widgets import create_xyz_input


class XYZControlPage(QWidget):
    """Página de control por posición XYZ."""
    
    def __init__(self, controller, update_status_func):
        super().__init__()
        self.controller = controller
        self.update_status = update_status_func
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        title = QLabel("Control por Posición XYZ")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # Tarjeta de posición actual
        current_card = QFrame()
        current_card.setObjectName("card")
        current_layout = QVBoxLayout(current_card)
        
        current_title = QLabel("Posición Actual del End-Effector")
        current_title.setObjectName("sectionTitle")
        current_layout.addWidget(current_title)
        
        pos_layout = QHBoxLayout()
        
        self.current_x_label = QLabel("X: 0.000 m")
        self.current_x_label.setStyleSheet(
            "font-size: 14pt; color: #00d9ff; font-weight: bold;"
        )
        pos_layout.addWidget(self.current_x_label)
        
        self.current_y_label = QLabel("Y: 0.000 m")
        self.current_y_label.setStyleSheet(
            "font-size: 14pt; color: #00d9ff; font-weight: bold;"
        )
        pos_layout.addWidget(self.current_y_label)
        
        self.current_z_label = QLabel("Z: 0.000 m")
        self.current_z_label.setStyleSheet(
            "font-size: 14pt; color: #00d9ff; font-weight: bold;"
        )
        pos_layout.addWidget(self.current_z_label)
        
        current_layout.addLayout(pos_layout)
        layout.addWidget(current_card)
        
        # Tarjeta de posición objetivo
        target_card = QFrame()
        target_card.setObjectName("card")
        target_layout = QVBoxLayout(target_card)
        
        target_title = QLabel("Posición Objetivo")
        target_title.setObjectName("sectionTitle")
        target_layout.addWidget(target_title)
        
        inputs_layout = QHBoxLayout()
        
        x_layout, self.x_entry = create_xyz_input("X (m)", "0.200")
        inputs_layout.addLayout(x_layout)
        
        y_layout, self.y_entry = create_xyz_input("Y (m)", "0.000")
        inputs_layout.addLayout(y_layout)
        
        z_layout, self.z_entry = create_xyz_input("Z (m)", "0.100")
        inputs_layout.addLayout(z_layout)
        
        target_layout.addLayout(inputs_layout)
        
        move_btn = QPushButton("🎯 MOVER A POSICIÓN XYZ")
        move_btn.setFixedHeight(50)
        move_btn.setStyleSheet("font-size: 13pt;")
        move_btn.clicked.connect(self.move_to_target)
        target_layout.addWidget(move_btn)
        
        layout.addWidget(target_card)
        
        # Tarjeta de posiciones rápidas
        presets_card = QFrame()
        presets_card.setObjectName("card")
        presets_layout = QVBoxLayout(presets_card)
        
        presets_title = QLabel("Posiciones Rápidas")
        presets_title.setObjectName("sectionTitle")
        presets_layout.addWidget(presets_title)
        
        presets_grid = QGridLayout()
        
        for i, (name, x, y, z) in enumerate(XYZ_PRESETS):
            btn = QPushButton(f"{name}\n({x:.2f}, {y:.2f}, {z:.2f})")
            btn.setFixedHeight(70)
            btn.clicked.connect(
                lambda checked, px=x, py=y, pz=z: self.move_to_preset(px, py, pz)
            )
            presets_grid.addWidget(btn, 0, i)
        
        presets_layout.addLayout(presets_grid)
        layout.addWidget(presets_card)
        
        # Label de estado IK
        self.ik_status_label = QLabel("✓ Listo para mover")
        self.ik_status_label.setStyleSheet(
            "color: #00d9ff; font-size: 12pt; padding: 10px;"
        )
        layout.addWidget(self.ik_status_label)
        
        layout.addStretch()
    
    def move_to_target(self):
        """Mueve el robot a la posición XYZ especificada."""
        try:
            x = float(self.x_entry.text())
            y = float(self.y_entry.text())
            z = float(self.z_entry.text())
            
            r = (x**2 + y**2) ** 0.5
            
            # Verificar límites
            if not (self.controller.planar_reach_min <= r <= 
                    self.controller.planar_reach_max + 0.01):
                QMessageBox.warning(
                    self, "Error", f"Fuera de alcance radial: {r:.3f} m"
                )
                return
            
            if not (self.controller.z_min - 0.01 <= z <= 
                    self.controller.z_max + 0.01):
                QMessageBox.warning(
                    self, "Error", f"Fuera de alcance en Z: {z:.3f} m"
                )
                return
            
            self.ik_status_label.setText("⏳ Calculando IK...")
            QApplication.processEvents()
            
            success = self.controller.move_to_xyz(x, y, z)
            
            if success:
                self.ik_status_label.setText(
                    f"✓ Movido a ({x:.3f}, {y:.3f}, {z:.3f})"
                )
                self.update_status("● Movimiento XYZ completado")
            else:
                self.ik_status_label.setText("✗ No se encontró solución IK")
                QMessageBox.critical(
                    self, "Error", "No se pudo alcanzar la posición"
                )
        
        except ValueError:
            QMessageBox.warning(self, "Error", "Valores numéricos inválidos")
    
    def move_to_preset(self, x, y, z):
        """Mueve el robot a una posición XYZ predefinida."""
        self.x_entry.setText(f"{x:.3f}")
        self.y_entry.setText(f"{y:.3f}")
        self.z_entry.setText(f"{z:.3f}")
        self.move_to_target()
    
    def update_position(self, x, y, z):
        """Actualiza la visualización de posición actual."""
        self.current_x_label.setText(f"X: {x:.3f} m")
        self.current_y_label.setText(f"Y: {y:.3f} m")
        self.current_z_label.setText(f"Z: {z:.3f} m")
