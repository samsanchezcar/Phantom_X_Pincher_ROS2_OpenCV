"""
Página de visualización (RViz y Toolbox).
"""
import os
import subprocess
import threading

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QFrame, QPushButton, QMessageBox
)
from PyQt5.QtCore import QTimer


class VisualizationPage(QWidget):
    """Página de visualización con RViz y Robotics Toolbox."""
    
    def __init__(self):
        super().__init__()
        
        self.rviz_process = None
        self.toolbox_process = None
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        title = QLabel("Visualización")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # Tarjeta RViz
        rviz_card = QFrame()
        rviz_card.setObjectName("card")
        rviz_layout = QVBoxLayout(rviz_card)
        
        rviz_title = QLabel("RViz - Modelo 3D")
        rviz_title.setObjectName("sectionTitle")
        rviz_layout.addWidget(rviz_title)
        
        self.rviz_btn = QPushButton("▶ LANZAR RViz")
        self.rviz_btn.setFixedHeight(60)
        self.rviz_btn.clicked.connect(self.launch_rviz)
        rviz_layout.addWidget(self.rviz_btn)
        
        self.stop_rviz_btn = QPushButton("⏹ DETENER RViz")
        self.stop_rviz_btn.setEnabled(False)
        self.stop_rviz_btn.setFixedHeight(60)
        self.stop_rviz_btn.clicked.connect(self.stop_rviz)
        rviz_layout.addWidget(self.stop_rviz_btn)
        
        self.rviz_status_label = QLabel("● RViz no iniciado")
        self.rviz_status_label.setStyleSheet(
            "color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;"
        )
        rviz_layout.addWidget(self.rviz_status_label)
        
        layout.addWidget(rviz_card)
        
        # Tarjeta Toolbox
        toolbox_card = QFrame()
        toolbox_card.setObjectName("card")
        toolbox_layout = QVBoxLayout(toolbox_card)
        
        toolbox_title = QLabel("Robotics Toolbox - Simulación")
        toolbox_title.setObjectName("sectionTitle")
        toolbox_layout.addWidget(toolbox_title)
        
        self.toolbox_btn = QPushButton("▶ LANZAR Toolbox")
        self.toolbox_btn.setFixedHeight(60)
        self.toolbox_btn.clicked.connect(self.launch_toolbox)
        toolbox_layout.addWidget(self.toolbox_btn)
        
        self.stop_toolbox_btn = QPushButton("⏹ DETENER Toolbox")
        self.stop_toolbox_btn.setEnabled(False)
        self.stop_toolbox_btn.setFixedHeight(60)
        self.stop_toolbox_btn.clicked.connect(self.stop_toolbox)
        toolbox_layout.addWidget(self.stop_toolbox_btn)
        
        self.toolbox_status_label = QLabel("● Toolbox no iniciado")
        self.toolbox_status_label.setStyleSheet(
            "color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;"
        )
        toolbox_layout.addWidget(self.toolbox_status_label)
        
        layout.addWidget(toolbox_card)
        layout.addStretch()
    
    def launch_rviz(self):
        """Lanza RViz en un proceso separado."""
        try:
            cmd = [
                "ros2", "launch", 
                "phantomx_pincher_description", "display.launch.py"
            ]
            
            def run_rviz():
                self.rviz_process = subprocess.Popen(cmd)
                self.rviz_process.wait()
                QTimer.singleShot(0, self.on_rviz_closed)
            
            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()
            
            self.rviz_btn.setEnabled(False)
            self.stop_rviz_btn.setEnabled(True)
            self.rviz_status_label.setText("● RViz ejecutándose")
            self.rviz_status_label.setStyleSheet(
                "color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;"
            )
        
        except Exception as e:
            QMessageBox.critical(
                self, "Error", f"No se pudo lanzar RViz: {str(e)}"
            )
    
    def stop_rviz(self):
        """Detiene el proceso de RViz."""
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        self.on_rviz_closed()
    
    def on_rviz_closed(self):
        """Callback cuando RViz se cierra."""
        self.rviz_btn.setEnabled(True)
        self.stop_rviz_btn.setEnabled(False)
        self.rviz_status_label.setText("● RViz no iniciado")
        self.rviz_status_label.setStyleSheet(
            "color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;"
        )
    
    def launch_toolbox(self):
        """Lanza el visualizador del Robotics Toolbox."""
        try:
            # Buscar toolbox.py en el directorio actual
            toolbox_path = os.path.join(os.path.dirname(__file__), '..', '..', 'toolbox.py')
            
            if not os.path.exists(toolbox_path):
                QMessageBox.warning(
                    self, "Error", 
                    f"No se encontró toolbox.py en:\n{toolbox_path}"
                )
                return
            
            def run_toolbox():
                self.toolbox_process = subprocess.Popen(['python3', toolbox_path])
                self.toolbox_process.wait()
                QTimer.singleShot(0, self.on_toolbox_closed)
            
            thread = threading.Thread(target=run_toolbox, daemon=True)
            thread.start()
            
            self.toolbox_btn.setEnabled(False)
            self.stop_toolbox_btn.setEnabled(True)
            self.toolbox_status_label.setText("● Toolbox ejecutándose")
            self.toolbox_status_label.setStyleSheet(
                "color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;"
            )
        
        except Exception as e:
            QMessageBox.critical(
                self, "Error", f"No se pudo lanzar Toolbox: {str(e)}"
            )
    
    def stop_toolbox(self):
        """Detiene el proceso del Toolbox."""
        if self.toolbox_process:
            try:
                self.toolbox_process.terminate()
                self.toolbox_process = None
            except:
                pass
        self.on_toolbox_closed()
    
    def on_toolbox_closed(self):
        """Callback cuando el Toolbox se cierra."""
        self.toolbox_btn.setEnabled(True)
        self.stop_toolbox_btn.setEnabled(False)
        self.toolbox_status_label.setText("● Toolbox no iniciado")
        self.toolbox_status_label.setStyleSheet(
            "color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;"
        )
    
    def cleanup(self):
        """Limpia los procesos al cerrar."""
        if self.rviz_process:
            self.stop_rviz()
        if self.toolbox_process:
            self.stop_toolbox()
