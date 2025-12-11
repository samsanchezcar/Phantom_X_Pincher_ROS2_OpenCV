"""
Ventana principal de la interfaz gráfica.
"""
import rclpy
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QFrame, QPushButton, QStackedWidget, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

from .widgets import PositionSignal
from .pages import (
    DashboardPage, ManualControlPage, FixedValuesPage,
    FixedAnglesPage, XYZControlPage, VisualizationPage, InfoPage
)


class ModernPincherGUI(QMainWindow):
    """Ventana principal de la aplicación."""
    
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        
        # Crear señal para actualización de posición
        self.position_signal = PositionSignal()
        self.position_signal.changed.connect(self._on_position_changed)
        self.controller.position_changed = self.position_signal.changed
        
        self.init_ui()
        
        # Timer para actualizar posición XYZ
        self.xyz_timer = QTimer()
        self.xyz_timer.timeout.connect(self._update_xyz_display)
        self.xyz_timer.start(200)
    
    def init_ui(self):
        self.setWindowTitle("PhantomX Pincher Control Studio")
        self.setGeometry(100, 50, 1400, 800)
        
        central = QWidget()
        central.setObjectName("centralWidget")
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # Sidebar
        sidebar = self._create_sidebar()
        main_layout.addWidget(sidebar)
        
        # Área de contenido
        content_container = QWidget()
        content_container.setObjectName("contentArea")
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(25, 25, 25, 25)
        
        self.stack = QStackedWidget()
        content_layout.addWidget(self.stack)
        
        self._create_pages()
        
        # Barra de estado
        status_frame = QFrame()
        status_layout = QHBoxLayout(status_frame)
        status_layout.addStretch()
        
        self.status_label = QLabel("● Sistema Listo")
        self.status_label.setObjectName("statusLabel")
        status_layout.addWidget(self.status_label)
        
        status_layout.addStretch()
        content_layout.addWidget(status_frame)
        
        main_layout.addWidget(content_container, 1)
    
    def _create_sidebar(self):
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(280)
        
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(0, 20, 0, 20)
        layout.setSpacing(5)
        
        # Título
        title = QLabel("PINCHER X100")
        title.setStyleSheet(
            "color: #00d9ff; font-size: 20pt; font-weight: bold; padding: 20px;"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        subtitle = QLabel("Control Studio")
        subtitle.setStyleSheet("color: #b0b0c0; font-size: 11pt; padding-bottom: 20px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        
        # Botón HOME
        home_btn = QPushButton("🏠 HOME (0°)")
        home_btn.setObjectName("homeButtonSidebar")
        home_btn.clicked.connect(self._home_all)
        layout.addWidget(home_btn)
        
        # Botón EMERGENCIA
        emergency_btn = QPushButton("🛑 EMERGENCIA")
        emergency_btn.setObjectName("emergencyButtonSidebar")
        emergency_btn.clicked.connect(self._emergency_stop)
        layout.addWidget(emergency_btn)
        
        # Separador
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #3a3a4e; margin: 10px;")
        layout.addWidget(separator)
        
        # Botones del menú
        self.menu_buttons = []
        
        menus = [
            ("📊 Panel Principal", 0),
            ("🎚️ Control Manual", 1),
            ("📝 Valores Fijos", 2),
            ("📐 Ángulos Predef.", 3),
            ("🎯 Control XYZ", 4),
            ("👁️ Visualización", 5),
            ("ℹ️ Información", 6),
        ]
        
        for text, index in menus:
            btn = QPushButton(text)
            btn.setObjectName("menuButton")
            btn.setCheckable(True)
            btn.setFixedHeight(55)
            btn.clicked.connect(lambda checked, i=index: self._change_page(i))
            layout.addWidget(btn)
            self.menu_buttons.append(btn)
        
        self.menu_buttons[0].setChecked(True)
        
        layout.addStretch()
        
        footer = QLabel("v2.0 • 2024")
        footer.setStyleSheet("color: #6a6a7e; font-size: 9pt; padding: 20px;")
        footer.setAlignment(Qt.AlignCenter)
        layout.addWidget(footer)
        
        return sidebar
    
    def _create_pages(self):
        # Dashboard
        self.dashboard_page = DashboardPage(
            self.controller, 
            self._on_speed_change
        )
        self.stack.addWidget(self.dashboard_page)
        
        # Control manual
        self.manual_page = ManualControlPage(
            self.controller,
            self._get_speed,
            self._update_status
        )
        self.stack.addWidget(self.manual_page)
        
        # Valores fijos
        self.fixed_values_page = FixedValuesPage(
            self.controller,
            self._get_speed,
            self._update_status
        )
        self.stack.addWidget(self.fixed_values_page)
        
        # Ángulos predefinidos
        self.fixed_angles_page = FixedAnglesPage(
            self.controller,
            self._update_status
        )
        self.stack.addWidget(self.fixed_angles_page)
        
        # Control XYZ
        self.xyz_page = XYZControlPage(
            self.controller,
            self._update_status
        )
        self.stack.addWidget(self.xyz_page)
        
        # Visualización
        self.visualization_page = VisualizationPage()
        self.stack.addWidget(self.visualization_page)
        
        # Información
        self.info_page = InfoPage()
        self.stack.addWidget(self.info_page)
    
    def _change_page(self, index):
        self.stack.setCurrentIndex(index)
        for i, btn in enumerate(self.menu_buttons):
            btn.setChecked(i == index)
    
    def _on_position_changed(self, motor_id, position):
        """Maneja cambios de posición externos."""
        self.manual_page.on_position_changed(motor_id, position)
    
    def _on_speed_change(self, value):
        """Maneja cambios en el slider de velocidad."""
        self.controller.update_speed(value)
    
    def _get_speed(self):
        """Obtiene la velocidad actual."""
        return self.dashboard_page.get_speed()
    
    def _update_status(self, message):
        """Actualiza el mensaje de estado."""
        self.status_label.setText(message)
        QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))
    
    def _update_xyz_display(self):
        """Actualiza la visualización de posición XYZ."""
        try:
            x, y, z = self.controller.get_current_xyz()
            self.dashboard_page.update_position(x, y, z)
            self.xyz_page.update_position(x, y, z)
        except:
            pass
    
    def _home_all(self):
        """Mueve todos los motores a home."""
        if self.controller.emergency_stop_activated:
            reply = QMessageBox.question(
                self, "Reactivar Sistema",
                "¿Reactivar sistema y mover a HOME (0°)?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.No:
                return
        
        self.controller.home_all_motors()
        self._update_status("● Todos los motores en HOME (0°)")
    
    def _emergency_stop(self):
        """Activa la parada de emergencia."""
        self.controller.emergency_stop()
        self.status_label.setText("⚠ PARADA DE EMERGENCIA ACTIVADA")
        self.status_label.setStyleSheet(
            "background-color: #ff416c; color: white; font-weight: bold; "
            "padding: 10px; border-radius: 8px;"
        )
    
    def closeEvent(self, event):
        """Maneja el cierre de la ventana."""
        # Limpiar procesos de visualización
        self.visualization_page.cleanup()
        
        reply = QMessageBox.question(
            self, "Salir",
            "¿Cerrar la aplicación?\nSe desactivará el torque.",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.controller.close()
            event.accept()
            rclpy.shutdown()
        else:
            event.ignore()
