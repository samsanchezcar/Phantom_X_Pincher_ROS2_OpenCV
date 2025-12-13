"""
Página de información del proyecto.
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QFrame, QScrollArea
)


class InfoPage(QWidget):
    """Página de información del proyecto y autores."""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        title = QLabel("Información del Proyecto")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # Autor 1
        card1 = QFrame()
        card1.setObjectName("card")
        layout1 = QVBoxLayout(card1)
        
        name1 = QLabel("Samuel David Sanchez Cardenas")
        name1.setObjectName("sectionTitle")
        layout1.addWidget(name1)
        
        info1 = QLabel(
            "📚 Ingeniería Mecatrónica\n"
            "☕ Café Lover\n"
            "🤖 Solo Robótica\n"
            "⏰ Veremos si acaba primero el semestre o el semestre acaba conmigo\n\n"
            "🔗 GitHub: samsanchezcar"
        )
        info1.setStyleSheet("color: #e0e0e0; line-height: 1.6; padding: 10px;")
        layout1.addWidget(info1)
        
        scroll_layout.addWidget(card1)
        
        # Autor 2
        card2 = QFrame()
        card2.setObjectName("card")
        layout2 = QVBoxLayout(card2)
        
        name2 = QLabel("Santiago Ávila Corredor")
        name2.setObjectName("sectionTitle")
        layout2.addWidget(name2)
        
        info2 = QLabel(
            "😵 El semestre me está matando\n"
            "🙏 Pronto todo acabará\n\n"
            "🔗 GitHub: Santiago-Avila"
        )
        info2.setStyleSheet("color: #e0e0e0; line-height: 1.6; padding: 10px;")
        layout2.addWidget(info2)
        
        scroll_layout.addWidget(card2)
        
        # Información del proyecto
        card3 = QFrame()
        card3.setObjectName("card")
        layout3 = QVBoxLayout(card3)
        
        proj_title = QLabel("Sobre el Proyecto")
        proj_title.setObjectName("sectionTitle")
        layout3.addWidget(proj_title)
        
        proj_info = QLabel(
            "🤖 PhantomX Pincher Control Studio\n"
            "📦 Versión: 2.0.0 (Modular)\n"
            "🐍 Python + ROS2 + PyQt5\n"
            "📐 Robotics Toolbox para cinemática\n\n"
            "Este proyecto proporciona una interfaz gráfica moderna\n"
            "para el control del robot PhantomX Pincher con:\n"
            "• Control manual por sliders\n"
            "• Movimientos a ángulos predefinidos\n"
            "• Control por cinemática inversa (XYZ)\n"
            "• Integración con RViz\n"
            "• Arquitectura modular"
        )
        proj_info.setStyleSheet("color: #e0e0e0; line-height: 1.6; padding: 10px;")
        layout3.addWidget(proj_info)
        
        scroll_layout.addWidget(card3)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
