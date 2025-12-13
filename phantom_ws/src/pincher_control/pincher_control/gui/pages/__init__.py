"""
Páginas de la interfaz gráfica.
"""
from .dashboard import DashboardPage
from .manual_control import ManualControlPage
from .fixed_values import FixedValuesPage
from .fixed_angles import FixedAnglesPage
from .xyz_control import XYZControlPage
from .visualization import VisualizationPage
from .info import InfoPage

__all__ = [
    'DashboardPage',
    'ManualControlPage', 
    'FixedValuesPage',
    'FixedAnglesPage',
    'XYZControlPage',
    'VisualizationPage',
    'InfoPage'
]
