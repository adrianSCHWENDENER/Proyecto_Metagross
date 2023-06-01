import sys
import serial

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton


class MenuInterfaz(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Crear los elementos de la interfaz
        self.btn_opcion1 = QPushButton('Modo Libre')
        self.btn_opcion2 = QPushButton('Modo Combate')
        self.btn_opcion3 = QPushButton('Modo Desplazamiento')

        # Crear el contenedor vertical
        vbox = QVBoxLayout()
        vbox.addWidget(self.btn_opcion1)
        vbox.addWidget(self.btn_opcion2)
        vbox.addWidget(self.btn_opcion3)

        # Configurar la ventana principal
        self.setLayout(vbox)
        self.setGeometry(100, 100, 200, 200)
        self.setWindowTitle('Menú')

    # Crear funcion para asignar acciones a los botones
    def connect_signals(self, custom_action1, custom_action2, custom_action3):
        self.btn_opcion1.clicked.connect(custom_action1)
        self.btn_opcion2.clicked.connect(custom_action2)
        self.btn_opcion3.clicked.connect(custom_action3)