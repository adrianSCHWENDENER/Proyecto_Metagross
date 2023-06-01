import sys
import time
import serial
from Adafruit_IO import Client, Feed, Data, RequestError
from Menu import MenuInterfaz
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

PORT = 'COM6'
BAUD_RATE = 9600
ser = serial.Serial(PORT, BAUD_RATE)

# Modo Libre ------------------------------------------------------------------------------------------------------
def custom_action1():
    print("Modo Libre activado")
    ser.write(str(1).encode())

# Modo Combate ----------------------------------------------------------------------------------------------------
def custom_action2():
    print("Modo Combate activado")
    ser.write(str(0).encode())
    
    # Valores de Adafruit
    ADAFRUIT_IO_KEY = 'aio_TRAq081QcT42rYmpm8UEuwGmtm0f'
    ADAFRUIT_IO_USERNAME = 'SCHadrian19'
    aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
    
    # Feeds
    Caja_de_combate = aio.feeds('caja-de-combate')
    HP = aio.feeds('hp')
    Tajo_umbrio = aio.feeds('tajo-umbrio')
    Lanzallamas = aio.feeds('lanzallamas')
    Golpe_karate = aio.feeds('golpe-karate')
    Cola_dragon = aio.feeds('cola-dragon')
    
    # Establecer vida de Metagross
    aio.send_data(Caja_de_combate.key, "¡Un METAGROSS salvaje ha aparecido!")
    aio.send_data(HP.key, 364)
       
    def loop_Modo_Combate():
        DamageTRUE = 0
        Badera = 0
        # Recibir datos de ADAFRUIT
        Dark = int(aio.receive(Tajo_umbrio.key).value)
        Fire = int(aio.receive(Lanzallamas.key).value)
        Fighting = int(aio.receive(Golpe_karate.key).value)
        Dragon = int(aio.receive(Cola_dragon.key).value)
        
        # Se usa 364 (vida max.) en lugar del receive(HP.key).value ya que no se implemento un combate continuo, solo un ataque
        
        if Dark == 1:
            Bandera = 2
            Damage = 364 - (70 * 2)
            aio.send_data(Caja_de_combate.key, "¡Es súper efectivo!")
            DamageTRUE = 1
        elif Fire == 1:
            Bandera = 2
            Damage = 364 - (90 * 2)
            aio.send_data(Caja_de_combate.key, "¡Es súper efectivo!")
            DamageTRUE = 1
        elif Fighting == 1:
            Bandera = 3
            Damage = 364 - (50 * 1)
            DamageTRUE = 1
        elif Dragon == 1:
            Bandera = 0
            Damage = 364 - (60 * (1/2))
            aio.send_data(Caja_de_combate.key, "No es muy efectivo...")
            DamageTRUE = 1

        if DamageTRUE == 1:
            aio.send_data(HP.key, Damage)
            ser.write(str(Bandera).encode())
            timer.stop()
    
    # Crear un temporizador para controlar las acciones repetitivas
    
    timer = QTimer()
    timer.timeout.connect(loop_Modo_Combate)
    timer.start(3000)

# Modo Desplazamiento ---------------------------------------------------------------------------------------------
def custom_action3():
    print("No implementado")


# Llamar y mostrar menú
if __name__ == "__main__":
    app = QApplication(sys.argv)
    menu = MenuInterfaz()
    
    # Cambiar la accion de los botones
    menu.connect_signals(custom_action1, custom_action2, custom_action3)

    menu.show()
    sys.exit(app.exec())
