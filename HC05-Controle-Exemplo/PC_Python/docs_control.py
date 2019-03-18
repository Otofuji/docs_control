import pyautogui
import serial
import argparse
import time
import logging
import docs_controlFunc as func



class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'alt','B': '='} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou nao 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
        self.bs = 0
        self.a = 1
    

    

    def update(self):

        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))

        if data == b'1' and self.bs == 0:
            func.listaNumerada()
        elif data == b'2' and self.bs == 0:
            func.bullet()
        elif data == b'3' and self.bs == 0:
            func.comentario()
        elif data == b'4' and self.bs == 0:
            func.notaRodape()
        elif data == b'5' and self.bs == 0:
            func.historicoDeRevisoes()
        elif data == b'6' and self.bs == 0:
            func.ditado()
        elif data == b'7' and self.bs == 0:
            func.equation()
        elif data == b'8' and self.bs == 0:
            func.gerarPDF()
        elif data == b'9' and self.bs == 0:
            func.enviarEmailParaColaboradores()
        elif data == b'a' and self.bs == 0:
            func.inserirImagem()
        elif data == b'b' and self.bs == 0:
            func.enviarPDFporEmail()
        elif data == b'c' and self.bs == 0:
            func.publicarNaWeb()

            self.bs = 200
            self.a+=1

            if(self.a>11):
                self.a = 0

        else:
            if self.bs > 0:
                self.bs = self.bs-1
#            logging.info("KEYUP A")
#            pyautogui.keyUp(self.mapping.button['A'])
#            pyautogui.keyUp(self.mapping.button['B'])


        self.incoming = self.ser.read()
    
   
class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
