import pyautogui
import time


def listaNumerada():
    pyautogui.hotkey('ctrl','shift','7')

def bullet():
    pyautogui.hotkey('ctrl','shift','8')

def comentario():
    pyautogui.hotkey('ctrl','alt','m')

def notaRodape():
    pyautogui.hotkey('ctrl','alt','f')

def historicoDeRevisoes():
    pyautogui.hotkey('ctrl','alt','shift','h')

def ditado():
    pyautogui.hotkey('ctrl','shift','s')

def equation():
    pyautogui.hotkey('alt','shift','i')
    time.sleep(0.1)
    for i in range(0,8):
        pyautogui.hotkey('down')
    pyautogui.hotkey('enter')

def gerarPDF():
    pyautogui.hotkey('alt','shift','f')
    time.sleep(0.1)
    for i in range(0,5):
        pyautogui.hotkey('down')
    pyautogui.hotkey('right')
    time.sleep(0.1)
    for i in range(0,3):
        pyautogui.hotkey('down')
    pyautogui.hotkey('enter')

def enviarEmailParaColaboradores():
    pyautogui.hotkey('alt','shift','f')
    time.sleep(0.1)
    for i in range(0,12):
        pyautogui.hotkey('down')
    pyautogui.hotkey('enter')

def inserirImagem():
    pyautogui.hotkey('alt','shift','i')
    time.sleep(0.1)
    pyautogui.hotkey('down')
    pyautogui.hotkey('right')
    time.sleep(0.1)

def enviarPDFporEmail():
    pyautogui.hotkey('alt','shift','f')
    time.sleep(0.1)
    for i in range(0,6):
        pyautogui.hotkey('down')
    pyautogui.hotkey('enter')

def publicarNaWeb():
    pyautogui.hotkey('alt','shift','f')
    time.sleep(0.1)
    for i in range(0,11):
        pyautogui.hotkey('down')
    pyautogui.hotkey('enter')