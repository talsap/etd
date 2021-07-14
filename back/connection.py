# -*- coding: utf-8 -*-

'''Bibliotecas'''

import time
import serial
import bancodedados
import numpy as np
from sys import *
from serial.tools import list_ports

'''Variaveis Globais'''
opcaoC = "C"    '''conectado'''
opcaoD = "D"    '''desconectado'''
opcaoI = "I"    '''DNIT134'''
opcaoE = "E"    '''CAMARA DE PRESSAO'''
opcaoM = "M"    '''MOTOR DE PASSOS'''
opcaoB = "B"    '''Break'''
opcaoG = "G"    '''Golpes'''
Y = []          #Array Deformações
T = []          #Array tempo grafico'''

'''Port Serial'''
portlist = [port for port,desc,hwin in list_ports.comports()]
conexao = serial.Serial()
conexao.baudrate = 115200

'''Coeficientes da calibracao'''
L = bancodedados.LVDT()
A1 = float(L[0])
B1 = float(L[1])
A2 = float(L[2])
B2 = float(L[3])

#-------------------------------------------------------------------
def connect():
    i = 0
    condicaoConeccao = False
    try:
        while i < len(portlist):
            conexao.port = portlist[i]
            try:
                conexao.open()
                if conexao.isOpen() == True:
                    print("Verificando Conexao com porta serial "+conexao.port+"...\n")
                    conexao.write(opcaoC)
                    conexao.timeout = 1
                    a = conexao.readline()
                    if a[0] == "c":
                        print a
                        condicaoConeccao = True
                        return conexao.port, "connectado"
                    else:
                        print "notconectado"
                        conexao.close()
            except:
                conexao.close()
                time.sleep(.2)
                print("Nao foi possivel manter a conexao com "+conexao.port+"! Verifique a conexao usb.\n")
                if i == len(portlist):
                    return conexao.port, "notconnectado"
                else:
                    i = i+1
    except:
        time.sleep(.2)
        print("Nao foi possivel manter a conexao! Verifique a conexao usb.\n")
        condicaoConeccao = False
        return "0", "notconnectado"

    else:
        if condicaoConeccao == True:
            pass
        else:
            time.sleep(.2)
            print("Nao foi possivel manter a conexao! Verifique a conexao usb.\n")
            condicaoConeccao = False
            return "0", "notconnectado"

#-------------------------------------------------------------------
'''Fim'''
def modeF():
    conexao.write(str(-3))  #O valor responsável em parar o ensaio é b

#-------------------------------------------------------------------
'''Continua'''
def modeC():
    conexao.write(str(-1))  #O valor responsável em continuar o ensaio é c

#-------------------------------------------------------------------
'''Pausa'''
def modeP():
    conexao.write(str(-4))  #O valor responsável em pausar o ensaio é p

#-------------------------------------------------------------------
'''Desconectando'''
def modeD():
    conexao.write(opcaoD)

#-------------------------------------------------------------------
'''Ativando camara'''
def modeE():
    conexao.write(opcaoE)

#-------------------------------------------------------------------
'''Conecxao com a DNIT134'''
def modeI():
    conexao.write(opcaoC)
    conexao.write(opcaoI)
    while (conexao.inWaiting() == 0):
        pass
    L = conexao.readline()
    while (conexao.inWaiting() == 0):
        pass
    L = conexao.readline()
    print L

#-------------------------------------------------------------------
'''Aplica os Golpes'''
def modeG(qtd, freq):
    print "OPC G"
    conexao.write(opcaoG)
    time.sleep(.5)
    conexao.write(str(int(round(qtd,0))))
    time.sleep(1)
    conexao.write(str(int(round(freq,0))))

#-------------------------------------------------------------------
'''Camara de ar'''
def modeCAM(p2):
    incremental = p2/5
    i = 1
    time.sleep(1)
    while i <= 6:
        conexao.write(str(int(round(incremental*i,0))))
        print str(int(round(incremental*i,0)))
        time.sleep(1)
        i += 1
        if i == 6:
            conexao.write(str(-1))
            return "p2ok"
            break

#-------------------------------------------------------------------
'''Ativando motor'''
def modeM():
    conexao.write(opcaoM)

#-------------------------------------------------------------------
'''Ativacao do motor de passos'''
def modeMotor(p1):
    conexao.write(str(int(round(p1,0))))
    while (conexao.inWaiting() == 0):
        pass
    contadorOK = 0
    time.sleep(.5)
    while True:
        while (conexao.inWaiting() == 0):
            pass
        a = conexao.readline()
        #print a
        try:
            if a[0] == "o":
                contadorOK += 1
                if contadorOK == 25: #contadorOK igual a 25
                    conexao.write(str(-1))
                    return "p1ok"
                    break
            else: #else apenas para testes
                time.sleep(5)
                conexao.write(str(-1))
                return "p1ok"
                break
        except:
            pass

#-------------------------------------------------------------------
def ColetaI():
    conexao.flushInput()
    #conexao.flushOutput()
    conexao.write(str(0))
    while (conexao.inWaiting() == 0):
        pass
    arduinoString = conexao.readline()
    Array = arduinoString.split(',')

    y1mm = float(Array[0])*A1+B1
    y2mm = float(Array[1])*A2+B2
    y1v = float(Array[2])
    y2v = float(Array[3])
    sen = float(Array[4])
    cam = float(Array[5])
    glp = float(Array[6])
    sts = float(Array[7])
    defE = float(Array[8])*A1+B1
    defP = float(Array[9])*A1+B1
    defAc = float(Array[10])*A1+B1
    defMax = float(Array[11])*A1+B1

    return y1mm, y2mm, y1v, y2v, sen/10000, cam/10000, glp, sts, defE, defP, defAc, defMax
