# -*- coding: utf-8 -*-

'''Bibliotecas'''
import wx
import time
import threading
import matplotlib
import numpy as np
import banco.bancodedados as bancodedados
import banco.bdConfiguration as bdConfiguration
import back.connection as con
import matplotlib.pyplot as plt
import back.MyProgressDialog as My
import back.SaveThread as SaveThread
import back.MotorThread as MotorThread
import back.DinamicaThread as DinamicaThread
import back.ConexaoThread as ConexaoThread
import back.HexForRGB as HexRGB
import banco.bdPreferences as bdPreferences
from drawnow import *
from front.quadrotensoes import quadro
from front.dialogoDinamico import dialogoDinamico
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas


'''Frequencias para o ensaio'''
frequencias = ['1', '2']

########################################################################
'''Painel Superior'''
class TopPanel(wx.Panel):
        def __init__(self, parent, _self):
            wx.Panel.__init__(self, parent = parent)
            global colorLineGrafic
            
            colors = bdPreferences.ListColors()
            colorCard = colors[0]
            colorTextCtrl = colors[1]
            colorBackground = colors[2]
            colorLineGrafic = colors[3]
            colorBackgroundGrafic = colors[4]

            self._self = _self
            self.SetBackgroundColour(colorBackground) 

            FontTitle = wx.Font(-1, wx.SWISS, wx.NORMAL, wx.BOLD)

            self.sizer = wx.BoxSizer(wx.VERTICAL)
            self.v_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.figure = plt.figure(constrained_layout=True)
            self.axes = self.figure.add_subplot(111)
            self.canvas = FigureCanvas(self, -1, self.figure)

            rect = self.figure.patch
            rect.set_facecolor(colorBackgroundGrafic)

            self.pausa = wx.Button(self, -1, 'PAUSA')
            self.Bind(wx.EVT_BUTTON, self.PAUSA, self.pausa)
            self.continua = wx.Button(self, -1, 'CONTINUA')
            self.Bind(wx.EVT_BUTTON, self.CONTINUA, self.continua)
            self.fim_inicio = wx.Button(self, -1, 'INICIO')
            self.Bind(wx.EVT_BUTTON, self.INICIO, self.fim_inicio)

            self.pausa.Disable()
            self.continua.Disable()
            self.fim_inicio.Disable()

            self.pausa.SetFont(FontTitle)
            self.continua.SetFont(FontTitle)
            self.fim_inicio.SetFont(FontTitle)

            self.v_sizer.Add(self.pausa, 1, wx.EXPAND | wx.ALL, 5)
            self.v_sizer.Add(self.continua, 1, wx.EXPAND | wx.ALL, 5)
            self.v_sizer.Add(self.fim_inicio, 1, wx.EXPAND | wx.ALL, 5)

            self.h_sizer.Add(self.canvas, 12, wx.EXPAND | wx.ALL, 5)
            self.h_sizer.Add(self.v_sizer, 1, wx.EXPAND | wx.ALL)
            self.h_sizer.AddStretchSpacer(4)

            self.sizer.Add(self.h_sizer, 0, wx.EXPAND | wx.ALL, 10)
            self.SetSizer(self.sizer)
            self.DINAMICA2_ANTERIOR = 0
            self.DINAMICA1_ANTERIOR = 0

    #--------------------------------------------------
        '''Função PAUSA'''
        def PAUSA(self, event):
            print '\nTopPanel - PAUSA'
            global conditionEnsaio
            global Pausa

            Pausa = True
            con.modeP()
            conditionEnsaio = False
            self._self.bottom.timer.Stop()
            self.continua.Enable()
            self.fim_inicio.Enable()
            self.pausa.Disable()

    #--------------------------------------------------
        '''Função CONTINUA'''
        def CONTINUA(self, event):
            print '\nTopPanel - CONTINUA'
            global conditionEnsaio
            global Ti
            global Pausa

            con.modeC()
            conditionEnsaio = True
            Pausa = False
            self._self.bottom.timer.Start(int('2500'))
            self.pausa.Enable()
            self.fim_inicio.Disable()
            self.continua.Disable()

    #--------------------------------------------------
        '''Função INICIO'''
        def INICIO(self, event):
            print '\nTopPanel - INICIO'
            global Fase
            global condition
            global conditionEnsaio
            global freq
            global Diam
            self.fim_inicio.Disable()

            if Fase == 'CONDICIONAMENTO':
                condition = False
                threadConection = DinamicaThread.DinamicaThreadOne(VETOR_COND[0][1], self.DINAMICA2_ANTERIOR, Diam)
                dlgC1 = My.MyProgressDialog(3)
                dlgC1.ShowModal()
                time.sleep(1)
                threadConection = DinamicaThread.DinamicaThreadTwo(VETOR_COND[0][0], self.DINAMICA1_ANTERIOR)
                dlgC2 = My.MyProgressDialog(3)
                dlgC2.ShowModal()
                time.sleep(1)
                self.DINAMICA2_ANTERIOR = VETOR_COND[0][1]
                self.DINAMICA1_ANTERIOR = VETOR_COND[0][0]

            if Fase == 'DP':
                condition = False
                threadConection = DinamicaThread.DinamicaThreadOne(VETOR_DP[0][1], self.DINAMICA2_ANTERIOR, Diam)
                dlgC1 = My.MyProgressDialog(3)
                dlgC1.ShowModal()
                time.sleep(1)
                threadConection = DinamicaThread.DinamicaThreadTwo(VETOR_DP[0][0], self.DINAMICA1_ANTERIOR)
                dlgC2 = My.MyProgressDialog(3)
                dlgC2.ShowModal()
                time.sleep(1)
                self.DINAMICA2_ANTERIOR = VETOR_DP[0][1]
                self.DINAMICA1_ANTERIOR = VETOR_DP[0][0]

            condition = False
            con.modeStoped()
            gl = self._self.bottom.NGolpes.GetValue()
            freq = self._self.bottom.freq.GetValue()
            con.modeG()
            time.sleep(0.5)
            con.modeGOLPES(int(gl)+1, int(freq))
            condition = True
            conditionEnsaio = True
            time.sleep(0.5)
            self._self.bottom.timer.Start(int('2500'))
            self.pausa.Enable()
            self.fim_inicio.SetLabel('FIM')
            self.Bind(wx.EVT_BUTTON, self.FIM, self.fim_inicio)

            #--------------------------------------------------
            #-------- Thread de parada e de salvamento --------
            def worker1(self):
                import banco.bancodedados as bancodedados
                global conddd
                global condition
                global conditionEnsaio
                global Fase
                global X
                global Y
                global pc
                global pg
                global mult
                global glpCOND
                global ntglp
                global DefResiliente
                global DefPermanente
                global REFERENCIA_MEDIA
                global idt
                global temposDNIT179_01
                global temposDNIT179_02
                valorGolpeAnterior = 0
                golpe = []
                vDR = []
                vDP = []
                ppc = []
                ppg = []

                if Fase == 'CONDICIONAMENTO':
                    while conddd:
                        try:
                            valorGolpe = int(self._self.bottom.GolpeAtual.GetValue())
                            if valorGolpe == int(glpCOND):
                                time.sleep(4)
                                con.modeI()
                                self.pausa.Disable()
                                conditionEnsaio = False
                                valorGolpe = 0
                                self._self.bottom.timer.Stop()
                                X = np.array([])
                                Y = np.array([])
                                mult = 0
                                self.draww()
                                self.pausa.Disable()
                                evt = wx.PyCommandEvent(wx.EVT_BUTTON.typeId, self._self.bottom.dp.GetId())
                                wx.PostEvent(self._self.bottom.condic, evt)
                                break
                        except:
                            pass

                if Fase == 'DP':
                    while conddd:
                        try:
                            valorGolpe = int(self._self.bottom.GolpeAtual.GetValue())
                            if valorGolpe != valorGolpeAnterior:
                                valorGolpeAnterior = valorGolpe
                                if valorGolpe in temposDNIT179_02:
                                    bancodedados.saveDNIT179(idt, valorGolpe, DefResiliente, DefPermanente, pc, pg)
                                if valorGolpe in temposDNIT179_01:
                                    golpe.append(valorGolpe)
                                    vDR.append(DefResiliente)
                                    vDP.append(DefPermanente)
                                    ppc.append(pc)
                                    ppg.append(pg)
                                    if valorGolpe == 200:
                                        self._self.bottom.gDP.Enable()
                                    if valorGolpe == 100:
                                        ix = 0
                                        while ix < len(golpe):
                                            bancodedados.saveDNIT179(idt, golpe[ix], vDR[ix], vDP[ix], ppc[ix], ppg[ix])
                                            ix += 1
                                        golpe *= 0 #limpa a lista
                                        vDR *= 0 #limpa a lista
                                        vDP *= 0 #limpa a lista
                                        ppc *= 0 #limpa a lista
                                        ppg *= 0 #limpa a lista
                                    if valorGolpe == 900:
                                        ix = 0
                                        while ix < len(golpe):
                                            bancodedados.saveDNIT179(idt, golpe[ix], vDR[ix], vDP[ix], ppc[ix], ppg[ix])
                                            ix += 1
                                        golpe *= 0 #limpa a lista
                                        vDR *= 0 #limpa a lista
                                        vDP *= 0 #limpa a lista
                                        ppc *= 0 #limpa a lista
                                        ppg *= 0 #limpa a lista

                            if valorGolpe == int(ntglp):
                                time.sleep(4)
                                con.modeI()
                                self.pausa.Disable()
                                self._fase = self._self.bottom._fase + 1
                                self._self.bottom._fase = self._fase
                                valorGolpe = 0
                                conditionEnsaio = False
                                self._self.bottom.timer.Stop()
                                X = np.array([])
                                Y = np.array([])
                                mult = 0
                                self.draww()
                                self.pausa.Disable()
                                evt = wx.PyCommandEvent(wx.EVT_BUTTON.typeId, self._self.bottom.dp.GetId())
                                wx.PostEvent(self._self.bottom.dp, evt)
                                break
                        except:
                            pass
                print 'Thread-worker1-finalizada\n'
            #--------------------------------------------------
            self.t1 = threading.Thread(target=worker1, args=(self,))
            self.t1.start()

    #--------------------------------------------------
        '''Função FIM'''
        def FIM(self, event):
            print '\nTopPanel - FIM'
            global conddd
            global condition
            global conditionEnsaio
            global Fase
            global mult

            '''Diálogo se deseja realmente finalizar o CONDICIONAMENTO'''
            dlg = wx.MessageDialog(None, 'Deseja realmente finalizar o '+Fase+'?', 'EDP', wx.YES_NO | wx.CENTRE| wx.NO_DEFAULT )
            result = dlg.ShowModal()

            if result == wx.ID_YES:
                dlg.Destroy()
                self.fim_inicio.Disable()
                con.modeFIM()
                self.continua.Disable()

                conditionEnsaio = False
                self._self.bottom.timer.Stop()
                X = np.array([])
                Y = np.array([])
                mult = 0
                self.draww()

                if Fase == 'CONDICIONAMENTO':
                    self._self.bottom._fase = 0
                    self._self.bottom.dp.Enable()
                    self.fim_inicio.SetLabel('INICIO')
                    self.Bind(wx.EVT_BUTTON, self.INICIO, self.fim_inicio)
                    self._self.bottom.pressao_zero(VETOR_COND[0][0], VETOR_COND[0][1])
                    con.modeI()

                if Fase == 'DP':
                    con.modeI()
                    self._self.bottom.pressao_zero(VETOR_DP[0][0], VETOR_DP[0][1])
                    con.modeI()
                    bancodedados.data_final_Update_idt(idt)
                    dlg3 = dialogoDinamico(3, "EDP 179/2018ME", "O ENSAIO FOI FINALIZADO!", "Os relatórios de extração são gerados na tela inicial.", "FIM!", "", None)
                    if dlg3.ShowModal() == wx.ID_OK:
                        time.sleep(.3)
                        con.modeStoped()
                        time.sleep(.3)
                        con.modeB()
                        condition = False
                        conddd = False
                        time.sleep(.3)
                        con.modeD()
                        
    #--------------------------------------------------
        '''Ajusta min e max EIXO X'''
        def changeAxesX(self, min, max):
            print '\nTopPanel - changeAxesY'
            self.axes.set_xlim(float(min), float(max))
            self.canvas.draw()

    #--------------------------------------------------
        '''Ajusta min e max EIXO Y'''
        def changeAxesY(self, min, max):
            print '\nTopPanel - changeAxesY'
            self.axes.set_ylim(float(min), float(max))
            self.canvas.draw()

    #--------------------------------------------------
        def draww(self):
            print '\nTopPanel - draww'
            self.axes.clear()
            self.axes.set_xlim(float(0), float(1))
            self.axes.set_ylim(float(0), float(0.01))
            self.axes.set_xlabel("TEMPO (seg)")
            self.axes.set_ylabel("DESLOCAMENTO (mm)")
            self.canvas.draw()

    #--------------------------------------------------
        def draw(self):
            print '\nTopPanel - draw'
            global mult
            global colorLineGrafic
            self.axes.clear()
            #self.axes.set_xlim(mult*5-5, mult*5)
            self.axes.set_xlabel("TEMPO (seg)")
            self.axes.set_ylabel("DESLOCAMENTO (mm)")
            self.axes.plot(X, Y, colorLineGrafic)
            self.canvas.draw()

'''Painel Inferior'''
class BottomPanel(wx.Panel):
        def __init__(self, parent, top):
            wx.Panel.__init__(self, parent = parent)
            
            colors = bdPreferences.ListColors()
            colorCard = colors[0]
            colorTextCtrl = colors[1]
            colorBackground = colors[2]
            colorLineGrafic = colors[3]
            colorBackgroundGrafic = colors[4]

            colorStaticBox = HexRGB.RGB(colorCard)
            colorTextBackground = HexRGB.RGB(colorCard)
            colorTextCtrl = HexRGB.RGB(colorTextCtrl)

            self.graph = top

            self.SetBackgroundColour(colorBackground) 

            FontTitle = wx.Font(12, wx.SWISS, wx.NORMAL, wx.BOLD)
            FontTitle1 = wx.Font(-1, wx.SWISS, wx.NORMAL, wx.BOLD)
            Fonttext = wx.Font(12, wx.SWISS, wx.NORMAL, wx.NORMAL)

            staticbox1 = wx.StaticBox(self, -1, '')
            staticbox2 = wx.StaticBox(self, -1, '')
            staticbox3 = wx.StaticBox(self, -1, '')
            staticbox4 = wx.StaticBox(self, -1, '')
            staticbox5 = wx.StaticBox(self, -1, '')
            staticbox6 = wx.StaticBox(self, -1, '')

            staticboxSizer1 = wx.StaticBoxSizer(staticbox1, wx.VERTICAL)
            staticboxSizer2 = wx.StaticBoxSizer(staticbox2, wx.VERTICAL)
            staticboxSizer3 = wx.StaticBoxSizer(staticbox3, wx.VERTICAL)
            staticboxSizer4 = wx.StaticBoxSizer(staticbox4, wx.VERTICAL)
            staticboxSizer5 = wx.StaticBoxSizer(staticbox5, wx.VERTICAL)
            staticboxSizer6 = wx.StaticBoxSizer(staticbox6, wx.VERTICAL)

            staticbox1.SetBackgroundColour(colorStaticBox)
            staticbox2.SetBackgroundColour(colorStaticBox)
            staticbox3.SetBackgroundColour(colorStaticBox)
            staticbox4.SetBackgroundColour(colorStaticBox)
            staticbox5.SetBackgroundColour(colorStaticBox)
            staticbox6.SetBackgroundColour(colorStaticBox)

            self.gDP = wx.Button(self, -1, 'GOLPE\nX\nD. P.')
            self.Bind(wx.EVT_BUTTON, self.GDP, self.gDP)
            self.condic = wx.Button(self, -1, 'CONDIC.')
            self.Bind(wx.EVT_BUTTON, self.CONDIC, self.condic)
            self.dp = wx.Button(self, -1, 'D. P.')
            self.Bind(wx.EVT_BUTTON, self.DP, self.dp)
            self.LTeste = wx.Button(self, -1, "CONECTAR", size = wx.DefaultSize)
            self.Bind(wx.EVT_BUTTON, self.LTESTE, self.LTeste)
            self.LZero = wx.Button(self, -1, "L. ZERO", size = wx.DefaultSize)
            self.Bind(wx.EVT_BUTTON, self.LZERO, self.LZero)

            self.gDP.Disable()         
            self.condic.Disable()
            self.dp.Disable()
            self.LZero.Disable()

            self.gDP.SetFont(FontTitle1)
            self.condic.SetFont(FontTitle1)
            self.dp.SetFont(FontTitle1)
            self.LTeste.SetFont(FontTitle1)
            self.LZero.SetFont(FontTitle1)

            texto1 = wx.StaticText(self, label = "EIXO Y", style = wx.ALIGN_CENTRE)
            texto3 = wx.StaticText(self, label = "σ3 - Tensão confinante (MPa)", style = wx.ALIGN_CENTRE)
            texto4 = wx.StaticText(self, label = "σd - Tensão desvio (MPa)", style = wx.ALIGN_CENTRE)
            texto5 = wx.StaticText(self, label = "Y1 (V)", style = wx.ALIGN_CENTER)
            texto6 = wx.StaticText(self, label = "Y2 (V)", style = wx.ALIGN_CENTER)
            texto7 = wx.StaticText(self, label = "Y1 (mm)", style = wx.ALIGN_CENTER)
            texto8 = wx.StaticText(self, label = "Y2 (mm)", style = wx.ALIGN_CENTER)
            texto13 = wx.StaticText(self, label = "Altura Final (mm)", style = wx.ALIGN_LEFT)
            texto14 = wx.StaticText(self, label = "REAL", style = wx.ALIGN_CENTER)
            texto15 = wx.StaticText(self, label = "REAL", style = wx.ALIGN_CENTER)
            texto16 = wx.StaticText(self, label = "ALVO", style = wx.ALIGN_CENTER)
            texto17 = wx.StaticText(self, label = "ALVO", style = wx.ALIGN_CENTER)
            texto18 = wx.StaticText(self, label = "Altura (mm)", style = wx.ALIGN_LEFT)
            texto19 = wx.StaticText(self, label = "Diâmetro (mm)", style = wx.ALIGN_LEFT)
            texto21 = wx.StaticText(self, label = "FASE", style = wx.ALIGN_CENTER)
            texto22 = wx.StaticText(self, label = "Nº de CICLOs", style = wx.ALIGN_CENTER)
            texto23 = wx.StaticText(self, label = "Freq. (Hz)", style = wx.ALIGN_CENTER)
            texto24 = wx.StaticText(self, label = "CICLO Atual", style = wx.ALIGN_CENTER)

            texto1.SetFont(FontTitle)
            texto3.SetFont(FontTitle)
            texto4.SetFont(FontTitle)
            texto5.SetFont(Fonttext)
            texto6.SetFont(Fonttext)
            texto7.SetFont(Fonttext)
            texto8.SetFont(Fonttext)
            texto13.SetFont(FontTitle)
            texto14.SetFont(Fonttext)
            texto15.SetFont(Fonttext)
            texto16.SetFont(Fonttext)
            texto17.SetFont(Fonttext)
            texto18.SetFont(Fonttext)
            texto19.SetFont(Fonttext)
            texto21.SetFont(FontTitle)
            texto22.SetFont(Fonttext)
            texto23.SetFont(Fonttext)
            texto24.SetFont(Fonttext)

            texto1.SetBackgroundColour(colorTextBackground)
            texto3.SetBackgroundColour(colorTextBackground)
            texto4.SetBackgroundColour(colorTextBackground)
            texto5.SetBackgroundColour(colorTextBackground)
            texto6.SetBackgroundColour(colorTextBackground)
            texto7.SetBackgroundColour(colorTextBackground)
            texto8.SetBackgroundColour(colorTextBackground)
            texto13.SetBackgroundColour(colorTextBackground)
            texto14.SetBackgroundColour(colorTextBackground)
            texto15.SetBackgroundColour(colorTextBackground)
            texto16.SetBackgroundColour(colorTextBackground)
            texto17.SetBackgroundColour(colorTextBackground)
            texto18.SetBackgroundColour(colorTextBackground)
            texto19.SetBackgroundColour(colorTextBackground)
            texto21.SetBackgroundColour(colorTextBackground)
            texto22.SetBackgroundColour(colorTextBackground)
            texto23.SetBackgroundColour(colorTextBackground)
            texto24.SetBackgroundColour(colorTextBackground)

            self.y1V = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.y2V = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.y1mm = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.y2mm = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.AlturaFinal = wx.TextCtrl(self, -1, wx.EmptyString, size = (50, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.PCreal = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.PCalvo = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.SigmaReal = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.SigmaAlvo = wx.TextCtrl(self, -1, wx.EmptyString, size = (100, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.AlturaMM = wx.TextCtrl(self, -1, str(H), size = (80, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.DiametroMM = wx.TextCtrl(self, -1, str(Diam), size = (80, 41), style = wx.TE_READONLY | wx.TE_CENTER)
            self.fase = wx.TextCtrl(self, -1, '1', size = (50, 35), style = wx.TE_READONLY | wx.TE_CENTER)
            self.NGolpes = wx.TextCtrl(self, -1, wx.EmptyString, size = (70, 35), style = wx.TE_READONLY | wx.TE_CENTER)
            self.GolpeAtual = wx.TextCtrl(self, -1, wx.EmptyString, size = (70, 35), style = wx.TE_READONLY | wx.TE_CENTRE)
            self.freq = wx.ComboBox(self, -1, frequencias[0], choices = frequencias, size = (50, 35), style = wx.CB_READONLY)
            self.ensaioAuto = wx.CheckBox(self, -1, 'Ensaio automático', (20,0), (260,-1), style = wx.ALIGN_LEFT)

            self.y1V.Disable()
            self.y2V.Disable()
            self.y1mm.Disable()
            self.y2mm.Disable()
            self.AlturaFinal.Disable()
            self.PCreal.Disable()
            self.PCalvo.Disable()
            self.SigmaReal.Disable()
            self.SigmaAlvo.Disable()
            self.AlturaMM.Disable()
            self.DiametroMM.Disable()
            self.fase.Disable()
            self.NGolpes.Disable()
            self.GolpeAtual.Disable()
            self.freq.Disable()

            self.y1V.SetFont(Fonttext)
            self.y2V.SetFont(Fonttext)
            self.y1mm.SetFont(Fonttext)
            self.y2mm.SetFont(Fonttext)
            self.AlturaFinal.SetFont(Fonttext)
            self.PCreal.SetFont(Fonttext)
            self.PCalvo.SetFont(Fonttext)
            self.SigmaReal.SetFont(Fonttext)
            self.SigmaAlvo.SetFont(Fonttext)
            self.AlturaMM.SetFont(Fonttext)
            self.DiametroMM.SetFont(Fonttext)
            self.fase.SetFont(Fonttext)
            self.NGolpes.SetFont(Fonttext)
            self.GolpeAtual.SetFont(Fonttext)
            self.freq.SetFont(Fonttext)

            self.y1V.SetForegroundColour(colorTextCtrl)
            self.y2V.SetForegroundColour(colorTextCtrl)
            self.y1mm.SetForegroundColour(colorTextCtrl)
            self.y2mm.SetForegroundColour(colorTextCtrl)
            self.AlturaFinal.SetForegroundColour(colorTextCtrl)
            self.PCreal.SetForegroundColour(colorTextCtrl)
            self.PCalvo.SetForegroundColour(colorTextCtrl)
            self.SigmaReal.SetForegroundColour(colorTextCtrl)
            self.SigmaAlvo.SetForegroundColour(colorTextCtrl)
            self.AlturaMM.SetForegroundColour(colorTextCtrl)
            self.DiametroMM.SetForegroundColour(colorTextCtrl)
            self.fase.SetForegroundColour(colorTextCtrl)
            self.NGolpes.SetForegroundColour(colorTextCtrl)
            self.GolpeAtual.SetForegroundColour(colorTextCtrl)

            #--------------------------------------------------
            '''Static Box 1'''
            self.v16_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v17_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v18_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v19_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v20_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v21_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h19_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h20_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h21_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h22_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.v16_sizer.Add(texto8, 1, wx.CENTER)
            self.v16_sizer.Add(self.y2mm, 2, wx.CENTER)

            self.v17_sizer.Add(texto6, 1, wx.CENTER)
            self.v17_sizer.Add(self.y2V, 2, wx.CENTER)

            self.v18_sizer.Add(texto7, 1, wx.CENTER)
            self.v18_sizer.Add(self.y1mm, 2, wx.CENTER)

            self.v19_sizer.Add(texto5, 1, wx.CENTER)
            self.v19_sizer.Add(self.y1V, 2, wx.CENTER)

            self.h19_sizer.Add(self.v17_sizer, 5, wx.ALL | wx.CENTER)
            self.h19_sizer.AddStretchSpacer(1)
            self.h19_sizer.Add(self.v16_sizer, 5, wx.ALL | wx.CENTER)

            self.h20_sizer.Add(self.v19_sizer, 5, wx.ALL | wx.CENTER)
            self.h20_sizer.AddStretchSpacer(1)
            self.h20_sizer.Add(self.v18_sizer, 5, wx.ALL | wx.CENTER)

            self.h21_sizer.Add(self.LTeste, 5, wx.EXPAND)
            self.h21_sizer.AddStretchSpacer(2)
            self.h21_sizer.Add(self.LZero, 5, wx.EXPAND)

            self.v20_sizer.Add(self.h20_sizer, 3, wx.CENTER)
            self.v20_sizer.AddStretchSpacer(1)
            self.v20_sizer.Add(self.h19_sizer, 3, wx.CENTER)
            self.v20_sizer.AddStretchSpacer(2)
            self.v20_sizer.Add(self.h21_sizer, 2, wx.CENTER)

            self.v21_sizer.Add(texto1, 1, wx.CENTER)
            self.v21_sizer.Add(self.v20_sizer, 7, wx.CENTER)

            self.h22_sizer.Add(self.v21_sizer, 1, wx.CENTER)
            staticboxSizer1.Add(self.h22_sizer, 0,  wx.ALL | wx.EXPAND  | wx.CENTER, 10)

            #--------------------------------------------------
            '''Static Box 3'''
            self.v12_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v13_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v14_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h11_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h12_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.v12_sizer.Add(texto16, 1, wx.ALL | wx.CENTER)
            self.v12_sizer.Add(self.PCalvo, 2, wx.ALL | wx.CENTER)

            self.v13_sizer.Add(texto14, 1, wx.ALL | wx.CENTER)
            self.v13_sizer.Add(self.PCreal, 2, wx.ALL | wx.CENTER)

            self.h11_sizer.Add(self.v13_sizer, 6, wx.CENTER)
            self.h11_sizer.AddStretchSpacer(1)
            self.h11_sizer.Add(self.v12_sizer, 6, wx.CENTER)

            self.v14_sizer.Add(texto3, 1, wx.ALL | wx.CENTER)
            self.v14_sizer.Add(self.h11_sizer, 3, wx.ALL | wx.CENTER)

            self.h12_sizer.Add(self.v14_sizer, 1, wx.CENTER)
            staticboxSizer3.Add(self.h12_sizer, 0, wx.ALL | wx.CENTER, 10)

            #--------------------------------------------------
            '''Static Box 4'''
            self.v11_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h7_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h8_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h9_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h10_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.h7_sizer.Add(texto18, 7, wx.ALIGN_CENTER_VERTICAL)
            self.h7_sizer.AddStretchSpacer(1)
            self.h7_sizer.Add(self.AlturaMM, 5, wx.CENTER)

            self.h8_sizer.Add(texto19, 7, wx.ALIGN_CENTER_VERTICAL)
            self.h8_sizer.AddStretchSpacer(1)
            self.h8_sizer.Add(self.DiametroMM, 5, wx.CENTER)

            self.h9_sizer.Add(texto13, 7, wx.ALIGN_CENTER_VERTICAL)
            self.h9_sizer.AddStretchSpacer(1)
            self.h9_sizer.Add(self.AlturaFinal, 5, wx.CENTER)

            self.v11_sizer.Add(self.h7_sizer, 5, wx.ALL | wx.EXPAND  | wx.CENTER)
            self.v11_sizer.AddStretchSpacer(1)
            self.v11_sizer.Add(self.h8_sizer, 5, wx.ALL | wx.EXPAND  | wx.CENTER)
            self.v11_sizer.AddStretchSpacer(1)
            self.v11_sizer.Add(self.h9_sizer, 5, wx.ALL | wx.EXPAND  | wx.CENTER)

            self.h10_sizer.Add(self.v11_sizer, 1, wx.CENTER)
            staticboxSizer4.Add(self.h10_sizer, 0, wx.ALL | wx.EXPAND  | wx.CENTER, 10)

            #--------------------------------------------------
            '''Static Box 5'''
            self.v8_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v9_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v10_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h5_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h6_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.v8_sizer.Add(texto17, 1, wx.ALL | wx.CENTER)
            self.v8_sizer.Add(self.SigmaAlvo, 2, wx.ALL | wx.CENTER)

            self.v9_sizer.Add(texto15, 1, wx.ALL | wx.CENTER)
            self.v9_sizer.Add(self.SigmaReal, 2, wx.ALL | wx.CENTER)

            self.h5_sizer.Add(self.v9_sizer, 6, wx.CENTER)
            self.h5_sizer.AddStretchSpacer(1)
            self.h5_sizer.Add(self.v8_sizer, 6, wx.CENTER)

            self.v10_sizer.Add(texto4, 1, wx.ALL | wx.CENTER)
            self.v10_sizer.Add(self.h5_sizer, 3, wx.ALL | wx.CENTER)

            self.h6_sizer.Add(self.v10_sizer, 1, wx.CENTER)
            staticboxSizer5.Add(self.h6_sizer, 0, wx.ALL | wx.CENTER, 10)

            #--------------------------------------------------
            '''Static Box 6'''
            self.v3_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v4_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v5_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v6_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v7_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h2_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h3_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h4_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.v3_sizer.Add(texto24, 1, wx.ALL | wx.CENTER)
            self.v3_sizer.Add(self.GolpeAtual, 2, wx.ALL | wx.CENTER, 5)

            self.v4_sizer.Add(texto23, 1, wx.ALL | wx.CENTER)
            self.v4_sizer.Add(self.freq, 2, wx.ALL | wx.CENTER, 5)

            self.v5_sizer.Add(texto22, 1, wx.ALL | wx.CENTER)
            self.v5_sizer.Add(self.NGolpes, 2, wx.ALL | wx.CENTER, 5)

            self.v6_sizer.Add(texto21, 1, wx.ALL | wx.CENTER)
            self.v6_sizer.Add(self.fase, 2, wx.ALL | wx.CENTER, 5)

            self.h2_sizer.Add(self.v4_sizer, 3, wx.CENTER)
            self.h2_sizer.AddStretchSpacer(1)
            self.h2_sizer.Add(self.v3_sizer, 4, wx.CENTER)

            self.h3_sizer.Add(self.v6_sizer, 3, wx.CENTER)
            self.h3_sizer.AddStretchSpacer(1)
            self.h3_sizer.Add(self.v5_sizer, 4, wx.CENTER)

            self.v7_sizer.Add(self.h3_sizer, 3, wx.ALL | wx.CENTER)
            self.v7_sizer.AddStretchSpacer(1)
            self.v7_sizer.Add(self.h2_sizer, 3, wx.ALL | wx.CENTER)

            self.h4_sizer.Add(self.v7_sizer, 1, wx.CENTER)
            staticboxSizer6.Add(self.h4_sizer, 0, wx.ALL | wx.CENTER, 10)

            #--------------------------------------------------
            self.sizer = wx.BoxSizer(wx.VERTICAL)
            self.v_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v1_sizer = wx.BoxSizer(wx.VERTICAL)
            self.v2_sizer = wx.BoxSizer(wx.VERTICAL)
            self.h_sizer = wx.BoxSizer(wx.HORIZONTAL)
            self.h1_sizer = wx.BoxSizer(wx.HORIZONTAL)

            self.v_sizer.Add(self.gDP, 1, wx.EXPAND | wx.ALL, 5)
            self.v_sizer.Add(self.condic, 1, wx.EXPAND | wx.ALL, 5)
            self.v_sizer.Add(self.dp, 1, wx.EXPAND | wx.ALL, 5)

            self.v1_sizer.Add(staticboxSizer3, 15, wx.EXPAND | wx.ALL)
            self.v1_sizer.AddStretchSpacer(1)
            self.v1_sizer.Add(staticboxSizer4, 20, wx.EXPAND | wx.ALL)

            self.v2_sizer.Add(staticboxSizer5, 15, wx.EXPAND | wx.ALL)
            self.v2_sizer.AddStretchSpacer(1)
            self.v2_sizer.Add(staticboxSizer6, 20, wx.EXPAND | wx.ALL)

            self.h1_sizer.Add(staticboxSizer1, 1, wx.EXPAND | wx.ALL, 3)
            self.h1_sizer.Add(self.v1_sizer, 1, wx.EXPAND | wx.ALL, 3)
            self.h1_sizer.Add(self.v2_sizer, 1, wx.EXPAND | wx.ALL, 3)

            self.h_sizer.Add(self.h1_sizer, 12, wx.EXPAND | wx.ALL, 5)
            self.h_sizer.Add(self.v_sizer, 1, wx.EXPAND | wx.ALL)
            self.h_sizer.AddStretchSpacer(4)

            self.sizer.Add(self.h_sizer, 0,  wx.EXPAND | wx.ALL, 10)
            self.SetSizer(self.sizer)

            self.timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self.TimeInterval, self.timer)
            self.Bind(wx.EVT_CHECKBOX, self.onCheck, self.ensaioAuto)
            self._fase = 0  #condicao dos fases inicia com zero
            self.erro = False  #indica se há erros na execução
            self.Automatico = True #inicia  com o ensaio Automatico sendo true
            self.ensaioAuto.SetValue(True)

    #--------------------------------------------------
        '''Função CheckBox'''
        def onCheck(self, event):
            print '\nBottomPanel - onCheck'
            global Automatico
            if  self.ensaioAuto.GetValue() == False:
                self.Automatico = False
            else:
                self.Automatico = True

    #--------------------------------------------------
        '''Função responsável em realizar a CONEXÃO'''
        def LTESTE(self, event):
            print '\nBottomPanel - LTESTE'

            try:
                threadConection = ConexaoThread.ConexaoThread(1.05)
                dlg = My.MyProgressDialog(2)
                dlg.ShowModal()
                cond = threadConection.ret()
                if cond[0] == 'connectado':
                    menssagError = wx.MessageDialog(self, 'CONECTADO!', 'EDP', wx.OK|wx.ICON_AUTH_NEEDED)
                    aboutPanel = wx.TextCtrl(menssagError, -1, style = wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL)
                    menssagError.ShowModal()
                    menssagError.Destroy()
                    con.modeConectDNIT134() #acessa o ensaio da 134 no arduino (134 é tido como modelo padrão)
                    self.LTeste.Disable()
                    self.LZero.Enable()

                    #--------------------------------------------------
                    def worker(self):
                        global conddd
                        global condition
                        global conditionEnsaio
                        global Fase
                        global Ti
                        global Pausa
                        global X
                        global Y
                        global H
                        global DefResiliente
                        global DefPermanente
                        global pc
                        global pg
                        global REFERENCIA1
                        global REFERENCIA2
                        global REFERENCIA_MEDIA
                        global ntglp
                        global freq
                        con.modeI()  #inicia o modo de impressão de dados
                        condition = True
                        conditionEnsaio = False
                        cnt = 0
                        cont = 0
                        cont1 = 0
                        amplitudeMax = 0
                        amplitudeMax2 = 0
                        patamarAnterior = 0
                        patamar = 0
                        DefResiliente = 0
                        DefPermanente = 0
                        pc = 0
                        pg = 0
                        GolpeAnterior = -1
                        self.leituraZerob1 = 0
                        self.leituraZerob2 = 0
                        x_counter = 0
                        valores = [0,0,0,0,0,0,0,0,0,0]
                        while conddd:
                            while condition == True:
                                valores = con.ColetaI(valores)
                                if cont1 >= 20: #mede a frequencia da impressão de dados na tela
                                    self.y1mm.Clear()
                                    self.y2mm.Clear()
                                    self.y1V.Clear()
                                    self.y2V.Clear()
                                    self.PCreal.Clear()
                                    self.SigmaReal.Clear()
                                    self.AlturaFinal.Clear()
                                    self.valorLeitura0 = valores[1] #usado apenas no LZERO
                                    self.valorLeitura1 = valores[2] #usado apenas no LZERO
                                    self.y1mm.AppendText(str(round(abs(valores[1]-self.leituraZerob1), 4)))
                                    self.y2mm.AppendText(str(round(abs(valores[2]-self.leituraZerob2), 4)))
                                    self.y1V.AppendText(str(round((valores[3]), 2)))
                                    self.y2V.AppendText(str(round((valores[4]), 2)))
                                    self.PCreal.AppendText(str(round(abs((valores[5])), 3)))
                                    self.SigmaReal.AppendText(str(round(abs(valores[6]-valores[5]), 3)))
                                    if self.leituraZerob1 == 0:
                                        self.AlturaFinal.AppendText(str(round(H, 3)))
                                    else:
                                        self.AlturaFinal.AppendText(str(round(H-abs(ymedio), 3)))
                                    if cont1 == 20:
                                        cont1 = 0
                                cont1 = cont1 + 1

                                ntglp = valores[9] #numero total de golpes
                                y1 = valores[1]-self.leituraZerob1
                                y2 = valores[2]-self.leituraZerob2  #alterar essa linha quando usar os 2 sensores
                                ymedio = (y1 + y2)/2 + H0 #A média + H0 que é o ponto de referência inicial

                                # Dados para a parte GRÁFICA #
                                if conditionEnsaio == True and valores[8] >= 0:
                                    X = np.append(X, valores[0])
                                    Y = np.append(Y, ymedio)
                                    x_counter = len(X)
                                    if x_counter >= 500: #antes era 1500
                                        X = np.delete(X, 0, 0)
                                        Y = np.delete(Y, 0, 0)

                                    # Dados do dp #
                                    if Fase == 'DP' and Pausa == False:
                                        #PEGA OS VALORES DE REFERENCIA
                                        if valores[0] == 0.01:
                                            REFERENCIA1 = y1+H0
                                            REFERENCIA2 = y2+H0
                                            REFERENCIA_MEDIA = ymedio
                                        if valores[0] > 0.2 and valores[0] < 0.5:
                                            REFERENCIA1 = (REFERENCIA1 + (y1+H0))/2
                                            REFERENCIA2 = (REFERENCIA2 + (y2+H0))/2
                                            REFERENCIA_MEDIA = (REFERENCIA_MEDIA + ymedio)/2

                                        #condicao de erro para o ensaio
                                        if int(valores[7]) == 1:
                                            print "ERRO NO ENSAIO"

                                        #PRESSÕES DO ENSAIO
                                        pc = (pc + valores[5])/2
                                        pg = (pg + (valores[6]-valores[5]))/2

                                        D = valores[0] - int(valores[0])
                                        #REFERENTE AOS DADOS DE DEFORMAÇÃO PERMANENTE FREQ 1
                                        if freq == '1':
                                            if D == 0.01:
                                                patamar = ymedio
                                                amplitudeMax = ymedio
                                            if D < 0.2:
                                                if ymedio > amplitudeMax:
                                                    amplitudeMax = ymedio
                                            if D > 0.2 and D < 0.9:
                                                patamar = (patamar + ymedio)/2
                                            if D > 0.9:
                                                DefResiliente  = amplitudeMax - patamar
                                                DefPermanente = patamar
                                                patamarAnterior = patamar
                                            #print D, DefResiliente, DefPermanente, REFERENCIA_MEDIA

                                        #REFERENTE AOS DADOS DE DEFORMAÇÃO PERMANENTE FREQ 2
                                        if freq == '2':
                                            if D == 0.01:
                                                patamar = ymedio
                                                amplitudeMax = ymedio
                                                amplitudeMax2 = ymedio
                                            if D < 0.2:
                                                if ymedio > amplitudeMax:
                                                    amplitudeMax = ymedio
                                            if D > 0.2 and D < 0.5:
                                                patamar = (patamar + ymedio)/2
                                            if D > 0.5 and D < 0.65:
                                                if ymedio > amplitudeMax2:
                                                    amplitudeMax2 = ymedio
                                                DefResiliente  = amplitudeMax - patamar
                                                DefPermanente = patamar
                                            if D > 0.6 and D < 0.9:
                                                patamar = (patamar + ymedio)/2
                                            if D > 0.9:
                                                DefResiliente  = amplitudeMax2 - patamar
                                                DefPermanente = patamar
                                            #print D, DefResiliente, DefPermanente, REFERENCIA_MEDIA
                                    if int(valores[8]) != GolpeAnterior:
                                        GolpeAnterior = int(valores[8])
                                        self.GolpeAtual.Clear()
                                        self.GolpeAtual.AppendText(str(int(valores[8])))
                        print "Thread-worker-Finalizada\n"
                    #--------------------------------------------------
                    self.t = threading.Thread(target=worker, args=(self,))
                    self.t.start()

                else:
                    menssagError = wx.MessageDialog(self, 'Não é possível manter uma conexão serial!', 'EDP', wx.OK|wx.ICON_EXCLAMATION)
                    aboutPanel = wx.TextCtrl(menssagError, -1, style = wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL)
                    menssagError.ShowModal()
                    menssagError.Destroy()
            except:
                menssagError = wx.MessageDialog(self, 'ERRO AO EXECUTAR O CONECTAR', 'EDP', wx.OK|wx.ICON_EXCLAMATION)
                aboutPanel = wx.TextCtrl(menssagError, -1, style = wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL)
                menssagError.ShowModal()
                menssagError.Destroy()

    #--------------------------------------------------
        '''Função responsável pela leitura zero'''
        def LZERO(self, event):
            print '\nBottomPanel - LZERO'
            self.freq.Enable()
            self.condic.Enable()
            self.dp.Enable()
            self.LTeste.Disable()
            self.leituraZerob1 = float(self.valorLeitura0)
            self.leituraZerob2 = float(self.valorLeitura1)
            print self.leituraZerob1
            print self.leituraZerob2
    
    #--------------------------------------------------
        '''Função responsável em mostrar o gráfico da deformação permanente'''
        def GDP(self, event):
            print '\nBottomPanel - GDP'
            xy = bancodedados.dados_GDP_179(idt)
            X = xy[0]
            Y = xy[1]
            try:
                self.window.Close()
            except:
                pass

            self.window = wx.MiniFrame(self, title="", size=(600,400), style= wx.CLOSE_BOX | wx.CAPTION | wx.STAY_ON_TOP)
            fig = plt.figure(constrained_layout=True)  
            axes = fig.add_subplot(111) 
            axes.set_xlabel("GOLPE")
            axes.set_ylabel("D. P. (mm)")
            axes.plot(X, Y, 'r-')
            canvas = FigureCanvas(self.window, -1, fig)   
            canvas.draw()
            self.window.Show() 

    #--------------------------------------------------
        '''Função responsável em realizar o CONDICIONAMENTO'''
        def CONDIC(self, event):
            print '\nBottomPanel - CONDIC'
            global condition
            global Fase
            global idt
            global REFERENCIA1
            global REFERENCIA2
            global REFERENCIA_MEDIA
            global modeADM
            Fase = 'CONDICIONAMENTO'
            self.erro = False

            self.LZero.Disable()
            self.freq.Disable()
            self.dp.Disable()
            self.condic.Disable()
            self.PCalvo.Clear()
            self.SigmaAlvo.Clear()
            self.fase.Clear()
            self.NGolpes.Clear()
            self.GolpeAtual.Clear()
            self.PCalvo.AppendText("%.3f" % VETOR_COND[0][0])
            self.SigmaAlvo.AppendText("%.3f" % (VETOR_COND[0][1]-VETOR_COND[0][0]))
            self.NGolpes.AppendText(str(glpCOND))
            self.fase.AppendText('1')
            self.GolpeAtual.AppendText(str(0))

            info = "EDP 179/2018IE"
            titulo = "Preparação da câmara triaxial."
            message1 = "Verifique se está tudo certo!"
            message2 = "Se as válvulas de escape estão fechadas, se as válvulas reguladoras de pressão estão devidamentes conectadas, se a passagem de ar comprimido para o sistema está liberado e se a câmara triaxial está totalmente fechada e com o fluido de atrito para o suporte vertical."
            dlg = dialogoDinamico(2, info, titulo, message1, message2, "", None)
            if dlg.ShowModal() == wx.ID_OK:
                freq = self.freq.GetValue()
                bancodedados.Update_freq(idt, int(freq))
                bancodedados.data_inicio_Update_idt(idt)
                if self.Automatico == False:
                    self.graph.fim_inicio.SetLabel('INICIO')
                    self.graph.Bind(wx.EVT_BUTTON, self.graph.INICIO, self.graph.fim_inicio)
                    self.graph.fim_inicio.Enable()

                if self.Automatico == True:
                    self.graph.Bind(wx.EVT_BUTTON, self.graph.INICIO, self.graph.fim_inicio)
                    evt = wx.PyCommandEvent(wx.EVT_BUTTON.typeId, self.graph.fim_inicio.GetId())
                    wx.PostEvent(self.graph.fim_inicio, evt)

    #--------------------------------------------------
        '''Função responsável em realizar a DEFORMAÇÃO PERMANENTE'''
        def DP(self, event):
            print '\nBottomPanel - DP'
            global condition
            global Fase
            global glpDP
            global idt
            global REFERENCIA1
            global REFERENCIA2
            global REFERENCIA_MEDIA
            self.erro = False
            fase = 1

            self.LZero.Disable()
            self.freq.Disable()
            self.dp.Disable()
            self.condic.Disable()
            self.PCalvo.Clear()
            self.SigmaAlvo.Clear()
            self.fase.Clear()
            self.NGolpes.Clear()
            self.GolpeAtual.Clear()
            self.PCalvo.AppendText("%.3f" % VETOR_DP[0][0])
            self.SigmaAlvo.AppendText("%.3f" % (VETOR_DP[0][1]-VETOR_DP[0][0]))
            self.NGolpes.AppendText(str(glpDP))
            self.fase.AppendText('1')
            self.GolpeAtual.AppendText(str(0))

            if Fase == '':
                info = "EDP 179/2018ME"
                titulo = "Preparação da câmara triaxial."
                message1 = "Verifique se está tudo certo!"
                message2 = "Se as válvulas de escape estão fechadas, se as válvulas reguladoras de pressão estão devidamentes conectadas, se a passagem de ar comprimido para o sistema está liberado e se a câmara triaxial está totalmente fechada e com o fluido de atrito para o suporte vertical."
                dlg = dialogoDinamico(2, info, titulo, message1, message2, "", None)
                if dlg.ShowModal() == wx.ID_OK:
                    Fase = 'DP'
                    if self._fase == 0:
                        freq = self.freq.GetValue()
                        bancodedados.Update_freq(idt, int(freq))
                        bancodedados.data_inicio_Update_idt(idt)

                    if self._fase >= 0 and self._fase < fase and self.Automatico == False:
                        self.graph.fim_inicio.SetLabel('INICIO')
                        self.graph.Bind(wx.EVT_BUTTON, self.graph.INICIO, self.graph.fim_inicio)
                        self.graph.fim_inicio.Enable()

                    if self._fase >= 0 and self._fase < fase and self.Automatico == True:
                        self.graph.Bind(wx.EVT_BUTTON, self.graph.INICIO, self.graph.fim_inicio)
                        evt = wx.PyCommandEvent(wx.EVT_BUTTON.typeId, self.graph.fim_inicio.GetId())
                        wx.PostEvent(self.graph.fim_inicio, evt)

                    if self._fase >= fase and self.erro == False:
                        self.dp.Disable()
                        self.pressao_zero(VETOR_DP[0][0], VETOR_DP[0][1])
                        self._fase = 0
                        bancodedados.data_final_Update_idt(idt)
                        dlg3 = dialogoDinamico(3, "EDP 179/2018ME", "O ENSAIO FOI FINALIZADO!", "Os relatórios de extração são gerados na tela inicial.", "FIM!", "", None)
                        dlg3.ShowModal()
            else:
                Fase = 'DP'
                if self._fase >= 0 and self._fase < fase and self.Automatico == False:
                    self.graph.fim_inicio.SetLabel('INICIO')
                    self.graph.Bind(wx.EVT_BUTTON, self.graph.INICIO, self.graph.fim_inicio)
                    self.graph.fim_inicio.Enable()

                if self._fase >= 0 and self._fase < fase and self.Automatico == True:
                    self.graph.Bind(wx.EVT_BUTTON, self.graph.INICIO, self.graph.fim_inicio)
                    evt = wx.PyCommandEvent(wx.EVT_BUTTON.typeId, self.graph.fim_inicio.GetId())
                    wx.PostEvent(self.graph.fim_inicio, evt)

                if self._fase >= fase and self.erro == False:
                    self.dp.Disable()
                    self.pressao_zero(VETOR_DP[0][0], VETOR_DP[0][1])
                    self._fase = 0
                    bancodedados.data_final_Update_idt(idt)
                    dlg3 = dialogoDinamico(3, "EDP 179/2018ME", "O ENSAIO FOI FINALIZADO!", "Os relatórios de extração são gerados na tela inicial.", "FIM!", "", None)
                    dlg3.ShowModal()

    #--------------------------------------------------
        '''Função responsável em zerar a pressão do sistema'''
        def pressao_zero(self, p2Sen ,p1Sen):
            print '\nBottomPanel - pressao_zero'
            global condition
            condition = False
            threadConection = DinamicaThread.DinamicaThreadOneZero(0.005, p1Sen) #0.005 é o menor valor de pressão admissível para valvula dinamica
            dlgC1 = My.MyProgressDialog(4)
            dlgC1.ShowModal()
            time.sleep(1)
            #threadConection = MotorThread.MotorThreadZero(0.030)  #0.030 é o menor valor de pressão admissível para SI do motor de passos
            threadConection = DinamicaThread.DinamicaThreadTwoZero(0.005, p2Sen) #0.005 é o menor valor de pressão admissível para valvula dinamica
            dlg2 = My.MyProgressDialog(4)
            dlg2.ShowModal()
            condition = True

    #--------------------------------------------------
        '''Função responsável pela plotagem'''
        def TimeInterval(self, event):
            print '\nBottomPanel - TimeInterval'
            global mult
            mult += 1
            self.graph.draw()


'''Tela Realização do Ensaio'''
class TelaRealizacaoEnsaioDNIT179(wx.Dialog):
    #--------------------------------------------------
        def __init__(self, identificador, tipo, diametro, altura, *args, **kwargs):
            wx.Dialog.__init__(self, parent = None, title = 'EDP - Ensaios Dinâmicos para Pavimentação - DNIT 179/2018IE - Tela Ensaio', size = (1000,750), style = wx.MAXIMIZE_BOX | wx.MINIMIZE_BOX | wx.SYSTEM_MENU | wx.CLOSE_BOX | wx.CAPTION)
            self.Bind(wx.EVT_CLOSE, self.onExit)
            
            '''Variáveis Globais'''
            global conddd #variavel de controle das Threads
            global idt #identificador do ensaio
            global leituraZerob1 #leitura zero do sensor 1
            global leituraZerob2 #leitura zero do sensor 2
            global H  #Altura do corpo de prova em milímetros
            global Diam  #Diametro do corpo de prova em milímetros
            global H0 #Altura de referência do medididor de deslocamento
            global X  #valores X do gráfico
            global Y  #valores Y do gráfico
            global pc #pressão confinante
            global pg #pressão dos golpes (desvio)
            global DefResiliente #Deformação resiliente ou recuperável
            global DefPermanente #Deformação Permanente
            global REFERENCIA1 #referencia de ponto de partida para o sensor 1
            global REFERENCIA2 #referencia de ponto de partida para o sensor 2
            global REFERENCIA_MEDIA #referencia de ponto de partidada MÉDIA
            global Ti #valor temporal
            global Fase #valor para identificar se esta no CONDICIONAMENTO ou no MR
            global Automatico #idica se o ensaio será automático ou não
            global Pausa #indica se o ensaio foi pausado
            global mult  #Multiplo de 5 que ajuda a arrumar o gráfico em 5 em 5
            global glpCOND #quantidade de golpes do CONDICIONAMENTO
            global glpDP #quantidade de golpes do DP
            global ntglp #quantidade total de golpes disponíveis
            global modeADM #modo Administrador de salvar dados (apenas para dbug)
            global freq #frequencia do ensaio
            global temposDNIT179_01
            global temposDNIT179_02
            global VETOR_COND #Vetor com os pares de pressões do CONDICIONAMENTO
            global VETOR_DP #Vetor com os pares de pressões do MR
    
            '''Banco de dados'''
            pressoes = bdConfiguration.QD_179_MOD()
            config = bdConfiguration.CONFIG_179()

            con
            idt = identificador
            H0 = 0.0000001
            H = altura
            Diam = diametro
            glpCOND = config[0] #número total de golpes do condicionamento
            glpDP = config[1] #número total de golpes da deformação permanente
            modeADM = False
            mult = 0
            Pausa = False
            X = np.array([])
            Y = np.array([])
            Fase = ''
            freq = 1  #a frequencia inicia sendo 1 (default)

            VETOR_COND = pressoes[0]

            VETOR_DP =  [pressoes[1][tipo]]

            temposDNIT179_01 = [1,2,3,4,5,10,15,20,30,40,50,60,70,80,90,100,200,300,400,500,600,700,800,900]
            temposDNIT179_02 = [1000,2000,3000,4000,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,15000,16000,17000,18000,
                                19000,20000,21000,22000,23000,24000,25000,26000,27000,28000,29000,30000,31000,32000,33000,34000,
                                35000,36000,37000,38000,39000,40000,41000,42000,43000,44000,45000,46000,47000,48000,49000,50000,
                                55000,60000,65000,70000,75000,80000,85000,90000,95000,100000,110000,120000,130000,140000,150000,
                                160000,170000,180000,190000,200000,210000,220000,230000,240000,250000,260000,270000,280000,290000,
                                300000,310000,320000,330000,340000,350000,360000,370000,380000,390000,400000,410000,420000,430000,
                                440000,450000,460000,470000,480000,490000,500000]

            '''Iserção do IconeLogo'''
            try:
                ico = wx.Icon('icons\logo.ico', wx.BITMAP_TYPE_ICO)
                self.SetIcon(ico)
            except:
                pass

            '''Configurações do SPLITTER'''
            splitter = wx.SplitterWindow(self)
            top = TopPanel(splitter, self)
            self.bottom = BottomPanel(splitter, top)
            splitter.SplitHorizontally(top, self.bottom, 0)
            splitter.SetMinimumPaneSize(390)
            top.draww()
            #top.draw(X,Y)
            '''plt.ion()'''

            self.Centre()
            self.Show()
            self.Maximize(True)

            '''Dialogo Inicial'''
            info = "EDP 179/2018IE"
            titulo = "Ajuste o Zero dos LVDTs"
            message1 = "Com o valor entre:"
            message2 = "2.5 e 3.0 Volts"
            message3 = "realizando a CONEXAO"
            dlg = dialogoDinamico(1, info, titulo, message1, message2, message3, None)
            dlg.ShowModal()

        #--------------------------------------------------
        def onExit(self, event):
            '''Opcao Sair'''
            self.Destroy()
