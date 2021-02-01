# -*- coding: utf-8 -*-

'''Bibliotecas'''

import wx
import bancodedadosCAB
import wx.lib.mixins.listctrl as listmix
from front.NovoCabecalho import NovoCabecalho
from wx.lib.agw import ultimatelistctrl as ULC

'''lista = [[0, [u'(Default)']], [1, [u'(Default1)']], [2, [u'(Default2)']], [3, [u'(Default3)']], [4, [u'(Default4)']], [5, [u'(Default5)']], [6, [u'(Default6)']], [7, [u'(Default7)']], [8, [u'(Default8)']], [9, [u'(Default9)']], [10, [u'(Default10)']]]'''

'''Classe da Lista editável'''
class EditableListCtrl(ULC.UltimateListCtrl, listmix.ListCtrlAutoWidthMixin):
    #----------------------------------------------------------------------
        def __init__(self, parent, ID=wx.ID_ANY, pos=wx.DefaultPosition, size=wx.DefaultSize, style=0):
            ULC.UltimateListCtrl.__init__(self, parent, ID, pos, size, agwStyle = ULC.ULC_REPORT | ULC.ULC_HAS_VARIABLE_ROW_HEIGHT | ULC.ULC_HRULES | ULC.ULC_VRULES | ULC.ULC_NO_HIGHLIGHT)

        def UpdateListCtrl(self):
            self.DeleteAllItems()
            lista = bancodedadosCAB.ListaVisualizacaoCab()
            index = 0

            for key, row in lista:
                   if index == 0:
                           pos = self.InsertStringItem(index, row[0])
                           buttonEDT = wx.Button(self, id = key, label="")
                           buttonDEL = wx.Button(self, id = 15000+key, label="")
                           buttonEDT.SetBitmap(wx.Bitmap(r'icons\icons-neditar-arquivo-24px.png'))
                           buttonDEL.SetBitmap(wx.Bitmap(r'icons\icons-nlixo-24px.png'))
                           self.SetItemWindow(pos, col=1, wnd=buttonEDT, expand=True)
                           self.SetItemWindow(pos, col=2, wnd=buttonDEL, expand=True)
                           self.SetItemData(index, key)
                           index += 1
                   else:
                           pos = self.InsertStringItem(index, row[0])
                           buttonEDT = wx.Button(self, id = key, label="")
                           buttonDEL = wx.Button(self, id = 15000+key, label="")
                           buttonEDT.SetBitmap(wx.Bitmap(r'icons\icons-editar-arquivo-24px.png'))
                           buttonDEL.SetBitmap(wx.Bitmap(r'icons\icons-lixo-24px.png'))
                           self.SetItemWindow(pos, col=1, wnd=buttonEDT, expand=True)
                           self.SetItemWindow(pos, col=2, wnd=buttonDEL, expand=True)
                           self.SetItemData(index, key)
                           index += 1

            if len(lista)>=11:
               self.SetColumnWidth(0, width=210)
               self.SetColumnWidth(1, width=40)
               self.SetColumnWidth(2, width=40)

            else:
               self.SetColumnWidth(0, width=230)
               self.SetColumnWidth(1, width=40)
               self.SetColumnWidth(2, width=40)

'''Tela Cabeçalhos'''
class Cab(wx.Frame):
        #--------------------------------------------------
        def __init__(self, *args, **kwargs):
                wx.Frame.__init__(self, None, -1, 'EDP - Cabeçalhos', style = wx.SYSTEM_MENU | wx.CLOSE_BOX | wx.CAPTION)

                v_sizer = wx.BoxSizer(wx.VERTICAL)
                h_sizer = wx.BoxSizer(wx.HORIZONTAL)
                h2_sizer = wx.BoxSizer(wx.HORIZONTAL)
                panel = wx.Panel(self)

                '''Iserção do IconeLogo'''
                ico = wx.Icon(r'icons\logo.ico', wx.BITMAP_TYPE_ICO)
                self.SetIcon(ico)

                '''Configurações do Size'''
                self.SetSize((360,520))

                self.CadastrarCabecalhoButton = wx.Button(panel, -1, '', size=(30, 30))
                self.CadastrarCabecalhoButton.SetBitmap(wx.Bitmap(r'icons\icons-adicionar-48px.png'))
                self.Bind(wx.EVT_BUTTON, self.NovoCabecalho, self.CadastrarCabecalhoButton)
                v_sizer.AddStretchSpacer(5)
                v_sizer.Add(self.CadastrarCabecalhoButton, 0, wx.ALIGN_CENTER_HORIZONTAL)
                v_sizer.AddStretchSpacer(4)
                panel.SetSizerAndFit(v_sizer)

                lista = bancodedadosCAB.ListaVisualizacaoCab()

                '''Lista dos Cabeçalhos'''
                self.list_ctrl = EditableListCtrl(panel, size=(315,0))
                h_sizer.AddStretchSpacer(5)
                h_sizer.Add(self.list_ctrl, 0, wx.EXPAND)
                h_sizer.AddStretchSpacer(5)
                v_sizer.Add(h_sizer, 40, wx.ALIGN_CENTER_HORIZONTAL)
                v_sizer.AddStretchSpacer(1)
                panel.SetSizerAndFit(v_sizer)

                if len(lista)>=11:
                        self.list_ctrl.InsertColumn(0, 'CABEÇALHOS', wx.LIST_FORMAT_CENTRE, width=210)
                        self.list_ctrl.InsertColumn(1, 'EDT', wx.LIST_FORMAT_CENTRE, width=40)
                        self.list_ctrl.InsertColumn(2, 'DEL', wx.LIST_FORMAT_CENTRE, width=40)
                else:
                        self.list_ctrl.InsertColumn(0, 'CABEÇALHOS', wx.LIST_FORMAT_CENTRE, width=230)
                        self.list_ctrl.InsertColumn(1, 'EDT', wx.LIST_FORMAT_CENTRE, width=40)
                        self.list_ctrl.InsertColumn(2, 'DEL', wx.LIST_FORMAT_CENTRE, width=40)

                index = 0

                list_cab = []
                print lista

                for key, row in lista:
                        list_cab.append(row[0])
                        if index == 0:
                                pos = self.list_ctrl.InsertStringItem(index, row[0])
                                buttonEDT = wx.Button(self.list_ctrl, id = key, label="")
                                buttonDEL = wx.Button(self.list_ctrl, id = 15000+key, label="")
                                buttonEDT.SetBitmap(wx.Bitmap(r'icons\icons-neditar-arquivo-24px.png'))
                                buttonDEL.SetBitmap(wx.Bitmap(r'icons\icons-nlixo-24px.png'))
                                self.list_ctrl.SetItemWindow(pos, col=1, wnd=buttonEDT, expand=True)
                                self.list_ctrl.SetItemWindow(pos, col=2, wnd=buttonDEL, expand=True)
                                self.list_ctrl.SetItemData(index, key)
                                index += 1
                        else:
                                pos = self.list_ctrl.InsertStringItem(index, row[0])
                                buttonEDT = wx.Button(self.list_ctrl, id = key, label="")
                                buttonDEL = wx.Button(self.list_ctrl, id = 15000+key, label="")
                                buttonEDT.SetBitmap(wx.Bitmap(r'icons\icons-editar-arquivo-24px.png'))
                                buttonDEL.SetBitmap(wx.Bitmap(r'icons\icons-lixo-24px.png'))
                                self.list_ctrl.SetItemWindow(pos, col=1, wnd=buttonEDT, expand=True)
                                self.list_ctrl.SetItemWindow(pos, col=2, wnd=buttonDEL, expand=True)
                                self.Bind(wx.EVT_BUTTON, self.Editar, buttonEDT)
                                self.Bind(wx.EVT_BUTTON, self.Deletar, buttonDEL)
                                self.list_ctrl.SetItemData(index, key)
                                index += 1

                self.Bind(wx.EVT_LIST_COL_DRAGGING, self.ColumAdapter, self.list_ctrl)
                self.Bind(wx.EVT_LIST_COL_RIGHT_CLICK, self.ColumAdapter2, self.list_ctrl)
                self.Bind(wx.EVT_LIST_COL_CLICK, self.ColumAdapter3, self.list_ctrl)

                self.list_ctrl.UpdateListCtrl()

                self.combo = wx.ComboBox(panel, value = list_cab[0] ,choices = list_cab, style = wx.EXPAND | wx.CB_READONLY)
                definirAtual = wx.Button(panel, -1, 'Definir Atual')

                if len(list_cab) == 1:
                    definirAtual.SetForegroundColour((119,118,114))

                h2_sizer.AddStretchSpacer(5)
                h2_sizer.Add(self.combo, 10, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 3)
                h2_sizer.Add(definirAtual, 10, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 3)
                h2_sizer.AddStretchSpacer(5)
                v_sizer.Add(h2_sizer, 5, wx.ALIGN_CENTER_HORIZONTAL)
                v_sizer.AddStretchSpacer(1)

                panel.SetSizer(v_sizer)
                self.Centre()
                self.Show()

    #--------------------------------------------------
        def NovoCabecalho(self, event):
            dialogo = NovoCabecalho()

            self.list_ctrl.UpdateListCtrl()

            '''index = 0
            self.list_ctrl.DeleteAllItems()
            lista = bancodedadosCAB.ListaVisualizacaoCap()



            for key, row in lista:
                pos = self.list_ctrl.InsertStringItem(index, row[0])
                self.list_ctrl.SetStringItem(index, 1, row[1])
                buttonEDT = wx.Button(self.list_ctrl, id = key, label="")
                buttonDEL = wx.Button(self.list_ctrl, id = 15000+key, label="")
                buttonEDT.SetBitmap(wx.Bitmap(r'icons\icons-editar-arquivo-24px.png'))
                buttonDEL.SetBitmap(wx.Bitmap(r'icons\icons-lixo-24px.png'))
                self.list_ctrl.SetItemWindow(pos, col=2, wnd=buttonEDT, expand=True)
                self.list_ctrl.SetItemWindow(pos, col=3, wnd=buttonDEL, expand=True)
                self.Bind(wx.EVT_BUTTON, self.Editar, buttonEDT)
                self.Bind(wx.EVT_BUTTON, self.Deletar, buttonDEL)
                self.list_ctrl.SetItemData(index, key)
                index += 1

            lista = bancodedadosCAB.ListaVisualizacaoCap()

            if len(lista) >=11:
                self.list_ctrl.SetColumnWidth(0, width=210)
                self.list_ctrl.SetColumnWidth(1, width=40)
                self.list_ctrl.SetColumnWidth(2, width=40)
            else:
                self.list_ctrl.SetColumnWidth(0, width=230)
                self.list_ctrl.SetColumnWidth(1, width=40)
                self.list_ctrl.SetColumnWidth(2, width=40)'''

    #--------------------------------------------------
        def Selecionar(self, event):
            a = self.list_ctrl.GetFirstSelected()
            print a

    #--------------------------------------------------
        def Editar(self, event):
            a = self.list_ctrl.GetFocusedItem()
            print a
            '''id = event.GetId()
            dialogo = edtCapsula(id)
            resultado = dialogo.ShowModal()
            self.list_ctrl.UpdateListCtrl()'''

    #--------------------------------------------------
        def Deletar(self, event):
            id = event.GetId()
            id = id - 15000

            '''Diálogo se deseja realmente excluir a Cápsula'''
            dlg = wx.MessageDialog(None, 'Deseja mesmo excluir essa Cápsula?', 'EAU', wx.YES_NO | wx.CENTRE| wx.NO_DEFAULT )
            result = dlg.ShowModal()

            if result == wx.ID_YES:
                bancodedadosCAB.deleteCap(id)
                dlg.Destroy()
                self.list_ctrl.UpdateListCtrl()
            else:
                dlg.Destroy()

    #--------------------------------------------------
        def ColumAdapter(self, event):
             '''Ajusta os tamanhos das colunas ao arrastar'''
             lista = bancodedadosCAB.ListaVisualizacaoCap()
             if len(lista) >=11:
                 self.list_ctrl.SetColumnWidth(0, width=210)
                 self.list_ctrl.SetColumnWidth(1, width=40)
                 self.list_ctrl.SetColumnWidth(2, width=40)
             else:
                 self.list_ctrl.SetColumnWidth(0, width=230)
                 self.list_ctrl.SetColumnWidth(1, width=40)
                 self.list_ctrl.SetColumnWidth(2, width=40)

    #--------------------------------------------------
        def ColumAdapter2(self, event):
            '''Ajusta os tamanhos das colunas ao clicar com botão esquerdo sobre a coluna'''
            lista = bancodedadosCAB.ListaVisualizacaoCap()
            if len(lista) >=11:
                self.list_ctrl.SetColumnWidth(0, width=210)
                self.list_ctrl.SetColumnWidth(1, width=40)
                self.list_ctrl.SetColumnWidth(2, width=40)
            else:
                self.list_ctrl.SetColumnWidth(0, width=230)
                self.list_ctrl.SetColumnWidth(1, width=40)
                self.list_ctrl.SetColumnWidth(2, width=40)

    #--------------------------------------------------
        def ColumAdapter3(self, event):
            '''Ajusta os tamanhos das colunas ao clicar com o botão direito sobre a coluna'''
            lista = bancodedadosCAB.ListaVisualizacaoCap()
            if len(lista) >=11:
                self.list_ctrl.SetColumnWidth(0, width=210)
                self.list_ctrl.SetColumnWidth(1, width=40)
                self.list_ctrl.SetColumnWidth(2, width=40)
            else:
                self.list_ctrl.SetColumnWidth(0, width=230)
                self.list_ctrl.SetColumnWidth(1, width=40)
                self.list_ctrl.SetColumnWidth(2, width=40)
