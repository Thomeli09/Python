# -*- coding: utf-8 -*-
"""
Created on Wed Oct 30 14:15:29 2024

@author: Thommes Eliott
"""
# General Plotting library

import matplotlib.pyplot as plt


"""
Base de donnée
"""
# Paramètre d'affichage
class ParamPLT:
    def __init__(self, colour, linetype, marker, linesize, fontsize, scale, scale3D):
        # Plot
        self.Colour = colour
        self.LineType = linetype
        self.MarkerType = marker
        self.LineSize = 2
        # Text
        self.FontSize = fontsize
        self.TitleSize = fontsize
        self.XLabel = None
        self.YLabel = None
        self.ZLabel = None
        self.Title = None
        self.Legends = []
        # Scale
        self.Scale = scale
        self.Scale3D = scale3D
        # Plot Format
        self.PltAspect = None
        # Grid
        self.GridAxis = 'both'
        self.GridColour = None
        self.GridLineType = None
        self.GridLineSize = 0.4

    @property
    def getColour(self):
        return self.Colour

    @getColour.setter
    def getColour(self, colour):
        self.Colour = colour

    @property
    def getLineType(self):
        # Determine line type based on linetype input
        line_type_dict = {0: '-', 1: '-', 2: '--', 3: '-.', 4: ':'}
        line_type = line_type_dict.get(self.LineType, '-')
        return line_type

    @getLineType.setter
    def getLineType(self, linetype):
        self.LineType = linetype

    @property
    def getMarker(self):
        marker_type_dict = {0: '', 1: '.', 2: ',', 3: 'o', 4: 'v', 5: '^',
                            6: '<', 7: '>', 8: '1', 9: '2', 10: '3', 11: '4',
                            12: '8', 13: 's', 14: 'p', 15: 'P', 16: '*',
                            17: 'h', 18: 'H', 19: '+', 20: 'x', 21: 'X',
                            22: 'D', 23: 'd', 24: '|', 25: '_'}
        marker_type = marker_type_dict.get(self.MarkerType, '')
        return marker_type

    @getMarker.setter
    def getMarker(self, marker):
        self.MarkerType = marker

    @property
    def getLineSize(self):
        return self.LineSize

    @getLineSize.setter
    def getLineSize(self, linesize):
        self.LineSize = linesize

    @property
    def getFontSize(self):
        return self.FontSize

    @getFontSize.setter
    def getFontSize(self, fontsize):
        self.FontSize = fontsize

    @property
    def getTitleSize(self):
        return self.TitleSize

    @getFontSize.setter
    def getTitleSize(self, TitleSize):
        self.TitleSize = TitleSize

    @property
    def getTitle(self):
        return self.Title

    @property
    def getXLabel(self):
        return self.XLabel

    @property
    def getYLabel(self):
        return self.YLabel

    @property
    def getZLabel(self):
        return self.ZLabel

    def getLabelTitle(self, Title, XLabel, YLabel, ZLabel=None):
        self.Title = Title
        self.XLabel = XLabel
        self.YLabel = YLabel
        self.ZLabel = ZLabel

    @property
    def getLegends(self):
        if not self.Legends:
            return None
        else:
            return self.Legends.pop(0)
    
    @getLegends.setter
    def getLegends(self, Legends):
        self.Legends = Legends

    @property
    def getScale(self):
        return self.Scale

    @getScale.setter
    def getScale(self, scale):
        self.Scale = scale

    @property
    def getScale3D(self):
        return self.Scale3D

    @getScale3D.setter
    def getScale3D(self, scale3D):
        self.Scale3D = scale3D

    @property
    def getFMT(self):
        # Construct plot format
        fmt = f"{self.getMarker}{self.getLineSize}{self.getColour}"
        return fmt

    @property
    def getAspect(self):
        return self.PltAspect

    @getAspect.setter
    def getAspect(self, Aspect):
        self.PltAspect = Aspect

    def getGrid(self, Axis, Colour=None):
        if Axis == 1:
            self.GridAxis = 'x'
        elif Axis == 2:
            self.GridAxis = 'y'
        elif Axis >= 0:
            self.GridAxis = 'both'
        else:
            self.GridAxis = None

        self.GridColour = Colour

        self.GridLineType = None
        self.GridLineSize = 0.4

    @property
    def getGridLineType(self):
        # Determine line type based on linetype input
        LineTypeDict = {0: '-', 1: '-', 2: '--', 3: '-.', 4: ':'}
        LineType = LineTypeDict.get(self.GridLineType, '-')
        return LineType

    @getGridLineType.setter
    def getGridLineType(self, LineType):
        self.GridLineType = LineType

    @property
    def getGridLineSize(self):
        return self.GridLineSize

    @getGridLineSize.setter
    def getGridLineSize(self, LineSize):
        self.GridLineSize = LineSize

    @property
    def getGridAxis(self):
        return self.GridAxis

    @getGridAxis.setter
    def getGridAxis(self, GridAxis):
        self.GridAxis = GridAxis



"""
Fcts générales
"""
def ClosePlots(PlotOBJ):
    plt.close(PlotOBJ)

def CloseALLPlots():
    plt.close('all')

def StartPlots():
    plt.figure()


"""
Type de Plots
"""
# 2D
def PLTShow(paramPLT):
    plt.xlabel(paramPLT.getXLabel)
    plt.ylabel(paramPLT.getYLabel)
    plt.title(paramPLT.getTitle)
    if paramPLT.getAspect:
        plt.gca().set_aspect('equal', adjustable='box')
    if paramPLT.getGridAxis:
        plt.grid(axis=paramPLT.getGridAxis,
                 color=paramPLT.getColour,
                 linestyle=paramPLT.getGridLineType,
                 linewidth=paramPLT.getGridLineSize)
    plt.legend()
    plt.show()



# 3D

