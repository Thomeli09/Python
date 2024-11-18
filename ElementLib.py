# -*- coding: utf-8 -*-
"""
Created on Mon Nov  4 14:12:41 2024

@author: Thommes Eliott
"""

# Element library

import matplotlib.pyplot as plt
from GeometryLib import Node, Line


"""
Noeuds d'élément
"""


class StructNode(Node):
    def __init__(self, x, y, z):
        super().__init__(x, y, z)  # pour bénéficier de l'inhéritance
        self.Myy = None
        self.Mzz = None
        self.Tzz = None
        self.Tyy = None
        self.Nxx = None

    @property
    def getMt(self):
        return self.Mt

    @getMt.setter
    def getMt(self, val):
        self.Mt = val

    @property
    def getMyy(self):
        return self.Myy

    @getMyy.setter
    def getMyy(self, val):
        self.Myy = val

    @property
    def getMzz(self):
        return self.Mzz

    @getMzz.setter
    def getMzz(self, val):
        self.Mzz = val

    @property
    def getTzz(self):
        return self.Tzz

    @getTzz.setter
    def getTzz(self, val):
        self.Tzz = val

    @property
    def getTyy(self):
        return self.Tyy

    @getTyy.setter
    def getTyy(self, val):
        self.Tyy = val

    @property
    def getN(self):
        return self.Nxx

    @getN.setter
    def getN(self, val):
        self.Nxx = val


"""
Eléments
"""


class Element(Line):
    def __init__(self, node1, node2, section, CondAppuieN1, CondAppuieN2,
                 Name, Color):
        super().__init__(node1, node2)  # pour bénéficier de l'inhéritance
        self.LStructNode = []
        self.Section = section
        self.CondAppuieN1 = CondAppuieN1
        self.CondAppuieN2 = CondAppuieN2

    @property
    def getStructNodes(self):
        return self.LStructNode

    @getStructNodes.setter
    def getStructNodes(self, LStructNode):
        self.LStructNode = LStructNode

    @property
    def getSection(self):
        return self.Section

    @getSection.setter
    def getSection(self, section):
        self.Section = section

    def PLT2DElement(self, axis, paramPLT):
        if axis[0] == 1:
            xValues = [self.getn1.getx, self.getn2.getx]
        elif axis[0] == 2:
            xValues = [self.getn1.gety, self.getn2.gety]
        else:
            xValues = [self.getn1.getz, self.getn2.getz]

        if axis[1] == 1:
            yValues = [self.getn1.getx, self.getn2.getx]
        elif axis[1] == 2:
            yValues = [self.getn1.gety, self.getn2.gety]
        else:
            yValues = [self.getn1.getz, self.getn2.getz]

        # Plot the line with specified parameters
        plt.plot(xValues, yValues,
                 color=paramPLT.getColour,
                 linestyle=paramPLT.getLineType,
                 marker=paramPLT.getMarker,
                 linewidth=paramPLT.getLineSize,
                 markersize=paramPLT.getLineSize,
                 label=paramPLT.getLegends)
"""
Plot the result of the element in 2d et juste l'élément avec les cond d'appuis

"""


"""
Section
"""
# Simplified cross section definition
class CrossSectionEasy:  # (Surface):
    def __init__(self):
        # super().__init__(node1, node2)  # pour bénéficier de l'inhéritance
        self.bool = False
        self.Name
        self.Color

    @property
    def get(self):
        return False

    @get.setter
    def get(self):
        return False

# Complexe cross section definition
class CrossSection(CrossSectionEasy):  # (Surface):
    def __init__(self, val1, val2):
        super().__init__(val1, val2)  # pour bénéficier de l'inhéritance
        self.bool = False
        """
        Créer ici une structure surface 2d avec plus d'option
        Surface 2D qui est une classe fille de surface mais avec plus d'option
        """

    @property
    def get(self):
        return False

    @get.setter
    def get(self):
        return False

    def PLT2DSection(self, axis, paramPLT):
        """
        Affiché la section qui a été utilisée
        """
        return False

"""
Condition d'appuis
"""
class Support:
    def __init__(self, NSupport, BoolDx, BoolDy, BoolDz, BoolMx, BoolMy, BoolMz):
        # True = unlocked and false = locked
        self.NodeSupport = NSupport
        self.Bool_DX = BoolDx
        self.Bool_DY = BoolDy
        self.Bool_DZ = BoolDz
        self.Bool_Mx = BoolMx
        self.Bool_My = BoolMy
        self.Bool_Mz = BoolMz

    @property
    def getDDl(self):
        return (self.Bool_DX, self.Bool_DY, self.Bool_DZ,
                self.Bool_Mx, self.Bool_Mx, self.Bool_Mz)

    def getBoolDx(self, BoolDx):
        self.Bool_DX = BoolDx

    def getBoolDy(self, BoolDy):
        self.Bool_DY = BoolDy

    def getBoolDz(self, BoolDz):
        self.Bool_DZ = BoolDz

    def getBoolMx(self, BoolMx):
        self.Bool_Mx = BoolMx

    def getBoolMy(self, BoolMy):
        self.Bool_My = BoolMy

    def getBoolMz(self, BoolMz):
        self.Bool_Mz = BoolMz

    @property
    def getNSupport(self):
        return self.NodeSupport

    @getNSupport.setter
    def getNSupport(self, NSupport):
        self.NodeSupport = NSupport

    def PLT2DAppui(self, axis, paramPLT):
        if axis == [1,2]:
            BoolDAbsc = self.getBoolDx
            BoolDOrdo = self.getBoolDy
            BoolM = self.getBoolMz
        elif axis == [1,3]:
            BoolDAbsc = self.getBoolDx
            BoolDOrdo = self.getBoolDz
            BoolM = self.getBoolMy
        elif axis == [2,3]:
            BoolDAbsc = self.getBoolDy
            BoolDOrdo = self.getBoolDz
            BoolM = self.getBoolMx

        if BoolDAbsc and BoolDOrdo and BoolM:
            # Extrémité libre
            a = 1
        elif BoolDAbsc and BoolDOrdo and not (BoolM):
            # Extrémité 
            a = 1
        elif BoolDAbsc and BoolDOrdo and BoolM:
            # Extrémité 
            a = 1
        elif BoolDAbsc and BoolDOrdo and BoolM:
            # Extrémité 
            a = 1
        elif BoolDAbsc and BoolDOrdo and BoolM:
            # Extrémité 
            a = 1


        return False

    def PLT3DAppui(self, axis, paramPLT):
        """
        Affiché le noeuds de différentes façons
        """
        """
        Faire avec des barres continus si ok ddl ou flèches
        sinon barre avec droite perpendiculaire si bloquer

        plusieurs flèche à la base pour dire si ok rota ou rond en 2d
        """
        return False
