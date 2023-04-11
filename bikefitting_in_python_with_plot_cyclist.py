#Copyright (c) 2009, Lior Cohen
#All rights reserved.

#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are
#met:

  #  * Redistributions of source code must retain the above copyright
  #    notice, this list of conditions and the following disclaimer.
  #  * Redistributions in binary form must reproduce the above copyright
   #   notice, this list of conditions and the following disclaimer in
   #   the documentation and/or other materials provided with the distribution

#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

from math import sin, cos, sqrt, radians
import numpy as np
import matplotlib.pyplot as plt


import numpy as np

def seatheight_calc(DlugoscUda, DlugoscPodudzia, KatWStopie, DlugoscStopy, S):

    KG_x = (sin(radians((180 - S) / 2)) * DlugoscUda)
    KG_y = 0 - (cos(radians((180 - S) / 2)) * DlugoscUda)

    SP_x = KG_x
    SP_y = KG_y - DlugoscPodudzia

    CzubekStopy_x = SP_x + (sin(radians(90 - KatWStopie)) * DlugoscStopy)
    CzubekStopy_y = SP_y - (cos(radians(90 - KatWStopie)) * DlugoscStopy)

    A = CzubekStopy_x
    B = CzubekStopy_y
    C = A**2 + B**2

    WysokoscSiedzenia = sqrt(C)

    return WysokoscSiedzenia

def WysokoscSiedzenia_2(DlugoscWspornika, KatWspornika, KierownicaX, Spadek, S, DlugoscPodudzia, DlugoscUda, SW, DlugoscStopy, KL, WysokoscSiedzenia, NS):
    Korba = KL
    Alfa = 90 - SW
    Odstęp = 2
    WysokoscWspornika = 2

    WysokoscSiedzeniaBezKorby = WysokoscSiedzenia - Korba

    Stackmax = (cos(radians(Alfa)) * WysokoscSiedzeniaBezKorby) - (-Spadek) - (DlugoscWspornika * sin(radians(KatWspornika))) - Odstęp - WysokoscWspornika

    WysokoscSztycy = 12

    Stackmin = Stackmax - WysokoscSztycy

    PrzesuniecieSiodla = NS

    Reachmax = KierownicaX - (DlugoscWspornika * sin(radians(KatWspornika))) - (PrzesuniecieSiodla)

    Reachmin = Reachmax - (DlugoscWspornika * cos(radians(KatWspornika)))

    return Stackmin, Reachmin, Reachmax, Stackmax

def stack_to_reach_output(S, OS, US, FL, OK, A, KL, H, VW, VL, AR, Innenbeinlaenge, Spacer_height):
    WinkelimFuss = 13
    tretlager_x = 30
    tretlager_y = 0
    Vorbauhoehe = 2
    Spacerhoehe = Spacer_height

    Schulterhoehe = OK
    Unterschenkel = US
    Oberschenkel = OS
    Oberkoerper = (Schulterhoehe - (Unterschenkel + Oberschenkel)) * 0.9
    Arme = A
    Fusslaenge = (FL * 0.52) * 0.77
    Kurbel = KL
    Vorbaulaenge = VL
    Vorbauwinkel = VW
    SW = 74
    Armrumpf = AR

    # Wyznaczanie współrzędnych poszczególnych punktów
    SH = seatheight_calc(Oberschenkel, Unterschenkel, WinkelimFuss, Fusslaenge, Innenbeinlaenge)

    Becken_x = tretlager_x - (np.cos(np.deg2rad(74)) * (SH - Kurbel))
    Becken_y = tretlager_y + (np.sin(np.deg2rad(74)) * (SH - Kurbel))

    Schulterx = Becken_x + np.cos(np.deg2rad(H)) * Oberkoerper
    Schultery = Becken_y + np.sin(np.deg2rad(H)) * Oberkoerper

    Lenkerx = Schulterx + (np.sin(np.deg2rad(Armrumpf + H - 90)) * (Arme * 0.8))
    Lenkery = Schultery - (np.cos(np.deg2rad(Armrumpf + H - 90)) * (Arme * 0.8))

    Arm_x = Lenkerx
    Arm_y = Lenkery

    Sitzlaenge = Lenkerx - Becken_x
    Drop = Lenkery - Becken_y

    # Polecane wartości
    # [SHa, Nachsitza, Stackmina, Reachmina, Reachmaxa, Stackmaxa, Sitzlaengea, Dropa] = recommend(...)

    # NS = Nachsitz
    # NSa = Nachsitza + NSa

    D = Drop
    SL = Sitzlaenge

    # Stack = Stackmax
    # Reach = Reachmin

    # R = (Reachmin + Reachmax) / 2
    VL = Vorbaulaenge
    VW = Vorbauwinkel

    # Obliczenia kontrolne

    KG_x = Becken_x + np.sin(np.deg2rad((180 - S) / 2)) * Oberschenkel
    KG_y = Becken_y - np.cos(np.deg2rad((180 - S) / 2)) * Oberschenkel

    SP_x = KG_x
    SP_y = KG_y - Unterschenkel

    Fussspitze_x = SP_x + np.sin(np.deg2rad(90 - WinkelimFuss)) * Fusslaenge
    Fussspitze_y = SP_y - np.cos(np.deg2rad(90 - WinkelimFuss)) * Fusslaenge

    A = Becken_x - Fussspitze_x
    B = Becken_y - Fussspitze_y
    C = A ** 2 + B ** 2
    SH = np.sqrt(C)  # Obliczenie wysokości siedzenia

    NS = np.abs(A) - np.sin(np.deg2rad(180 - 90 - 73)) * Kurbel

    Vorbau_x = Arm_x - (Vorbaulaenge * np.cos(np.deg2rad(Vorbauwinkel)))
    Vorbau_y = Arm_y - (Vorbaulaenge * np.sin(np.deg2rad(Vorbauwinkel)))

    Vorbauhoehe_x = Vorbau_x
    Vorbauhoehe_y = Vorbau_y - (Vorbauhoehe + Spacerhoehe)

    x2 = [Becken_x, KG_x, SP_x, Fussspitze_x]
    y2 = [Becken_y, KG_y, SP_y, Fussspitze_y]
    x = [tretlager_x, Becken_x, Schulterx, Arm_x]
    y = [tretlager_y, Becken_y, Schultery, Arm_y]

    R = Vorbauhoehe_x - tretlager_x
    Stack = Vorbauhoehe_y - tretlager_y

    Stackmin, Reachmin, Reachmax, Stackmax = WysokoscSiedzenia_2(Vorbaulaenge,Vorbauwinkel,Lenkerx,Drop,S,Unterschenkel,Oberschenkel,SW,Fusslaenge,KL,SH,NS)
    

    St = Stack
    St = round(St, 1)
    R = round(R, 1)
    S2R = St / R

    SH = round(SH, 1)
    NS = round(NS, 1)
    SL = round(SL, 1)
    D = round(D, 1)
    Stackmin = round(Stackmin, 1)
    Reachmin = round(Reachmin, 1)
    Reachmax = round(Reachmax, 1)
    Stackmax = round(Stackmax, 1)

    # Zwracanie wyników
    return SH, NS, SL, D, S2R, St, R, Stackmin, Reachmin, Reachmax, Stackmax, VL, VW, x, y, x2, y2

def rowerzysta(S, SH, SL, D, NS, OK, A, VL, VW, St, R, KL, H, AR, Stack_wish, Reach_wish, x, y, x2, y2, Spacer_height):
    # 4.5.1: Powierzchnia płaska ograniczona wielobokiem
    plt.close('all')

    # Rowerzysta
    # Rama
    Stack = St
    Reach = R
    tretlager_x = 30
    tretlager_y = 0
    sitzwinkel = 74
    kurbel = KL
    Vorbau = VL
    Vorbausteigung = VW
    Vorbauhoehe = 2
    Spacerhoehe = Spacer_height

    # Człowiek
    Schulterhoehe = OK
    Innenbeinlaenge = S
    oberkoerperwinkel = H
    oberkoerperlaenge = Schulterhoehe - Innenbeinlaenge
    Armrumpfwinkel = AR
    Armlaenge = A

    # Pozycja siedzenia
    sitzhoehe_neu = SH - kurbel

    # Rama
    Stack_x = tretlager_x
    Stack_y = Stack

    Reach_x = tretlager_x + Reach
    Reach_y = Stack

    # Rama życzenia
    Stack_x2 = tretlager_x
    Stack_y2 = Stack_wish

    Reach_x2 = tretlager_x + Reach_wish
    Reach_y2 = Stack_wish

    # Części dodatkowe
    Vorbau_x = x[3] - (Vorbau * np.cos(np.radians(Vorbausteigung)))
    Vorbau_y = y[3] - (Vorbau * np.sin(np.radians(Vorbausteigung)))

    Vorbauhoehe_x = Vorbau_x
    Vorbauhoehe_y = Vorbau_y - (Vorbauhoehe + Spacerhoehe)

    xx = np.array([tretlager_x, Stack_x, Reach_x])
    yy = np.array([tretlager_y, Stack_y, Reach_y])

    xx2 = np.array([tretlager_x, Stack_x2, Reach_x2])
    yy2 = np.array([tretlager_y, Stack_y2, Reach_y2])

    xxx = np.array([Vorbauhoehe_x, Vorbau_x, x[3]])
    yyy = np.array([Vorbauhoehe_y, Vorbau_y, y[3]])
    Maximum_x = max(x) + 10
    Maximum_y = max(y) + 10

    Reach_recommend = Vorbauhoehe_x - tretlager_x
    Stack_recommend = Vorbauhoehe_y - tretlager_y

    xx_2 = np.array([tretlager_x, Stack_x, Reach_recommend + tretlager_x])
    yy_2 = np.array([tretlager_y, Stack_recommend, Stack_recommend])

    h1 = plt.figure()
    ax = h1.add_subplot(1, 1, 1)

    ax.plot(x[1:], y[1:], 'b', linewidth=1)
    ax.text(x[1]+5, y[1], 'Biodro')
    ax.text(x[2], y[2]-5, 'Ramie')

    ax.plot(x[1:], y[1:], 'bo', linewidth=1)
    ax.plot(x2, y2, 'b', linewidth=1)
    ax.text(x2[1]+1, y2[1]+3, 'Kolano')
    ax.text(x2[2]+5, y2[2]+5, 'Kostka')
    ax.plot(x2, y2, 'bo', linewidth=1)
    ax.plot(xx, yy, 'g*')
    a = ax.plot(xx, yy, 'g', linewidth=1)

    ax.plot(xx2, yy2, 'm*')
    g = ax.plot(xx2, yy2, 'm', linewidth=1)
    ax.plot(xxx, yyy, 'k')
    b = ax.plot(xx_2, yy_2, 'r')
    ax.text(xx_2[0]+15, yy_2[1]+3, 'Stack')
    ax.text(xx_2[0]+1, yy_2[0]+30, 'Reach')
    ax.legend([a[0], b[0], g[0]], ['Pozycja siedzenia', 'Rama zalecana', 'Rama życzenia'], loc='upper right', bbox_to_anchor=(1.3, 1))
    ax.plot(x[3], y[3], 'bo', linewidth=1)
    ax.plot(x[1], y[1], 'b+', linewidth=1)

    ax.set_xticks(range(int(round(min(x2))), int(Maximum_x), 10))
    ax.set_yticks(range(int(round(min(y2))), int(Maximum_y), 10))
    ax.axis([0, Maximum_x, 0, Maximum_y])
    ax.grid(True)
    ax.axis('equal')
    ax.set_xlabel('Oś x [cm]')
    ax.set_ylabel('Oś y [cm]')
    plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9)
    #ax.set_position([100, 50, 560, 420])
    ax.set_title('Pozycja siedzenia na rowerze')

    plt.show()

def cosd(angle):
    return cos(radians(angle))

def sind(angle):
    return sin(radians(angle))

def bicycle_function(Sa, NSa, SWa, S, OS, US, FL, OK, A, KL, bicycle_type, riding_style):
    KatWStopie = 13
    support_x = 30
    support_y = 0
    WysokoscWspornika = 2
    WysokoscDystansu = 2

    WysokoscRamienia = OK
    DlugoscPodudzia = US
    DlugoscUda = OS
    GornaCzescCiala = (WysokoscRamienia - (DlugoscPodudzia + DlugoscUda)) * 0.9
    Rece = A
    DlugoscWewnetrznaNogi = S
    DlugoscStopy = (FL * 0.52) * 0.77
    Korba = KL

    if bicycle_type == 'RowerMiejski/Turystyczny':
        S = 145
        SW = 70
    elif bicycle_type == 'Górski':
        S = 148
        SW = 70
    elif bicycle_type == 'szosa':
        S = 150
        SW = 73

    if riding_style == 'jazda ergonomiczna':
        M = 0.53
        H = 45
        Armrumpf = 85
        DlugoscWspornika = 10
        KatWspornika = 10
        PS = 1
    elif riding_style == 'jazda sportowa':
        M = 0.54
        H = 37.5
        Armrumpf = 85
        DlugoscWspornika = 10
        KatWspornika = 10
        PS = 2
    elif riding_style == 'Aerodynamiczna':
        M = 0.53
        H = 30
        Armrumpf = 85
        DlugoscWspornika = 10
        KatWspornika = 10
        PS = 3

   
    WysokoscSiedzenia = seatheight_calc(DlugoscUda, DlugoscPodudzia, KatWStopie, DlugoscStopy, DlugoscWewnetrznaNogi)

    Miednica_x = support_x - (cosd(74) * (WysokoscSiedzenia - Korba))
    Miednica_y = support_y + (sind(74) * (WysokoscSiedzenia - Korba))

    RamieX = Miednica_x + cosd(H) * GornaCzescCiala
    RamieY = Miednica_y + sind(H) * GornaCzescCiala

    KierownicaX = RamieX + (sind(Armrumpf + H - 90) * (Rece * 0.8))
    KierownicaY = RamieY - (cosd(Armrumpf + H - 90) * (Rece * 0.8))

    Reka_x = KierownicaX
    Reka_y = KierownicaY
    DlugoscSiedzenia = KierownicaX - Miednica_x
    Spadek = KierownicaY - Miednica_y

    D = Spadek
    SL = DlugoscSiedzenia
    VL = DlugoscWspornika
    VW = KatWspornika

    KG_x = Miednica_x + (sind((180 - S) / 2) * DlugoscUda)
    KG_y = Miednica_y - (cosd((180 - S) / 2) * DlugoscUda)

    SP_x = KG_x
    SP_y = KG_y - DlugoscPodudzia
    CzubekStopy_x = SP_x + (sind(90 - KatWStopie) * DlugoscStopy)
    CzubekStopy_y = SP_y - (cosd(90 - KatWStopie) * DlugoscStopy)

    A = Miednica_x - CzubekStopy_x
    B = Miednica_y - CzubekStopy_y

    C = A**2 + B**2

    WysokoscSiedzenia = sqrt(C)
    NS = abs(A) - (sind(180 - 90 - 73) * Korba)

    Wspornik_x = Reka_x - (DlugoscWspornika * cosd(KatWspornika))
    Wspornik_y = Reka_y - (DlugoscWspornika * sind(KatWspornika))
    WysokoscWspornika_x = Wspornik_x
    WysokoscWspornika_y = Wspornik_y - (WysokoscWspornika + WysokoscDystansu)
    R = WysokoscWspornika_x - support_x
    Stack = WysokoscWspornika_y - support_y

    
    Stackmin, Reachmin, Reachmax, Stackmax = WysokoscSiedzenia_2(DlugoscWspornika, KatWspornika, KierownicaX, Spadek, S, DlugoscPodudzia, DlugoscUda, SW, DlugoscStopy, KL, WysokoscSiedzenia, NS)

    St = Stack
    St = round(St, 1)
    R = round(R, 1)
    S2R= St / R
    SH = round(WysokoscSiedzenia, 1)
    NS = round(NS, 1)
    SL = round(DlugoscSiedzenia, 1)
    D = round(Spadek, 1) 
    Stackmin = round(Stackmin, 1) 
    Reachmin = round(Reachmin, 1) 
    Reachmax = round(Reachmax, 1) 
    Stackmax = round(Stackmax, 1) 

    return SH,NS,SL,D,S2R,St,R,M,Stackmin,Reachmin,Reachmax,Stackmax,VL,VW,PS,H,Armrumpf,S

def sitzgroesse(S, KG, PS):
    SG = S / KG
    SS = None
    C = None
    S2RE = None
    ST2 = None
    RE2 = None

    if 0.48 <= SG <= 0.52:
        SG = 'Normal'
        SS = 1
    elif SG < 0.48:
        SG = 'Sitzriese'
        SS = 2
    elif SG > 0.52:
        SG = 'Langbeiner'
        SS = 3

    if SS == 1:
        if PS == 1:
            S2RE = 'STR:>1,6'
            C = 3
        elif PS == 2:
            S2RE = 'STR:1,37-1,59'
            C = 0
        elif PS == 3:
            S2RE = 'STR:<1,37'
            C = -2
    elif SS == 2:
        if PS == 1:
            S2RE = 'STR:1,37-1,59'
            C = 0
        elif PS == 2 or PS == 3:
            S2RE = 'STR:<1,37'
            C = -2
    elif SS == 3:
        if PS == 3:
            S2RE = 'STR:1,37-1,59'
            C = 0
        elif PS == 1 or PS == 2:
            S2RE = 'STR:>1,6'
            C = 3

    ST2 = (S * 0.69) + C
    if SS == 1:
        RE2 = ST2 / 1.6
    elif SS == 2:
        RE2 = ST2 / 1.48
    elif SS == 3:
        RE2 = ST2 / 1.37

    return SG, S2RE, ST2, RE2

def wysokosc_ramki(OK, S, OS):
    RL = OK - S  # Długość tułowia

    RH = S * 0.66  # Wysokość ramy

    # Określenie długości mostka w zależności od wysokości ramy
    # (no further code provided for this part)

    # Określenie długości korby w zależności od długości kroku
    KLe = OS / 2.5  # Długość korby zależna od długości uda w calach (Mestdagh, 1998)

    return RL, RH, KLe


Sa = 0

# KLIK od "Clickk Peals?"
Klik = 1 

if Klik == 1: 
    Klik = 2.3
else:
    Klik = 1

SWa = 0

#Neck or Back Pain?
RSa = 2

if RSa == 1:
    RS = 'pozycja siodełka zbyt agresywna'
    SWa = 5
else:
    RS = ''

#But Pain?
GS = 2

if GS == 1:
    GS = 'siodełko nieoptymalne'
else:
    GS = ''

#Feet Pain?
FP = 2

if FP == 1:
    FP = 'potrzebujesz wkładek do butów rowerowych'
else:
    FP = ''

#Knee Pain?
KSa = 2

if KSa == 1:
    KS = 'skonsultuj się z rowerowym dopasowaniem'
    Sa = -5
    NSa = 1
else:
    KS = ''
    Sa = 0
    NSa = 0


S = 81 #inseam
OS = 42 * 0.83#upper leg
US = 47 #lower leg
FL = 42 #shoe size 
OK = 145 #shoulder heigth 
A = 55  #arm length 
KG = 175 # body heigth

bicycle_type = 'szosa'
riding_style = 'jazda sportowa'

KL = 18 #crank length
SH1 = 90 # Saddle height
NS1 = 20 # Setback
SL1 = 40 #Seat length 
D1 = -5 #Drop 


# B = 1 more than 10 cm to ground 
#  B = 2 more than 5 cm to ground 
# B = 3 finger tips to ground 
# B = 4 palms to ground 

B = 1
if B == 1:
    B = 'choose a more upright position'
    SWa = SWa + 50
elif B == 2:
    B = 'you have to do mobility exercise'
    SWa = SWa + 45
elif B == 3:
    if RSa == 1:
        B = 'a sports like position is not recommended'
        SWa = SWa + 50
    else:
        B = 'Sports like position'
        SWa = SWa + 40
elif B == 4:
    if RSa == 1:
        B = 'an aerodynamic position is not recommended'
        SWa = SWa + 45
    else:
        B = 'aerodynamic'
        SWa = SWa + 35


RL, RH, KLe = wysokosc_ramki(OK, S, OS)

IB = S

SH,NS,SL,D,S2R,St,R,M,Stackmin,Reachmin,Reachmax,Stackmax,VL,VW,PS,H,Armrumpf,S = bicycle_function(Sa, NSa, SWa, S, OS, US, FL, OK, A, KL, bicycle_type, riding_style)

SG, S2RE, ST2, RE2 = sitzgroesse(S, KG, PS)

AR = Armrumpf

SHlemond = round((S * 0.883) + KL)

SHhamley = round(S * 1.09)

ST2 = round(ST2, 1) 

Spacer_height = 2 #W programie matlabowym było to tak zdefiniowane 

RE2 = round(RE2, 1) 

SH = Klik + SH

print("Drop:", D)
print("Seat length:", SL)
print("Seat heigth:", SH)
print("Setback:", NS)

print("Stem Length:", VL)
print("Stem Angle:", VW)
print("Frame Height:", RH)

print("Stack2Reach Index 1 kolumna:", St)
print("Stack2Reach Index 2 kolumna:", R)
print("Stack2Reach Index 3 kolumna:", S2R)
print("Torsoangle:", H)
print("Arm-Torsoangle:", AR)
print("Spacer_height:", Spacer_height)


print("Stackmin:", Stackmin)
print("Stackmax:", Stackmax)
print("Reachmin:", Reachmin)
print("Reachmax:", Reachmax)

#OPIS DZIAŁANIA SUWAKA AERODYNAMIC <> ERGONOMIC <> COMFORTABLE 
# SUWAK STERUJE WARTOSCIA H(Torsoangle) w zakresie 25.0(AERODYNAMIC) 42.5(ERGONOMIC) 60(COMFORTABLE) 

#OPIS DZIALANIA Arm-Torsoangle
# Suwak steruje wartoscia AR(Arm-Torsoangle) w zakresie 50(min) do 90(max)

#Część z wykresem człowieka
SH, NS, SL, D, S2R, St, R, Stackmin, Reachmin, Reachmax, Stackmax, VL, VW, x, y, x2, y2 = stack_to_reach_output(S, OS, US, FL, OK, A, KL, H, VW, VL, AR, IB, Spacer_height)

SH=round(SH,1)
NS=round(NS,1)
SL=round(SL,1)
D=round(D,1)
Stackmin=round(Stackmin,1)
Stackmax=round(Stackmax,1)
Reachmin=round(Reachmin,1)
Reachmax=round(Reachmax,1)
S2R=round(S2R,1)
H=round(H,1)
St=round(St,1)
R=round(R,1)

Stack_wish = 50
Reach_wish = 50
S2R_wish = None


rowerzysta(S, SH, SL, D, NS, OK, A, VL, VW, St, R, KL, H, AR, Stack_wish, Reach_wish, x, y, x2, y2, Spacer_height)

#print("St:", St)
