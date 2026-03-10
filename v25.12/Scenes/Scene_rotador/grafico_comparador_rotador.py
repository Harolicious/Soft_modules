#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  5 17:58:21 2026

@author: lab
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# =====================================================
# CARGA DE DATOS EXPERIMENTALES
# =====================================================
df_opt = pd.read_csv('rotate_datos_opt2.csv')
df_ard = pd.read_csv('rotate_datos_ard.csv')

# FEM Mooney-Rivlin
mate = pd.read_excel('rotator.xlsx')
mate_PSI = 145.03773773 * mate.iloc[:, 3].values
mate_desp = mate.iloc[:, 4].values

# Conversión a mm
df_opt.iloc[:, 6] *= 1000  # Z
df_opt.iloc[:, 5] *= 1000  # X

# =====================================================
# PARÁMETROS
# =====================================================
tamanos_opt = [2476] * 10
tamanos_ard = [2476] * 10

# =====================================================
# FUNCIONES
# =====================================================
def dividir_y_extraer_columna(df, tamanos, indice):
    pruebas = []
    inicio = 0
    for tam in tamanos:
        pruebas.append(df.iloc[inicio:inicio+tam, indice])
        inicio += tam
    return pruebas

def dividir_presurizado_despresurizado(prueba):
    mid = int(len(prueba)/2)
    return prueba[:mid], prueba[mid:]

def calcular_estadisticas(pruebas):
    data = np.array(pruebas)
    mean = np.mean(data, axis=0)
    se = np.std(data, axis=0) / np.sqrt(data.shape[0])
    return mean, se

# =====================================================
# EXTRACCIÓN DE PRUEBAS
# =====================================================
pruebas_opt_Z = dividir_y_extraer_columna(df_opt, tamanos_opt, 6)[3:10]
pruebas_opt_Angle = dividir_y_extraer_columna(df_opt, tamanos_opt, 3)[3:10]
pruebas_ard = dividir_y_extraer_columna(df_ard, tamanos_ard, 1)[3:10]

# =====================================================
# PRESURIZADO / DESPRESURIZADO
# =====================================================
def separar(pruebas):
    up, down = [], []
    for p in pruebas:
        a, b = dividir_presurizado_despresurizado(p)
        up.append(a)
        down.append(b)
    return up, down

optZ_up, optZ_down = separar(pruebas_opt_Z)
opt_Angle_up, opt_Angle_down = separar(pruebas_opt_Angle)
ard_up, ard_down = separar(pruebas_ard)

# =====================================================
# PROMEDIOS Y ERRORES
# =====================================================
Z_up_m, Z_up_e = calcular_estadisticas(optZ_up)
Z_down_m, Z_down_e = calcular_estadisticas(optZ_down)

Angle_up_m, Angle_up_e = calcular_estadisticas(opt_Angle_up)
Angle_down_m, Angle_down_e = calcular_estadisticas(opt_Angle_down)

P_up_m, _ = calcular_estadisticas(ard_up)
P_down_m, _ = calcular_estadisticas(ard_down)

# Ajuste a cero
Z_up_m -= Z_up_m[0]
Angle_up_m -= Angle_up_m[0]
P_up_m -= P_up_m[0]

Z_down_m += Z_up_m[-1] - Z_down_m[0]
Angle_down_m += Angle_up_m[-1] - Angle_down_m[0]
P_down_m += P_up_m[-1] - P_down_m[0]

# =====================================================
# SUAVIZADO
# =====================================================
win = 17
poly = 1

Z_up_m = savgol_filter(Z_up_m, win, poly)
Z_down_m = savgol_filter(Z_down_m, win, poly)
Angle_up_m = savgol_filter(Angle_up_m, win, poly)
Angle_down_m = savgol_filter(Angle_down_m, win, poly)

P_up_m = savgol_filter(P_up_m, win, poly)
P_down_m = savgol_filter(P_down_m, win, poly)

Z_up_e = savgol_filter(Z_up_e, win, poly)
Z_down_e = savgol_filter(Z_down_e, win, poly)
Angle_up_e = savgol_filter(Angle_up_e, win, poly)
Angle_down_e = savgol_filter(Angle_down_e, win, poly)

# =====================================================
# CARGA DATOS SOFA
# =====================================================
df_sofa = pd.read_csv('end_effector_data_Rotador_YMA.csv')

Pressure_PSI = df_sofa['Pressure'].values / 6.89
z_sofa = df_sofa['Position_Y'].values
angle = df_sofa['Angle'].values

z_sofa -= z_sofa[0]
angle -= angle[0]

mid = int(len(Pressure_PSI)/2)

P_sofa_up = Pressure_PSI[:mid]
P_sofa_down = Pressure_PSI[mid:]

z_up_sofa = z_sofa[:mid]
z_down_sofa = z_sofa[mid:]

angle_up_sofa = -angle[:mid]
angle_down_sofa = -angle[mid:]

# =====================================================
# GRAFICO Z (EXP vs SOFA)
# =====================================================
plt.figure(figsize=(10,6))

plt.plot(P_up_m, Z_up_m, color='blue', label='Exp Z Inflated')
plt.fill_between(P_up_m, Z_up_m-Z_up_e, Z_up_m+Z_up_e, alpha=0.3)

plt.plot(P_down_m, Z_down_m, color='red', label='Exp Z Deflated')

plt.plot(P_sofa_up, z_up_sofa, '--k', label='SOFA Z Inflated')
plt.plot(P_sofa_down, z_down_sofa, ':k', label='SOFA Z Deflated')

plt.xlabel('Pressure (PSI)')
plt.ylabel('Displacement Z (mm)')
plt.title('Pressure vs Displacement Z – Experimental vs SOFA')
plt.legend()
plt.grid(True)

plt.savefig('Z_Exp_vs_SOFA_biaxial.pdf', dpi=300, bbox_inches='tight')
plt.show()

# =====================================================
# GRAFICO X (EXP vs SOFA vs FEM MR)
# =====================================================
plt.figure(figsize=(10,6))

plt.plot(P_up_m, Angle_up_m, color='blue', label='Exp X Inflated')
plt.fill_between(P_up_m, Angle_up_m-Angle_up_e, Angle_up_m+Angle_up_e, alpha=0.3)

plt.plot(P_down_m, Angle_down_m, color='red', label='Exp X Deflated')

plt.plot(P_sofa_up, angle_up_sofa, '--k', label='SOFA X Inflated')
plt.plot(P_sofa_down, angle_down_sofa, ':k', label='SOFA X Deflated')

# FEM Mooney-Rivlin
# plt.plot(mate_PSI, mate_desp, 'k', linewidth=2, label='FEM Mooney–Rivlin')

plt.xlabel('Pressure (PSI)')
plt.ylabel('Angle (Degree)')
plt.title('Pressure vs Angle – Experimental vs SOFA')
plt.legend()
plt.grid(True)

plt.savefig('X_Exp_vs_SOFA_vs_FEM_biaxial.pdf', dpi=300, bbox_inches='tight')
plt.show()

