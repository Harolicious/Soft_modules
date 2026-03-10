#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:37:34 2026

@author: lab
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# ==========================
# CARGA DE DATOS
# ==========================
df_opt = pd.read_csv('estirar_datos_opt3.csv')
df_ard = pd.read_csv('estirar_datos_ard3.csv')
mate = pd.read_excel('estirardsm10.xlsx')

# ==========================
# FEM
# ==========================
mate_PSI = mate.iloc[:, 3] * 145.03773773
mate_desp = -mate.iloc[:, 4] + mate_PSI * 1.9

mate_PSI = np.array(mate_PSI)
mate_desp = np.array(mate_desp)

# ==========================
# SIMULACIÓN YMA
# ==========================
df_sim = pd.read_csv('end_effector_data_Estirar_YMA.csv')

Pressure_sim = df_sim['Pressure'].values / 6.89
y1 = df_sim['P1_Position_Y'].values   # Z
z2 = df_sim['P2_Position_Z'].values   # X

y1_ad = y1 - y1[0]
z2_ad = z2 - z2[0]

mid_sim = int(len(Pressure_sim) / 2)

P_up = Pressure_sim[:mid_sim]
P_down = Pressure_sim[mid_sim:]

Z_up = y1_ad[:mid_sim]
Z_down = y1_ad[mid_sim:]

X_up = z2_ad[:mid_sim]
X_down = z2_ad[mid_sim:]

# ==========================
# FUNCIONES
# ==========================
def dividir_y_extraer_columna(df, tamanos, indice):
    pruebas = []
    inicio = 0
    for t in tamanos:
        pruebas.append(df.iloc[inicio:inicio + t, indice])
        inicio += t
    return pruebas

def dividir_pres_desp(prueba):
    mid = int(len(prueba) / 2)
    return prueba[:mid], prueba[mid:]

def estadisticas(pruebas):
    data = np.array(pruebas)
    mean = np.mean(data, axis=0)
    se = np.std(data, axis=0) / np.sqrt(data.shape[0])
    return mean, se

# ==========================
# PREPROCESAMIENTO
# ==========================
df_opt.iloc[:, 6] *= 1000   # Z
df_opt.iloc[:, 5] *= 1000   # X

tamanos = [2476] * 10

# Deseada (Z)
opt_Z = dividir_y_extraer_columna(df_opt, tamanos, 6)[1:9]
ard_Z = dividir_y_extraer_columna(df_ard, tamanos, 1)[1:9]

# No deseada (X)
opt_X = dividir_y_extraer_columna(df_opt, tamanos, 5)[1:9]
ard_X = dividir_y_extraer_columna(df_ard, tamanos, 1)[1:9]

# ==========================
# DIVISIÓN PRES / DESP
# ==========================
pZ_opt, dZ_opt, pZ_ard, dZ_ard = [], [], [], []
pX_opt, dX_opt, pX_ard, dX_ard = [], [], [], []

for oZ, aZ, oX, aX in zip(opt_Z, ard_Z, opt_X, ard_X):
    p, d = dividir_pres_desp(oZ)
    pZ_opt.append(p)
    dZ_opt.append(d)

    p, d = dividir_pres_desp(aZ)
    pZ_ard.append(p)
    dZ_ard.append(d)

    p, d = dividir_pres_desp(oX)
    pX_opt.append(p)
    dX_opt.append(d)

    p, d = dividir_pres_desp(aX)
    pX_ard.append(p)
    dX_ard.append(d)

# ==========================
# PROMEDIOS Y ERRORES
# ==========================
mZ_p, seZ_p = estadisticas(pZ_opt)
mZ_d, seZ_d = estadisticas(dZ_opt)
pZ_p, _ = estadisticas(pZ_ard)
pZ_d, _ = estadisticas(dZ_ard)

mX_p, seX_p = estadisticas(pX_opt)
mX_d, seX_d = estadisticas(dX_opt)
pX_p, _ = estadisticas(pX_ard)
pX_d, _ = estadisticas(dX_ard)

# Ajuste a cero
for m_p, m_d, p_p, p_d in [(mZ_p, mZ_d, pZ_p, pZ_d),
                           (mX_p, mX_d, pX_p, pX_d)]:
    m_p -= m_p[0]
    m_d += m_p[-1] - m_d[0]
    p_p -= p_p[0]
    p_d += p_p[-1] - p_d[0]

# ==========================
# SUAVIZADO
# ==========================
wl, po = 15, 1

def smooth(*arrays):
    return [savgol_filter(a, wl, po) for a in arrays]

mZ_p, mZ_d, pZ_p, pZ_d, seZ_p, seZ_d = smooth(mZ_p, mZ_d, pZ_p, pZ_d, seZ_p, seZ_d)
mX_p, mX_d, pX_p, pX_d, seX_p, seX_d = smooth(mX_p, mX_d, pX_p, pX_d, seX_p, seX_d)

# ==========================
# GRÁFICO Z (DESEADA)
# ==========================
plt.figure(figsize=(10,6))

plt.plot(pZ_p, mZ_p, 'b', label='Experimental Inflated')
plt.fill_between(pZ_p, mZ_p-seZ_p, mZ_p+seZ_p, color='b', alpha=0.3)

plt.plot(pZ_d, mZ_d, 'r', label='Experimental Deflated')
plt.fill_between(pZ_d, mZ_d-seZ_d, mZ_d+seZ_d, color='r', alpha=0.3)

plt.plot(mate_PSI, mate_desp, 'k', lw=2, label='FEM Mooney-Rivlin')
plt.plot(P_up, Z_up, 'k--', lw=2, label='Simulation YMA Inflated')
plt.plot(P_down, Z_down, 'k:', lw=2, label='Simulation YMA Deflated')

plt.xlabel('Pressure (PSI)')
plt.ylabel('Displacement Z (mm)')
plt.title('Desired Deformation (Z)')
plt.grid(True)
plt.legend()
plt.savefig('Z_desired.pdf', dpi=300, bbox_inches='tight')

# ==========================
# GRÁFICO X (NO DESEADA)
# ==========================
plt.figure(figsize=(10,6))

plt.plot(pX_p, mX_p, 'b', label='Experimental Inflated')
plt.fill_between(pX_p, mX_p-seX_p, mX_p+seX_p, color='b', alpha=0.3)

plt.plot(pX_d, mX_d, 'r', label='Experimental Deflated')
plt.fill_between(pX_d, mX_d-seX_d, mX_d+seX_d, color='r', alpha=0.3)

plt.plot(P_up, X_up, 'k--', lw=2, label='Simulation YMA Inflated')
plt.plot(P_down, X_down, 'k:', lw=2, label='Simulation YMA Deflated')

plt.xlabel('Pressure (PSI)')
plt.ylabel('Displacement X (mm)')
plt.title('Undesired Deformation (X)')
plt.grid(True)
plt.legend()
plt.savefig('X_undesired.pdf', dpi=300, bbox_inches='tight')

plt.show()
