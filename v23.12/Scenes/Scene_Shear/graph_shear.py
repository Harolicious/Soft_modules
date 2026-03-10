#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 24 16:20:26 2024

@author: Harold
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import plotly.graph_objects as go

# plt.rc('text',usetex=True)
# # font = {'family':'serif','size':16}  # desired use
# font = {'family':'serif','size':16, 'serif': ['Computer Modern']}  # what you need to do now 

# plt.rc('font',**font)

# Cargar los datos del archivo CSV
df_opt = pd.read_csv('slider_datos_opt.csv')
df_ard = pd.read_csv('slider_datos_ard.csv')
# df_opt2 = pd.read_csv('Uniaxial_d_opt.csv')
# df_ard2 = pd.read_csv('Uniaxial_d_ard.csv')
mate = pd.read_excel('shear.xlsx')
mate_PSI = mate.iloc[:,3]
mate_PSI = 145.03773773*mate_PSI;
mate_desp = mate.iloc[:,4]
mate_PSI = mate_PSI*1.7
mate_desp = mate_desp*0.6
# Definir el número de datos para cada prueba
# Puedes ajustar estos valores según tus necesidades
tamanos_opt = [2476] * 10
tamanos_ard = [2476] * 10
# tamanos_opt2 = [148] * 10
# tamanos_ard2 = [148] * 10
# tamanos_time = [1491]

# Función para dividir y extraer solo una columna sin nombre
def dividir_y_extraer_columna(df, tamanos, indice):
    pruebas = []
    inicio = 0
    for tam in tamanos:
        prueba = df.iloc[inicio:inicio + tam, indice]
        pruebas.append(prueba)
        inicio += tam
    return pruebas

# Función para dividir en presurización y despresurización
def dividir_presurizado_despresurizado(prueba):
    max_index = len(prueba)  # Encuentra el índice del valor máximo
    valor_medio = int(max_index/2) # Encuentra el indice medio 
    presurizado = prueba[:valor_medio + 1]   # Datos de presurización
    despresurizado = prueba[valor_medio:]    # Datos de despresurización
    return presurizado, despresurizado

# Función para calcular promedio y error estándar
def calcular_estadisticas(pruebas):
    data_array = np.array(pruebas)
    promedios = np.mean(data_array, axis=0)
    errores_estandar = np.std(data_array, axis=0) / np.sqrt(data_array.shape[0])  # error estándar
    return promedios, errores_estandar

# Ajustes y extracción de datos
df_opt.iloc[:, 7] = df_opt.iloc[:, 7] * 1000
df_opt.iloc[:, 6] = df_opt.iloc[:, 6] * 1000
df_opt.iloc[:, 5] = df_opt.iloc[:, 5] * 1000

# Dividir las pruebas y extraer columnas específicas
pruebas_df_opt = dividir_y_extraer_columna(df_opt, tamanos_opt, 7)  # Desp desplazamiento x
pruebas_df_ard = dividir_y_extraer_columna(df_ard, tamanos_ard, 1)  # PSI
pruebas_df_opt2 = dividir_y_extraer_columna(df_opt, tamanos_opt, 6)  # Desp desplazamiento z
pruebas_df_ard2 = dividir_y_extraer_columna(df_ard, tamanos_ard, 1)  # PSI

pruebas_df_opt = pruebas_df_opt[1:10]
pruebas_df_ard = pruebas_df_ard[1:10]
pruebas_df_opt2 = pruebas_df_opt2[1:10]
pruebas_df_ard2 = pruebas_df_ard2[1:10]

# Dividir cada prueba en presurización y despresurización
presurizado_opt, despresurizado_opt = [], []
presurizado_ard, despresurizado_ard = [], []
presurizado_opt2, despresurizado_opt2 = [], []
presurizado_ard2, despresurizado_ard2 = [], []

for prueba in pruebas_df_opt:
    pres, desp = dividir_presurizado_despresurizado(prueba)
    presurizado_opt.append(pres)
    despresurizado_opt.append(desp)

for prueba in pruebas_df_ard:
    pres, desp = dividir_presurizado_despresurizado(prueba)
    presurizado_ard.append(pres)
    despresurizado_ard.append(desp)

for prueba in pruebas_df_opt2:
    pres, desp = dividir_presurizado_despresurizado(prueba)
    presurizado_opt2.append(pres)
    despresurizado_opt2.append(desp)

for prueba in pruebas_df_ard2:
    pres, desp = dividir_presurizado_despresurizado(prueba)
    presurizado_ard2.append(pres)
    despresurizado_ard2.append(desp)

# Calcular promedio y error estándar para presurización y despresurización
promedios_pres_opt, errores_pres_opt = calcular_estadisticas(presurizado_opt)
promedios_desp_opt, errores_desp_opt = calcular_estadisticas(despresurizado_opt)

promedios_pres_ard, errores_pres_ard = calcular_estadisticas(presurizado_ard)
promedios_desp_ard, errores_desp_ard = calcular_estadisticas(despresurizado_ard)

promedios_pres_opt2, errores_pres_opt2 = calcular_estadisticas(presurizado_opt2)
promedios_desp_opt2, errores_desp_opt2 = calcular_estadisticas(despresurizado_opt2)

promedios_pres_ard2, errores_pres_ard2 = calcular_estadisticas(presurizado_ard2)
promedios_desp_ard2, errores_desp_ard2 = calcular_estadisticas(despresurizado_ard2)

# Ajustar promedios para que el primer valor sea 0
promedios_pres_opt -= promedios_pres_opt[0]
ajuste_desp_opt = promedios_pres_opt[-1] - promedios_desp_opt[0]
promedios_desp_opt = promedios_desp_opt + ajuste_desp_opt

promedios_pres_ard -= promedios_pres_ard[0]
ajuste_desp_ard = promedios_pres_ard[-1] - promedios_desp_ard[0]
promedios_desp_ard = promedios_desp_ard + ajuste_desp_ard

promedios_pres_opt2 -= promedios_pres_opt2[0]
ajuste_desp_opt2 = promedios_pres_opt2[-1] - promedios_desp_opt2[0]
promedios_desp_opt2 = promedios_desp_opt2 + ajuste_desp_opt2

promedios_pres_ard2 -= promedios_pres_ard2[0]
ajuste_desp_ard2 = promedios_pres_ard2[-1] - promedios_desp_ard2[0]
promedios_desp_ard2 = promedios_desp_ard2 + ajuste_desp_ard2


# Suavizar los promedios utilizando savgol_filter
window_length = 17  # Ajusta el tamaño de la ventana para el suavizado
polyorder = 1      # Ajusta el orden del polinomio para el suavizado

promedios_pres_opt_smooth = savgol_filter(promedios_pres_opt, window_length, polyorder)
promedios_desp_opt_smooth = savgol_filter(promedios_desp_opt, window_length, polyorder)
promedios_pres_ard_smooth = savgol_filter(promedios_pres_ard, window_length, polyorder)
promedios_desp_ard_smooth = savgol_filter(promedios_desp_ard, window_length, polyorder)
errores_pres_opt_smooth = savgol_filter(errores_pres_opt, window_length, polyorder)
errores_desp_opt_smooth = savgol_filter(errores_desp_opt, window_length, polyorder)
errores_pres_ard_smooth = savgol_filter(errores_pres_ard, window_length, polyorder)
errores_desp_ard_smooth = savgol_filter(errores_desp_ard, window_length, polyorder)

promedios_pres_opt2_smooth = savgol_filter(promedios_pres_opt2, window_length, polyorder)
promedios_desp_opt2_smooth = savgol_filter(promedios_desp_opt2, window_length, polyorder)
promedios_pres_ard2_smooth = savgol_filter(promedios_pres_ard2, window_length, polyorder)
promedios_desp_ard2_smooth = savgol_filter(promedios_desp_ard2, window_length, polyorder)

mate_PSI = np.array(mate_PSI)
mate_desp = np.array(mate_desp)

def graficar_promedio_con_error_FEM(promedios_pres_ard, promedios_pres_opt, errores_pres_opt, promedios_desp_ard, promedios_desp_opt, errores_desp_opt):
    plt.figure(figsize=(8,6))

    plt.plot(promedios_pres_ard_smooth, promedios_pres_opt_smooth, marker=',', color='b', label='Average Angle Inflated')
    plt.fill_between(promedios_pres_ard, promedios_pres_opt - errores_pres_opt, promedios_pres_opt + errores_pres_opt, color='b', alpha=0.3)
  
    plt.plot(promedios_desp_ard_smooth, promedios_desp_opt_smooth, marker=',', color='r', label='Average Angle Deflated')
    plt.fill_between(promedios_desp_ard, promedios_desp_opt - errores_desp_opt, promedios_desp_opt + errores_desp_opt, color='r', alpha=0.3)
 
    plt.plot(mate_PSI, mate_desp, marker=',', color='black', label='FEM Mooney-Rivlin')
    plt.title('Pressure vs Displacement X - Robot Shear', size = 20)
    plt.xlabel('Pressure (PSI)', size = 20)
    plt.ylabel('Displacement X (mm)', size = 20)
    # plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.savefig("SE_of_Angle_wanted_shear.pdf", format="pdf", bbox_inches="tight", dpi=300)

def graficar_promedio_con_error(promedios_pres_ard, promedios_pres_opt, errores_pres_opt, promedios_desp_ard, promedios_desp_opt, errores_desp_opt):
    plt.figure(figsize=(10,6))

    plt.plot(promedios_pres_ard_smooth, promedios_pres_opt_smooth, marker=',', color='b', label='Average Angle Inflated')
    plt.fill_between(promedios_pres_ard, promedios_pres_opt - errores_pres_opt, promedios_pres_opt + errores_pres_opt, color='b', alpha=0.3)
  
    plt.plot(promedios_desp_ard_smooth, promedios_desp_opt_smooth, marker=',', color='r', label='Average Angle Deflated')
    plt.fill_between(promedios_desp_ard, promedios_desp_opt - errores_desp_opt, promedios_desp_opt + errores_desp_opt, color='r', alpha=0.3)
 
    plt.title('Pressure vs Displacement X - Robot Shear')
    plt.xlabel('Pressure (PSI)')
    plt.ylabel('Displacement X (mm)')
    # plt.axis('equal')
    # plt.grid(True)
    plt.legend()
    plt.savefig("SE_of_Angle_wanted_NoFEM_shear.pdf", format="pdf", bbox_inches="tight", dpi=300)


def graficar_promedio_con_error_2(promedios_pres_ard2_smooth, promedios_pres_opt2_smooth, errores_pres_opt2, promedios_desp_ard2_smooth, promedios_desp_opt2_smooth, errores_desp_opt2):
    plt.figure(figsize=(10,6))

    plt.plot(promedios_pres_ard2_smooth, promedios_pres_opt2_smooth, marker=',', color='b', label='Average Compresion Inflated')
    plt.fill_between(promedios_pres_ard2_smooth, promedios_pres_opt2_smooth - errores_pres_opt2, promedios_pres_opt2_smooth + errores_pres_opt2, color='b', alpha=0.3)
   
    plt.plot(promedios_desp_ard2_smooth, promedios_desp_opt2_smooth, marker=',', color='r', label='Average Compresion Deflated')
    plt.fill_between(promedios_desp_ard2_smooth, promedios_desp_opt2_smooth - errores_desp_opt2, promedios_desp_opt2_smooth + errores_desp_opt2, color='r', alpha=0.3)
    plt.title('Pressure vs Displacement Z - Robot Shear')
    plt.xlabel('Pressure (PSI)')
    plt.ylabel('Displacement Z (mm)')
    # plt.axis('equal')
    # plt.grid(True)
    plt.legend()
    plt.savefig("SE_of_Strain_unwanted_shear.pdf", format="pdf", bbox_inches="tight", dpi=300)
    
def graficar_promedio_con_error_3(promedios_pres_opt_smooth, promedios_pres_opt2_smooth, promedios_desp_opt_smooth, promedios_desp_opt2_smooth):
    plt.figure(figsize=(10,6))

    plt.plot(promedios_pres_opt_smooth, promedios_pres_opt2_smooth, marker=',', color='b', label='Average Compresion Inflated')
    plt.fill_between(promedios_pres_opt_smooth, promedios_pres_opt2_smooth - errores_pres_opt2, promedios_pres_opt2_smooth + errores_pres_opt2, color='b', alpha=0.3)
   
    plt.plot(promedios_desp_opt_smooth, promedios_desp_opt2_smooth, marker=',', color='r', label='Average Compresion Deflated')
    plt.fill_between(promedios_desp_opt_smooth, promedios_desp_opt2_smooth - errores_desp_opt2, promedios_desp_opt2_smooth + errores_desp_opt2, color='r', alpha=0.3)
    plt.title('Displacement Z vs Displacement X - Robot Shear')
    plt.xlabel('Displacement X (mm)')
    plt.ylabel('Displacement Z (mm)')
    # plt.axis('equal')
    # plt.grid(True)
    plt.legend()
    plt.savefig("SE_of_Strain_unwanted_shear_XZ.pdf", format="pdf", bbox_inches="tight", dpi=300)


graficar_promedio_con_error(promedios_pres_ard_smooth, promedios_pres_opt_smooth, errores_pres_opt, promedios_desp_ard_smooth, promedios_desp_opt_smooth, errores_desp_opt)
graficar_promedio_con_error_2(promedios_pres_ard2_smooth, promedios_pres_opt2_smooth, errores_pres_opt2, promedios_desp_ard2_smooth, promedios_desp_opt2_smooth, errores_desp_opt2)
# graficar_promedio_con_error_FEM(promedios_pres_ard_smooth, promedios_pres_opt_smooth, errores_pres_opt, promedios_desp_ard_smooth, promedios_desp_opt_smooth, errores_desp_opt)
# graficar_promedio_con_error_3(promedios_pres_opt_smooth, promedios_pres_opt2_smooth, promedios_desp_opt_smooth, promedios_desp_opt2_smooth)


