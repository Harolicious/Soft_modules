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

plt.rc('text',usetex=True)
# font = {'family':'serif','size':16}  # desired use
font = {'family':'serif','size':16, 'serif': ['Computer Modern']}  # what you need to do now 

plt.rc('font',**font)


# Cargar los datos del archivo CSV
df_opt = pd.read_csv('plataforma_xz.csv')

# Definir el número de datos para cada prueba
rangos_opt = [(237,4237), (4237,8273), (8281,12281), (12312,16312), (16324,20324), 
              (20352,24352), (24371,28371), (28396,32396), (32427,36427), (36643,40643)  
              ]

# Ajusta la función para dividir y extraer por rangos
def dividir_y_extraer_columna(df, rangos, indice):
    pruebas = []
    for inicio, fin in rangos:
        prueba = df.iloc[inicio:fin, indice]
        pruebas.append(prueba)
    return pruebas


# Función para calcular promedio y error estándar
def calcular_estadisticas(pruebas):
    data_array = np.array(pruebas)
    promedios = np.mean(data_array, axis=0)
    errores_estandar = np.std(data_array, axis=0)# error estándar
    return promedios, errores_estandar

# Ajustes y extracción de datos

df_opt.iloc[:, 6] = df_opt.iloc[:, 6] * -1000 
df_opt.iloc[:, 7] = df_opt.iloc[:, 7] * 1000 
# df_opt.iloc[:, 7] = df_opt.iloc[:, 7] * 1000 

# Dividir las pruebas y extraer columnas específicas
pruebas_df_opt = dividir_y_extraer_columna(df_opt, rangos_opt, 6)  # Desp X
pruebas_df_opt2 = dividir_y_extraer_columna(df_opt, rangos_opt, 7)  # Desp Y

pruebas_df_opt = pruebas_df_opt[2:9]
pruebas_df_opt2 = pruebas_df_opt2[2:9]


# Calcular promedio y error estándar para presurización y despresurización
promedios_pres_opt, errores_pres_opt = calcular_estadisticas(pruebas_df_opt)
promedios_pres_opt2, errores_pres_opt2 = calcular_estadisticas(pruebas_df_opt2)

# # Ajustar promedios para que el primer valor sea 0
promedios_pres_opt -= promedios_pres_opt[0]
promedios_pres_opt2 -= promedios_pres_opt2[0]

# # Suavizar datos
# window_length, polyorder = 7, 1
# promedios_pres_opt_smooth = savgol_filter(promedios_pres_opt, window_length, polyorder)
# promedios_pres_opt2_smooth = savgol_filter(promedios_pres_opt2, window_length, polyorder)

# # graficar cuadrado 2D
m= 3
n= 1.5
Cuadr_x = [0, n, n, 0, 0]
Cuadr_y = [0, 0, m, m, 0]

def graficar_promedio_con_error(promedios_pres_opt2,promedios_pres_opt, errores_pres_opt, errores_pres_opt2):
    plt.figure(figsize=(8,6))

    plt.plot(promedios_pres_opt, promedios_pres_opt2, marker=',', color='b', label='Average \n Trajectory \n Obtained')
    plt.fill_between(promedios_pres_opt, 
                     promedios_pres_opt2 - errores_pres_opt2, 
                     promedios_pres_opt2 + errores_pres_opt2, 
                     color='r', label='Standar Error' , alpha=0.3)
    plt.fill_betweenx(promedios_pres_opt2,
                      promedios_pres_opt - errores_pres_opt,
                      promedios_pres_opt + errores_pres_opt,
                      color='r', alpha=0.3)
    
    plt.plot(Cuadr_x, Cuadr_y, marker='.', color='green', label='Target Route')
    plt.title('Displacement X vs Displacement Z',size = 20)
    plt.xlabel('Displacement X (mm)',size = 20)
    plt.ylabel('Displacement Z (mm)',size = 20)
    plt.axis('equal')
    plt.grid(True)
    plt.legend(loc = 4)
    plt.savefig("SE_of_Strain_wanted_XZ.pdf", format="pdf", bbox_inches="tight", dpi=300)
    plt.show()

graficar_promedio_con_error(promedios_pres_opt2 ,promedios_pres_opt , errores_pres_opt, errores_pres_opt2)


def graficar_datos(pruebas_df_opt, pruebas_df_opt2):
    plt.figure(figsize=(8,6))
    
    plt.plot(pruebas_df_opt, pruebas_df_opt2, marker=',')
    plt.title('Displacement X vs Displacement Z',size = 20)
    plt.xlabel('Displacement X (mm)',size = 20)
    plt.ylabel('Displacement Z (mm)',size = 20)
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    # plt.savefig("SE_of_Strain_wanted_XZ.pdf", format="pdf", bbox_inches="tight", dpi=300)
    plt.show() 
    
# n=5
# graficar_datos(pruebas_df_opt[0:n], pruebas_df_opt2[0:n])
graficar_datos(promedios_pres_opt, promedios_pres_opt2)