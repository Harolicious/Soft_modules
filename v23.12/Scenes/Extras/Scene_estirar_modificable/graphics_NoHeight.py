#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# Lista de archivos CSV
files = [f'end_effector_data_Estirar_NoHeight_{i}.csv' for i in range(1, 18)]

# Generar colores en degradé
num_files = len(files)
colors = cm.viridis(np.linspace(0, 1, num_files))

# Configuración de la figura
plt.figure(figsize=(10, 6))

# Iterar sobre los archivos y graficar
for idx, file in enumerate(files):
    i = 1 + idx  # Número del archivo (1 a 18)
    color = colors[idx]  # Color en degradé para el archivo actual
    
    # Cargar el archivo
    df = pd.read_csv(file)
    
    # Variables
    Pressure = df['Pressure'].values
    y1 = df['P1_Position_Y'].values
    
    # Ajustes
    Pressure_PSI = Pressure / 6.89
    y1_ad = y1 - y1[0]
    # mid = len(Pressure_PSI) // 2

    # Separar en fases
    # y1_ad_up = y1_ad[:mid]
    # y1_ad_down = y1_ad[mid:]
    # Pressure_PSI_up = Pressure_PSI[:mid]
    # Pressure_PSI_down = Pressure_PSI[mid:]

    # Agregar curvas al gráfico
    # plt.plot(Pressure_PSI_up, y1_ad_up, marker=',', label=f'{file} Inflated')
    # plt.plot(Pressure_PSI_down, y1_ad_down, marker=',', linestyle='--', label=f'{file} Deflated')
    
    plt.plot(Pressure_PSI, y1_ad, color=color, marker=',', label=f'Altura del Núcleo = {i}')

# Personalización del gráfico
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')
plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre desplazamiento y presión')
plt.grid(True)
plt.tight_layout()  # Ajustar layout para evitar solapamientos
plt.savefig('Displacement_Pressure_NoHeight.png', dpi=300)
plt.show()

# Configuración de la figura
plt.figure(figsize=(10, 6))

# Iterar sobre los archivos y graficar
for idx, file in enumerate(files):
    i = 1 + idx  # Número del archivo (1 a 18)
    color = colors[idx]  # Color en degradé para el archivo actual
    
    # Cargar el archivo
    df = pd.read_csv(file)
    
    # Variables
    Pressure = df['Pressure'].values
    y1 = df['P2_Position_Z'].values
    
    # Ajustes
    Pressure_PSI = Pressure / 6.89
    y1_ad = y1 - y1[0]
    # mid = len(Pressure_PSI) // 2

    # Separar en fases
    # y1_ad_up = y1_ad[:mid]
    # y1_ad_down = y1_ad[mid:]
    # Pressure_PSI_up = Pressure_PSI[:mid]
    # Pressure_PSI_down = Pressure_PSI[mid:]

    # Agregar curvas al gráfico
    # plt.plot(Pressure_PSI_up, y1_ad_up, marker=',', label=f'{file} Inflated')
    # plt.plot(Pressure_PSI_down, y1_ad_down, marker=',', linestyle='--', label=f'{file} Deflated')
    
    plt.plot(Pressure_PSI, y1_ad, color=color, marker=',', label=f'Altura del Núcleo = {i}')

# Personalización del gráfico
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')
plt.ylabel('Axis Y (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre desplazamiento y presión')
plt.grid(True)
plt.tight_layout()  # Ajustar layout para evitar solapamientos
plt.savefig('UW_Displacement_Pressure_NoHeight.png', dpi=300)
plt.show()
