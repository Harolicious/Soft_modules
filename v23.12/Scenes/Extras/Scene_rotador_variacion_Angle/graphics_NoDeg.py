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
files = [f'end_effector_data_Rotador_No_Deg_{i}.csv' for i in range(0, 86, 5)]

# Generar colores en degradé
num_files = len(files)
colors = cm.viridis(np.linspace(0, 1, num_files))  # Cambiar mapa de colores si es necesario

# Gráfico 1: Relación entre ángulo y presión
plt.figure(figsize=(10, 6))
for idx, file in enumerate(files):
    i = idx * 5  # Número del archivo
    color = colors[idx]
    
    # Cargar datos
    df = pd.read_csv(file)
    Pressure = df['Pressure'].values
    Angle = -df['Angle'].values  # Invertir el ángulo
    
    # Convertir presión a PSI
    Pressure_PSI = Pressure / 6.89
    
    # Graficar ángulo vs presión
    plt.plot(Pressure_PSI, Angle, color=color, label=f'Valor Angular = {i}')
    
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='x-small')
plt.ylabel('Angle (Degree)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre la Rotación y Presión')
plt.grid(True)
plt.tight_layout()
plt.savefig('Angle_vs_Pressure_NoDeg_Rot.png', dpi=300)
plt.show()

# Gráfico 2: Relación entre desplazamiento y presión
plt.figure(figsize=(10, 6))
for idx, file in enumerate(files):
    i = idx * 5  # Número del archivo
    color = colors[idx]
    
    # Cargar datos
    df = pd.read_csv(file)
    Pressure = df['Pressure'].values
    Position_Y = df['Position_Y'].values - df['Position_Y'].iloc[0]  # Normalizar desplazamiento
    
    # Convertir presión a PSI
    Pressure_PSI = Pressure / 6.89
    
    # Graficar desplazamiento vs presión
    plt.plot(Pressure_PSI, Position_Y, color=color, label=f'Valor Angular = {i}')
    
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='x-small')
plt.ylabel('Axis Z (mm)')  # Cambiar etiqueta si corresponde
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre Desplazamiento y Presión')
plt.grid(True)
plt.tight_layout()
plt.savefig('UW_Displacement_vs_Pressure_NoDeg_Rot.png', dpi=300)
plt.show()

