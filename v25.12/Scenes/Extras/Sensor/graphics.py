#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Rotador_45.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x = df['Position_X'].values
y = df['Position_Y'].values
z = df['Position_Z'].values
Angle = df['Angle'].values

Angle = -Angle
Pressure_PSI = Pressure/6.89
y_ad = y - y[0]

mid = int(len(Time)/2)
y_ad_up= y_ad[:mid]
y_ad_down = y_ad[mid:]
Angle_up= Angle[:mid]
Angle_down = Angle[mid:]
Pressure_PSI_up = Pressure_PSI[:mid]
Pressure_PSI_down = Pressure_PSI[mid:]

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, Angle_up, marker=',', color='blue')
plt.plot(Pressure_PSI_down, Angle_down, marker=',', color='green')
plt.legend(['Angle Inflated', 'Angle Deflated'])

plt.ylabel('Angle (degrees)')
plt.xlabel('Pressure (PSI)')
plt.title('Rotación del módulo variando la presión')

plt.savefig('Angulo_presion_rotator_45.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, y_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, y_ad_down , marker=',', color='green')
plt.legend(['Strain Z Inflated', 'Strain Z Deflated'])

plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Desplazamiento en Z variando la presión')

plt.savefig('desplazamiento_presion_rotator_45.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Time, Pressure_PSI, marker=',')

plt.xlabel('Time (s)')
plt.ylabel('Pressure (PSI)')
plt.title('Presión a lo largo de la simulación')

plt.savefig('Pressure_time_rotator.png', dpi=300)

plt.grid(True)
plt.show()