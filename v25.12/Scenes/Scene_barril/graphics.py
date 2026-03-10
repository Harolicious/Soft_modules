#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Barril_YMA.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x1 = df['P1_Position_X'].values
y1 = df['P1_Position_Y'].values
z1 = df['P1_Position_Z'].values
x2 = df['P2_Position_X'].values
y2 = df['P2_Position_Y'].values
z2 = df['P2_Position_Z'].values

z2_ad = z2 - z2[0] 
y1_ad = y1 - y1[0] 
Pressure_PSI = Pressure/6.89

mid = int(len(Time)/2)
z2_ad_up= z2_ad[:mid]
z2_ad_down = z2_ad[mid:]
y1_ad_up= y1_ad[:mid]
y1_ad_down = y1_ad[mid:]
Pressure_PSI_up = Pressure_PSI[:mid]
Pressure_PSI_down = Pressure_PSI[mid:]

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, y1_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, y1_ad_down , marker=',', color='green')
plt.legend(['Strain Z Inflated', 'Strain Z Deflated'])

plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre el eje Z y la presión')

plt.savefig('despla_Z_presion_biaxial.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, z2_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, z2_ad_down , marker=',', color='green')
plt.legend(['Strain X Inflated', 'Strain X Deflated'])

plt.ylabel('Axis X (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Desplazamiento Biaxial')

plt.savefig('despla_X_presion_biaxial.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Time, Pressure_PSI, marker=',')


plt.grid(True)
plt.show()
