#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 21 17:38:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Acordeon.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x = df['Position_X'].values
y = df['Position_Y'].values
z = df['Position_Z'].values
Angle = df['Angle'].values


y_ad = y - y[0]
Pressure_PSI = Pressure/6.89
Neg_Angle = -Angle

mid = int(len(Time)/2)
Time_up = Time[:mid]
Time_down = Time[mid:]
y_ad_up= y_ad[:mid]
y_ad_down = y_ad[mid:]
Pressure_PSI_up = Pressure_PSI[:mid]
Pressure_PSI_down = Pressure_PSI[mid:]
Neg_Angle_up = Neg_Angle[:mid]
Neg_Angle_down = Neg_Angle[mid:]


plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, y_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, y_ad_down , marker=',', color='green')
plt.legend(['Strain Z Inflated', 'Strain Z Deflated'])

plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre el eje Z y la presión')

plt.savefig('despla_Z_presion_tilt.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, Neg_Angle_up, marker=',' ,color='blue')
plt.plot(Pressure_PSI_down, Neg_Angle_down, marker=',', color='green')
plt.legend(['Angle Inflated', 'Angle Deflated'])

plt.ylabel('Angle (degrees)')
plt.xlabel('Pressure (PSI)')
plt.title('Inclinación angular del módulo')

plt.savefig('Angle_presion_tilt.png', dpi=300)

plt.grid(True)
plt.show()

