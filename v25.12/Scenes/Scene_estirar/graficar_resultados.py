import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.interpolate import interp1d

# ── Cargar datos ──────────────────────────────────────────────────────────────
df   = pd.read_csv("end_effector_data_Estirar_inverso_lista.csv")
mate = pd.read_excel("estirardsm10_c.xlsx", header=0)

fem_P = mate.iloc[:,3].values * 6.89476  # PSI to kPa
fem_D = mate.iloc[:,4].values

p1_y0 = df['P1_Position_Y'].iloc[0]
mid   = len(df) // 2
up    = df.iloc[:mid]
dn    = df.iloc[mid:]

up_desp = (up['P1_Position_Y'] - p1_y0).values
dn_desp = (dn['P1_Position_Y'] - p1_y0).values
up_pres = up['SPA_Pressure_calculated'].values
dn_pres = dn['SPA_Pressure_calculated'].values
up_ym   = up['YM'].values
dn_ym   = dn['YM'].values
up_goal = up['Goal_Desp'].values
dn_goal = dn['Goal_Desp'].values

# ── Figura con dos subplots ───────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 6))
gs  = gridspec.GridSpec(1, 2, figure=fig, wspace=0.35)

# ── Subplot 1: Presión vs Desplazamiento ─────────────────────────────────────
ax1 = fig.add_subplot(gs[0])
ax1.plot(fem_P, fem_D, color='#534AB7', linewidth=2.5, label='FEM real')
ax1.plot(up_pres, up_desp, color='#1D9E75', linewidth=2, linestyle='--', label='SOFA subida')
ax1.plot(dn_pres, dn_desp, color='#D85A30', linewidth=2, linestyle=':', label='SOFA bajada')
ax1.set_xlabel('Presión (kPa)', fontsize=12)
ax1.set_ylabel('Desplazamiento P1 (mm)', fontsize=12)
ax1.set_title('Presión vs Desplazamiento', fontsize=13, fontweight='bold')
ax1.legend(fontsize=10)
ax1.set_xlim(0, 55)
ax1.set_ylim(0, 8.5)
ax1.grid(True, alpha=0.2)
ax1.set_xticks([0, 10, 20, 30, 40, 50])

# ── Subplot 2: Evolución del YM ──────────────────────────────────────────────
ax2 = fig.add_subplot(gs[1])
ax2.plot(up_goal, up_ym, color='#534AB7', linewidth=2, label='Subida')
ax2.plot(dn_goal, dn_ym, color='#D85A30', linewidth=2, linestyle=':', label='Bajada')

# Marcar los tramos

pressure_ranges = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]

YM_values       = [28000, 24000, 20000, 16000, 13000, 11000, 10000, 10000, 9000, 8000]

goal_targets    = [r / 50 * 7.67 for r in pressure_ranges]


for i, (g, ym) in enumerate(zip(goal_targets[:-1], YM_values)):
    ax2.axhline(y=ym, color='gray', linewidth=0.8, linestyle='--', alpha=0.5)
    ax2.axvline(x=g, color='gray', linewidth=0.8, linestyle='--', alpha=0.5)

ax2.set_xlabel('Goal Desp (mm)', fontsize=12)
ax2.set_ylabel('Young Modulus (Pa)', fontsize=12)
ax2.set_title('Evolución del YM durante la simulación', fontsize=13, fontweight='bold')
ax2.legend(fontsize=10)
ax2.set_xlim(0, 7.67)
ax2.grid(True, alpha=0.2)

plt.suptitle('Simulación SOFA — Modelo hiperelástico por tramos', 
             fontsize=14, fontweight='bold', y=1.02)

plt.savefig('resultados_simulacion.png', dpi=150, bbox_inches='tight')
print("Gráfico guardado en: resultados_simulacion.png")
plt.show()
