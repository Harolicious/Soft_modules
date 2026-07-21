"""
Grafica la comparación entre la simulación SOFA (scene_estirar_inverso_lista.py)
y los datos experimentales extraídos de estirardsm10_c.xlsx.

Uso:
    python3 graficar_comparacion.py

Requiere que existan en la misma carpeta:
    - end_effector_data_Estirar_inverso_lista.csv   (generado por la escena SOFA)
    - presion_desplazamiento_kpa.csv                (datos experimentales)
"""
import csv
import os
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_CSV  = os.path.join(BASE_DIR, "end_effector_data_Estirar_inverso_lista_40.csv")
EXP_CSV  = os.path.join(BASE_DIR, "presion_desplazamiento_kpa.csv")


def cargar_simulacion(path):
    tiempo, desp, p_sim, p_exp, ym = [], [], [], [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            tiempo.append(float(row["Time"]))
            desp.append(float(row["Goal_Desp"]))
            p_sim.append(float(row["SPA_Pressure_calculated"]))
            p_exp.append(float(row["Presion_experimental_kPa"]))
            ym.append(float(row["YM"]))
    return tiempo, desp, p_sim, p_exp, ym


def cargar_experimental(path):
    desp, pres = [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            desp.append(float(row["desplazamiento_mm"]))
            pres.append(float(row["presion_kPa"]))
    return desp, pres


def main():
    tiempo_sim, desp_sim, p_sim, p_exp_from_sim, ym_sim = cargar_simulacion(SIM_CSV)
    desp_exp, p_exp = cargar_experimental(EXP_CSV)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))

    # --- Panel 1: Presión vs Desplazamiento (curva completa) + YM en eje derecho ---
    ax = axes[0]
    ax.plot(desp_exp, p_exp, label="Presión experimental", color="black",
            linewidth=2, linestyle="--")
    ax.plot(desp_sim, p_sim, label="Presión simulada (QP inverso)",
            color="tab:red", linewidth=1.5)
    ax.set_xlabel("Desplazamiento (mm)")
    ax.set_ylabel("Presión (kPa)")
    ax.set_title("Presión y YM vs. Desplazamiento")
    ax.grid(alpha=0.3)


    # Leyenda combinada de ambos ejes
    lines1, labels1 = ax.get_legend_handles_labels()
    ax.legend(lines1, labels1, loc="best")

    # --- Panel 2: Error de presión simulada vs experimental, por paso ---
    ax2 = axes[1]
    error = [s - e for s, e in zip(p_sim, p_exp_from_sim)]
    ax2.plot(desp_sim, error, color="tab:blue", linewidth=1.2)
    ax2.axhline(0, color="gray", linewidth=1, linestyle=":")
    ax2.set_xlabel("Desplazamiento (mm)")
    ax2.set_ylabel("Error de presión (kPa) [sim - exp]")
    ax2.set_title("Error simulación vs. experimento")
    ax2.grid(alpha=0.3)

    # --- Panel 3: Evolución del Young Modulus vs. Desplazamiento ---
    ax3 = axes[2]
    ax3.plot(desp_sim, ym_sim, color="tab:green", linewidth=1.5)
    ax3.set_xlabel("Desplazamiento (mm)")
    ax3.set_ylabel("Young Modulus (Pa)")
    ax3.set_title("Evolución del YM vs. Desplazamiento")
    ax3.grid(alpha=0.3)

    fig.tight_layout()
    out_path = os.path.join(BASE_DIR, "comparacion_presion_desplazamiento_40.png")
    fig.savefig(out_path, dpi=150)
    print(f"Gráfico guardado en: {out_path}")


if __name__ == "__main__":
    main()
