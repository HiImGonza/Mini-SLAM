import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Definimos las cabeceras según el formato de salida del script de evaluación
headers = ['Scale', 'Pairs', 'RMSE', 'Mean', 'Median', 'Std', 'Min', 'Max']

# Cargamos los datos desde el archivo de texto
try:
    df = pd.read_csv('./z_resultados_lab4/ATE_task2Comparation.txt', names=headers)
    df['Run'] = range(1, len(df) + 1)
except FileNotFoundError:
    print("Error: El archivo 'z_task3_results.txt' no se encuentra.")
    exit()

# Cálculos adicionales
df['Scale_Error'] = np.abs(1 - df['Scale'])
rmse_cv = (df['RMSE'].std() / df['RMSE'].mean()) * 100

# --- Informe de Resultados en Inglés ---
print("--- SLAM Analysis Report (Task 3) ---")
print(f"Average RMSE ATE: {df['RMSE'].mean():.6f} m")
print(f"Average Mean ATE: {df['Mean'].mean():.6f} m") # Añadido al reporte
print(f"Average Trajectory Length (Pairs): {df['Pairs'].mean():.2f} frames")
print(f"RMSE Coefficient of Variation: {rmse_cv:.2f}%")

# Configuración del entorno gráfico
plt.switch_backend('Agg')
# Cambiamos a 1 fila y 3 columnas, aumentamos el ancho a 20
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 6))

# --- Gráfica 1: RMSE ATE Variability per Run ---
ax1.bar(df['Run'], df['RMSE'], color='skyblue', edgecolor='navy')
ax1.axhline(df['RMSE'].mean(), color='red', linestyle='--', 
            label=f'Mean RMSE: {df["RMSE"].mean():.4f}m')
ax1.set_title('RMSE ATE per Run', fontsize=14, fontweight='bold')
ax1.set_xlabel('Execution Run #', fontsize=12)
ax1.set_ylabel('RMSE (meters)', fontsize=12)
ax1.set_xticks(df['Run'])
ax1.legend()
ax1.grid(axis='y', linestyle=':', alpha=0.7)


# --- Gráfica 3: Trajectory Length per Run ---
ax2.bar(df['Run'], df['Pairs'], color='salmon', edgecolor='darkred')
ax2.axhline(df['Pairs'].mean(), color='blue', linestyle='--', 
            label=f'Mean Length: {df["Pairs"].mean():.2f}')
ax2.set_title('Trajectory Length per Run', fontsize=14, fontweight='bold')
ax2.set_xlabel('Execution Run #', fontsize=12)
ax2.set_ylabel('Number of Pairs (Frames)', fontsize=12)
ax2.set_xticks(df['Run'])
ax2.legend()
ax2.grid(axis='y', linestyle=':', alpha=0.7)

# Ajuste y guardado
plt.tight_layout()
plt.savefig('Task5_stats_2000.png', dpi=300)
print("Graph saved successfully as 'Task3_stats_2000.png'")