import matplotlib.pyplot as plt
import csv
import sys

archivo_datos = 'timing_stats.txt'
medias = []

try:
    # Leer el archivo generado por C++
    with open(archivo_datos, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row: # Evitamos líneas en blanco
                # row[0] es la media, row[1] la mediana, row[2] la std_dev
                medias.append(float(row[0]))

    if not medias:
        print("No se encontraron datos en el archivo.")
        sys.exit()

    # Configurar y crear el Box and Whisker plot
    plt.figure(figsize=(8, 6))
    
    # patch_artist=True permite colorear la caja
    box = plt.boxplot(medias, patch_artist=True)
    
    # Darle un poco de color (opcional)
    for patch in box['boxes']:
        patch.set_facecolor('lightblue')

    plt.title('Distribución de las Medias de Tiempo de Procesamiento')
    plt.ylabel('Tiempo (ms)')
    plt.xticks([1], ['Media de ejecución'])
    plt.grid(axis='y', linestyle='--', alpha=0.7)

    # Guardar la gráfica en una imagen en lugar de solo mostrarla
    nombre_imagen = 'boxplot_medias.png'
    plt.savefig(nombre_imagen)
    print(f"Gráfica generada y guardada con éxito como '{nombre_imagen}'.")

    # Si prefieres que además se abra una ventana para verla al instante, descomenta la siguiente línea:
    # plt.show()

except FileNotFoundError:
    print(f"Error: No se encontró el archivo '{archivo_datos}'. Asegúrate de que el programa de C++ lo ha creado.")
except Exception as e:
    print(f"Ocurrió un error inesperado: {e}")