#!/bin/bash

NUM_VECES=10
LOG_FILE="ATE_metrics.txt"

echo "Ejecutando el programa $NUM_VECES veces..."
echo "Los mensajes por consola se guardarán en $LOG_FILE"
echo "--------------------------------------------------------"

for i in $(seq 1 $NUM_VECES); do
    echo "--- Ejecucion $i de $NUM_VECES ---"
    
    # Ejecutamos tu comando y guardamos la salida en el fichero haciendo append
    ./build/Apps/mono_euroc ./Datasets/vicon_room1/V1_02_medium ./Apps/EuRoC_TimeStamps/V102.txt 
    python3 Evaluation/evaluate_ate_scale.py Datasets/vicon_room1/V1_02_medium/left_gt.txt trajectory.txt --save align.txt --save_associations assiciated.txt --plot plot.png --verbose >> $LOG_FILE 2>&1
done

echo -e "\n¡Bucle terminado! Ejecutando script de Python para generar métricas..."

# Ejecutamos el script de Python
python3 lab4_time_stats.py