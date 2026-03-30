import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# ── Configuration ──────────────────────────────────────────────────────────────
STATS_FILES = {
    'Task 2 (No mapping)': './z_resultados_lab4/stats_task2.txt',
    'Task 3 (Fusion)':     './z_resultados_lab4/stats_task3.txt',
    'Full system 30%':         './z_resultados_lab4/stats_taskAll_03.txt',
    'Full system 50%':         './z_resultados_lab4/stats_taskAll_05.txt',
    'Full system 100%':    './z_resultados_lab4/stats_taskAll.txt',
}

ATE_FILES = {
    'Task 2 (No mapping)': './z_resultados_lab4/ATE_task2.txt',
    'Task 3 (Fusion)':     './z_resultados_lab4/ATE_task3.txt',
    'Full system 30%':         './z_resultados_lab4/ATE_taskall_03.txt',
    'Full system 50%':         './z_resultados_lab4/ATE_taskall_05.txt',
    'Full system 100%':    './z_resultados_lab4/ATE_taskall.txt',
}

# ── Loaders ────────────────────────────────────────────────────────────────────
def load_stats(path):
    """
    Expected columns (no header):
        mean_time, median_time, std_time, map_points
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [float(x) for x in line.split(',')]
            rows.append({
                'mean_time':  parts[0],
                'median_time': parts[1],
                'std_time':   parts[2],
                'map_points': parts[3],
            })
    return pd.DataFrame(rows)


def load_ate(path):
    """
    Expected columns (no header):
        scale, traj_length, rmse_ate, mean_ate, median_ate, ...
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [float(x) for x in line.split(',')]
            rows.append({
                'scale':       parts[0],
                'traj_length': parts[1],
                'rmse_ate':    parts[2],
                'mean_ate':    parts[3],
                'median_ate':  parts[4],
            })
    return pd.DataFrame(rows)


# ── Collect data ───────────────────────────────────────────────────────────────
labels = list(STATS_FILES.keys())

stats_data = {label: load_stats(path) for label, path in STATS_FILES.items()}
ate_data   = {label: load_ate(path)   for label, path in ATE_FILES.items()}

# ── Plot helpers ───────────────────────────────────────────────────────────────
COLORS = ['#4C72B0', '#DD8452', '#55A868', '#C44E52']

def make_boxplot(ax, series_dict, labels, color_list, ylabel, title):
    data   = [series_dict[l] for l in labels]
    bp = ax.boxplot(
        data,
        patch_artist=True,
        notch=False,
        widths=0.45,
        medianprops=dict(color='black', linewidth=2),
        whiskerprops=dict(linewidth=1.4),
        capprops=dict(linewidth=1.4),
        flierprops=dict(marker='o', markersize=5, linestyle='none',
                        markeredgecolor='grey'),
    )
    for patch, color in zip(bp['boxes'], color_list):
        patch.set_facecolor(color)
        patch.set_alpha(0.75)

    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(title, fontsize=11, fontweight='bold')
    ax.yaxis.grid(True, linestyle='--', alpha=0.7)
    ax.set_axisbelow(True)


# ── Figure 1 – Trajectory length ──────────────────────────────────────────────
fig1, ax1 = plt.subplots(figsize=(8, 5))
make_boxplot(
    ax1,
    {l: ate_data[l]['traj_length'].values for l in labels},
    labels, COLORS,
    ylabel='Trajectory length (m)',
    title='Trajectory Length across Configurations',
)
fig1.tight_layout()
fig1.savefig('plot_traj_length.pdf', dpi=150)
fig1.savefig('plot_traj_length.png', dpi=150)
print('Saved: plot_traj_length')

# ── Figure 2 – RMSE ATE ───────────────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(8, 5))
make_boxplot(
    ax2,
    {l: ate_data[l]['rmse_ate'].values for l in labels},
    labels, COLORS,
    ylabel='RMSE ATE (m)',
    title='Absolute Translation Error (RMSE) across Configurations',
)
fig2.tight_layout()
fig2.savefig('plot_rmse_ate.pdf', dpi=150)
fig2.savefig('plot_rmse_ate.png', dpi=150)
print('Saved: plot_rmse_ate')

# ── Figure 3 – Mean processing time per frame ─────────────────────────────────
fig3, ax3 = plt.subplots(figsize=(8, 5))
make_boxplot(
    ax3,
    {l: stats_data[l]['mean_time'].values for l in labels},
    labels, COLORS,
    ylabel='Mean processing time (ms)',
    title='Mean Processing Time per Frame across Configurations',
)
fig3.tight_layout()
fig3.savefig('plot_mean_time.pdf', dpi=150)
fig3.savefig('plot_mean_time.png', dpi=150)
print('Saved: plot_mean_time')

# ── Figure 4 – Final map size ─────────────────────────────────────────────────
fig4, ax4 = plt.subplots(figsize=(8, 5))
make_boxplot(
    ax4,
    {l: stats_data[l]['map_points'].values for l in labels},
    labels, COLORS,
    ylabel='Number of MapPoints',
    title='Final Map Size across Configurations',
)
fig4.tight_layout()
fig4.savefig('plot_map_points.pdf', dpi=150)
fig4.savefig('plot_map_points.png', dpi=150)
print('Saved: plot_map_points')

# ── Summary statistics ─────────────────────────────────────────────────────────
print("\n" + "="*65)
print(f"{'Variable':<35} {'Mean':>8} {'Std':>8} {'Min':>8} {'Max':>8}")
print("="*65)

for label in labels:
    print(f"\n── {label} ──")
    
    # ATE file metrics
    df_ate = ate_data[label]
    for col, name in [
        ('traj_length', 'Trajectory length (m)'),
        ('rmse_ate',    'RMSE ATE (m)'),
        ('mean_ate',    'Mean ATE (m)'),
    ]:
        print(f"  {name:<33} {df_ate[col].mean():>8.4f} "
              f"{df_ate[col].std():>8.4f} "
              f"{df_ate[col].min():>8.4f} "
              f"{df_ate[col].max():>8.4f}")

    # Stats file metrics
    df_st = stats_data[label]
    for col, name in [
        ('mean_time',  'Mean time/frame (ms)'),
        ('map_points', 'Final map size (MPs)'),
    ]:
        print(f"  {name:<33} {df_st[col].mean():>8.2f} "
              f"{df_st[col].std():>8.2f} "
              f"{df_st[col].min():>8.2f} "
              f"{df_st[col].max():>8.2f}")

print("="*65)