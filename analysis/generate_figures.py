#!/usr/bin/env python3
"""
generar_figuras_candidato02.py
Genera figuras vectoriales PDF compactas y legibles para el candidato_02.
Cumple especificaciones IEEE: ancho de columna 3.5 in, tipografia 7.5-8 pt legible,
altura reducida para integracion perfecta en el presupuesto de 6 paginas.
"""
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import argparse
parser = argparse.ArgumentParser(description='Regenerate the three paper figures from checked CSV data.')
parser.add_argument('--data', type=Path, required=True)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
DATOS_DIR = args.data.resolve()
FIGURAS_DIR = args.out.resolve()
FIGURAS_DIR.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 7.5,
    'axes.labelsize': 7.5,
    'axes.titlesize': 8.0,
    'legend.fontsize': 7.0,
    'xtick.labelsize': 7.0,
    'ytick.labelsize': 7.0,
    'pdf.fonttype': 42,
    'ps.fonttype': 42,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'lines.linewidth': 1.1,
    'figure.dpi': 300
})

colors = {'Observer': '#0072B2', 'LTVObserver': '#D55E00'}
styles = {'Observer': '-', 'LTVObserver': '--'}
labels = {'Observer': 'Flatness', 'LTVObserver': 'LTV'}
profiles = ['Inertia', 'GravityReverse']
profile_names = {'Inertia': 'A: Inertia', 'GravityReverse': 'B: GravityReverse'}
kinds = ['Observer', 'LTVObserver']

C = pd.read_csv(DATOS_DIR / 'trazas_control_figuras.csv')
T = pd.read_csv(DATOS_DIR / 'trazas_finas_figuras.csv')
W = pd.read_csv(DATOS_DIR / 'resumen_ventanas.csv')
S = pd.read_csv(DATOS_DIR / 'resumen_pareado.csv')

def save_fig(fig, filename):
    pdf_path = FIGURAS_DIR / f"{filename}.pdf"
    fig.savefig(pdf_path, bbox_inches='tight', pad_inches=0.02)
    plt.close(fig)
    print(f"Generado: {pdf_path.name}")

def grid(ax):
    ax.grid(True, linestyle=':', alpha=0.5, color='.75')

def render_fig_1_1col():
    # Altura optimizada a 2.3 in para caber bajo Tabla II con fuentes de 7.5 pt
    fig, axs = plt.subplots(2, 2, figsize=(3.5, 2.3), sharex=True, sharey='row', layout='constrained')
    
    for j, profile in enumerate(profiles):
        onset = 0.2 * ([4.5, 3.0][j])
        for kind in kinds:
            c = C.loc[(C.profile == profile) & (C.case_name == 'shape1_onset1') & (C.controller == kind)]
            z = T.loc[(T.profile == profile) & (T.case_name == 'shape1_onset1') & (T.controller == kind)]
            axs[0, j].plot(c.time_s - onset, c.joint_error_norm_mrad, color=colors[kind], ls=styles[kind],
                           marker='o', ms=1.5, lw=0.9)
            axs[1, j].plot(z.time_s - onset, z.acceleration_utilization_all_joints, color=colors[kind],
                           ls=styles[kind], lw=1.0)
        
        axs[0, j].set_title(f"Path {'A' if j==0 else 'B'}", fontweight='bold', fontsize=7.5, pad=2)
        axs[1, j].axhline(1.0, color='black', ls=':', lw=0.9)
        axs[1, j].set(xlim=(-0.06, 0.32), xlabel='$t - t_0$ [s]')
        for ax in axs[:, j]:
            ax.axvline(0, color='.4', ls='-.', lw=0.7)
            grid(ax)
            
    axs[0, 0].set_ylabel(r'$\|e_q\|_2$ [mrad]', fontsize=7.2)
    axs[1, 0].set_ylabel(r'Max $|\ddot{q}_i| / \bar{\ddot{q}}_i$', fontsize=7.2)
    
    handles = [Line2D([0], [0], color=colors[k], ls=styles[k], lw=1.0, label=labels[k]) for k in kinds]
    handles.append(Line2D([0], [0], color='black', ls=':', lw=0.9, label='Limit'))
    fig.legend(handles=handles, loc='outside lower center', ncol=3, frameon=False, fontsize=6.8, handlelength=1.5)
    save_fig(fig, 'fig_1_transitorio_escalon_ieee_1col')

def render_fig_2_compacta_B_1col():
    # Altura optimizada a 2.3 in para caber bajo Tabla II con fuentes de 7.5 pt
    fig, axs = plt.subplots(2, 1, figsize=(3.5, 2.3), sharex=True, layout='constrained')
    profile = 'GravityReverse'
    Tnom = 3.0
    end = 3.8
    tend_adapted = float(C.loc[(C.profile == profile) & (C.case_name == 'limited_authority') & (C.phase >= 1 - 1e-8), 'time_s'].min())
    
    # 1. Fase de referencia
    for kind in kinds:
        c = C.loc[(C.profile == profile) & (C.case_name == 'limited_authority') & (C.controller == kind)]
        axs[0].plot(c.time_s, c.phase, color=colors[kind], ls=styles[kind], lw=1.2)
    tt = c.time_s.to_numpy()
    v = np.clip(tt / Tnom, 0, 1)
    nom = 10 * v**3 - 15 * v**4 + 6 * v**5
    axs[0].plot(tt, nom, color='black', ls=':', lw=1.0)
    axs[0].axvspan(0.5, 0.8, color='.92', zorder=-2)
    axs[0].axvline(Tnom, color='.5', ls='-.', lw=0.8)
    axs[0].set(title=r'Path B: GravityReverse ($\alpha_2 \to 0.50$)', ylabel='Phase $s(t)$', ylim=(-0.03, 1.06))
    axs[0].text(0.97, 0.12, f'Completion: {tend_adapted:.2f} s (nom. {Tnom:.1f} s)',
                ha='right', transform=axs[0].transAxes, fontsize=6.8,
                bbox=dict(boxstyle='square,pad=0.15', facecolor='white', alpha=0.85, edgecolor='none'))
    grid(axs[0])
    
    # 2. Torques y comandos
    for kind in kinds:
        c = C.loc[(C.profile == profile) & (C.case_name == 'limited_authority') & (C.controller == kind)]
        z = T.loc[(T.profile == profile) & (T.case_name == 'limited_authority') & (T.controller == kind)]
        axs[1].plot(c.time_s, c.u2_Nm, color=colors[kind], ls=(0, (3, 1, 1, 1)), lw=0.85, alpha=0.85)
        axs[1].plot(z.time_s, z.tau2_Nm, color=colors[kind], ls=styles[kind], lw=1.2)
        
    axs[1].plot(z.time_s, z.available_torque2_Nm, color='black', ls=':', lw=1.0)
    axs[1].plot(z.time_s, -z.available_torque2_Nm, color='black', ls=':', lw=1.0)
    axs[1].axhline(39, color='.55', ls='--', lw=0.8)
    axs[1].axhline(-39, color='.55', ls='--', lw=0.8)
    axs[1].axvspan(0.5, 0.8, color='.92', zorder=-2)
    axs[1].axvline(Tnom, color='.5', ls='-.', lw=0.8)
    axs[1].set(xlim=(0, end), xlabel='Time [s]', ylabel=r'Joint 2: $u_2, \tau_2$ [N$\cdot$m]', ylim=(-43, 43))
    grid(axs[1])
    
    legend_elements = [
        Line2D([0], [0], color=colors['Observer'], ls=styles['Observer'], lw=1.1, label=r'F $\tau_2$'),
        Line2D([0], [0], color=colors['Observer'], ls=(0, (3, 1, 1, 1)), lw=0.85, label=r'F $u_2$'),
        Line2D([0], [0], color=colors['LTVObserver'], ls=styles['LTVObserver'], lw=1.1, label=r'L $\tau_2$'),
        Line2D([0], [0], color=colors['LTVObserver'], ls=(0, (3, 1, 1, 1)), lw=0.85, label=r'L $u_2$'),
        Line2D([0], [0], color='black', ls=':', lw=1.0, label=r'$\pm\alpha_2\bar{u}_2$'),
        Line2D([0], [0], color='.55', ls='--', lw=0.8, label=r'$\pm\bar{u}_2$')
    ]
    fig.legend(handles=legend_elements, loc='outside lower center', ncol=3, frameon=False, fontsize=6.5, handlelength=1.5)
    save_fig(fig, 'fig_2_autoridad_limitada_compacta_B_1col')

def render_fig_3_1col():
    # Altura optimizada a 1.7 in
    fig, axs = plt.subplots(2, 1, figsize=(3.5, 1.7), sharex=True, layout='constrained')
    dot_colors = ['#222222', '#0072B2', '#D55E00']
    
    for j, profile in enumerate(profiles):
        rr = S.loc[(S.profile == profile) & (S.metric == 'TV_cmd')].iloc[0]
        vals = [(rr.mean_paired_change_percent, rr.ci_low, rr.ci_high)]
        for window in ['motion', 'hold']:
            r = W.loc[(W.profile == profile) & (W.window == window) & (W.metric == 'TV')].iloc[0]
            vals.append((r.paired_change, r.ci_low, r.ci_high))
            
        for y, (mean, lo, hi) in enumerate(vals):
            axs[j].errorbar(mean, y, xerr=[[mean - lo], [hi - mean]], fmt='o', ms=3.5,
                            color=dot_colors[y], capsize=2.5, lw=1.0)
            axs[j].text(mean + 1.0, y + 0.12, f'{mean:.1f}%', fontsize=6.8, va='bottom', fontweight='bold',
                        color=dot_colors[y])
            
        axs[j].axvline(0, color='.3', ls=':', lw=0.8)
        axs[j].set(title=profile_names[profile], yticks=[0, 1, 2],
                   yticklabels=['Full 10 s', 'Motion', 'Hold'],
                   xlim=(-32, 4), ylim=(2.5, -0.5))
        grid(axs[j])
    axs[1].set_xlabel('Mean paired TV change [%]', fontsize=7.2)
    save_fig(fig, 'fig_3_variacion_por_ventana_ieee_1col')

if __name__ == '__main__':
    render_fig_1_1col()
    render_fig_2_compacta_B_1col()
    render_fig_3_1col()
    print("Generated all three paper figures.")
