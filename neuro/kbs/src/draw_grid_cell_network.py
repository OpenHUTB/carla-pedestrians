#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""draw_grid_cell_network.py — 3D 网格细胞网络示意图 (替换旧 FCC 晶格图).

三个 panel, 全部对齐代码事实 (gc_iteration.m / main.m L136-164):
  (a) 3D 活动包: GRIDCELLS 36^3 中的局域化高斯活动 (分布式群体编码)
  (b) 可分离高斯激励核: 7x7x7, sigma=1.5; 3x3 平铺展示环面卷绕
  (c) 4-DoF 编码: 网格细胞 (x,y,z) + 多层头方向细胞 psi
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(SCRIPT_DIR, "..", "fig", "3d_grid_cell_network.pdf")

plt.rcParams["font.family"] = "DejaVu Sans"
plt.rcParams["axes.unicode_minus"] = False

fig = plt.figure(figsize=(13.5, 4.2))

# ---------------------------------------------------------------- (a) ----
ax_a = fig.add_subplot(1, 3, 1, projection="3d")
n = 36
cx, cy, cz = 18, 18, 18
x, y, z = np.meshgrid(np.arange(n), np.arange(n), np.arange(n), indexing="ij")
act = np.exp(-((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2) / (2 * 3.0 ** 2))
act /= act.max()
# 三层等活性壳 (近似 isosurface): 只画高活性点
for thr, s, alpha in [(0.75, 6, 0.9), (0.45, 6, 0.45), (0.25, 4, 0.18)]:
    m = (act > thr) & (act <= thr + 0.30)
    ax_a.scatter(x[m], y[m], z[m], c=act[m], cmap="hot", s=s, alpha=alpha,
                 linewidths=0)
ax_a.set_xlim(0, 35); ax_a.set_ylim(0, 35); ax_a.set_zlim(0, 35)
ax_a.set_xticks([0, 35]); ax_a.set_yticks([0, 35]); ax_a.set_zticks([0, 35])
ax_a.set_xlabel("$i$", labelpad=-6)
ax_a.set_ylabel("$j$", labelpad=-6)
ax_a.set_zlabel("$k$", labelpad=-6)
ax_a.view_init(elev=18, azim=38)
ax_a.set_title("(a) 3D activity packet ($36^3$ cells)", fontsize=11, pad=0)

# ---------------------------------------------------------------- (b) ----
ax_b = fig.add_subplot(1, 3, 2)
# 激励核中心切片: K(a,b,c)=N(a;0,1.5)N(b;0,1.5)N(c;0,1.5), 7x7x7 支撑
sig = 1.5
half = 3
g = np.exp(-np.arange(-half, half + 1) ** 2 / (2 * sig ** 2))
k_slice = np.outer(g, g)  # z=0 中心切片
# 3x3 平铺: 周围 8 块 = 环面卷绕后的邻域延伸, 中央为核本体
tiled = np.block([[k_slice] * 3])
tiled[7:14, 7:14] = 0  # 中央格留白画核本体
ax_b.imshow(tiled, cmap="Greys", extent=[-10.5, 10.5, 10.5, -10.5],
            vmin=0, vmax=1.05, interpolation="nearest")
im = ax_b.imshow(k_slice, cmap="YlOrRd", extent=[-3.5, 3.5, 3.5, -3.5],
                 vmin=0, vmax=1.05, interpolation="nearest")
for d in (-7, 0, 7):
    ax_b.axhline(d + 3.5 if d == -7 else d + 3.5, color="#B0BEC5", lw=0.5)
    ax_b.axvline(d + 3.5, color="#B0BEC5", lw=0.5)
ax_b.axhline(-3.5, color="#B0BEC5", lw=0.5); ax_b.axhline(3.5, color="#B0BEC5", lw=0.5)
ax_b.axvline(-3.5, color="#B0BEC5", lw=0.5); ax_b.axvline(3.5, color="#B0BEC5", lw=0.5)
ax_b.annotate("", xy=(-4.6, 0), xytext=(-10.2, 0),
              arrowprops=dict(arrowstyle="->", color="#1565C0", lw=1.4))
ax_b.annotate("", xy=(4.6, 0), xytext=(10.2, 0),
              arrowprops=dict(arrowstyle="->", color="#1565C0", lw=1.4))
ax_b.text(0, -9.6, "toroidal wrap", ha="center", fontsize=9, color="#1565C0")
ax_b.text(0, 9.0, "$7{\\times}7{\\times}7$ kernel, $\\sigma{=}1.5$", ha="center", fontsize=10)
fig.colorbar(im, ax=ax_b, shrink=0.8, pad=0.02).set_label("weight", fontsize=8)
ax_b.set_xlabel("$a$ (cells)"); ax_b.set_ylabel("$b$ (cells)")
ax_b.set_xticks([-10, -5, 0, 5, 10]); ax_b.set_yticks([-10, -5, 0, 5, 10])
ax_b.set_title("(b) Separable Gaussian excitation", fontsize=11)

# ---------------------------------------------------------------- (c) ----
ax_c = fig.add_subplot(1, 3, 3)
ax_c.set_xlim(0, 10); ax_c.set_ylim(0, 10); ax_c.axis("off")

def box(x0, y0, w, h, text, fc, fs=9.5):
    ax_c.add_patch(FancyBboxPatch((x0, y0), w, h,
                                  boxstyle="round,pad=0.12",
                                  facecolor=fc, edgecolor="#37474F", lw=1.1))
    ax_c.text(x0 + w / 2, y0 + h / 2, text, ha="center", va="center", fontsize=fs)

box(0.4, 6.2, 4.1, 2.2, "3D grid cells\nMD-CAN $36^3$\n$(x, y, z)$", "#FFCC80")
box(5.5, 6.2, 4.1, 2.2, "Multilayered\nhead direction cells\n$\\psi$ (yaw)", "#A5D6A7")
box(2.2, 2.2, 5.6, 2.0, "4-DoF pose\n$(x, y, z, \\psi)$", "#B3E5FC", fs=11)
ax_c.annotate("", xy=(4.4, 4.5), xytext=(2.4, 6.0),
              arrowprops=dict(arrowstyle="-|>", color="#37474F", lw=1.4))
ax_c.annotate("", xy=(5.6, 4.5), xytext=(7.6, 6.0),
              arrowprops=dict(arrowstyle="-|>", color="#37474F", lw=1.4))
ax_c.text(5.0, 9.6, "path integration + visual anchoring", ha="center",
          fontsize=8.5, style="italic", color="#546E7A")
ax_c.set_title("(c) 4-DoF encoding", fontsize=11)

plt.tight_layout()
plt.savefig(OUT, bbox_inches="tight", dpi=300)
png = OUT.replace(".pdf", ".png")
plt.savefig(png, bbox_inches="tight", dpi=150)
print(f"saved: {os.path.abspath(OUT)}")
