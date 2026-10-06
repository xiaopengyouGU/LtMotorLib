import numpy as np
import matplotlib.pyplot as plt
import os

# ========== 设置中文字体 ==========
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ========== 创建 images 文件夹 ==========
os.makedirs('images', exist_ok=True)

# ========== 参数设置 ==========
Udc = 100.0
Um = Udc / 2
f = 100.0
fs = 20000.0
T = 1.0 / f
N = 6000
t = np.linspace(0, T, N, endpoint=False)
theta = 2 * np.pi * f * t

# ========== 三相相电压 ==========
Ua = Um * np.cos(theta)
Ub = Um * np.cos(theta - 2*np.pi/3)
Uc = Um * np.cos(theta + 2*np.pi/3)

# ========== SPWM端电压 ==========
Uag_spwm = Udc/2 + Ua
Ubg_spwm = Udc/2 + Ub
Ucg_spwm = Udc/2 + Uc

# ========== SVPWM端电压 ==========
max_val = np.maximum(np.maximum(Ua, Ub), Uc)
min_val = np.minimum(np.minimum(Ua, Ub), Uc)
U0 = -(max_val + min_val) / 2.0

Uag_svpwm = Udc/2 + Ua + U0
Ubg_svpwm = Udc/2 + Ub + U0
Ucg_svpwm = Udc/2 + Uc + U0

# ========== 绘制（尺寸缩小） ==========
fig, axs = plt.subplots(3, 1, figsize=(10, 7))  # 从 (14, 10) 缩小到 (10, 7)

# --- 第一张图：SPWM端电压 ---
axs[0].plot(t*1000, Uag_spwm, label='A相', linewidth=1.0)
axs[0].plot(t*1000, Ubg_spwm, label='B相', linewidth=1.0)
axs[0].plot(t*1000, Ucg_spwm, label='C相', linewidth=1.0)
axs[0].axhline(Udc, color='r', linestyle='--', linewidth=0.8, label='Udc')
axs[0].axhline(0, color='k', linestyle='--', linewidth=0.8)
axs[0].axhline(Udc/2, color='gray', linestyle=':', linewidth=0.6)
axs[0].set_xlabel('时间 (ms)')
axs[0].set_ylabel('端电压 (V)')
axs[0].set_title('SPWM 端电压 (中心偏置)')
axs[0].legend(loc='upper right', fontsize=8)
axs[0].grid(True, alpha=0.3)
axs[0].set_xlim(0, 12)

# --- 第二张图：SVPWM端电压 ---
axs[1].plot(t*1000, Uag_svpwm, label='A相 (马鞍波)', linewidth=1.0)
axs[1].plot(t*1000, Ubg_svpwm, label='B相 (马鞍波)', linewidth=1.0)
axs[1].plot(t*1000, Ucg_svpwm, label='C相 (马鞍波)', linewidth=1.0)
axs[1].axhline(Udc, color='r', linestyle='--', linewidth=0.8, label='Udc')
axs[1].axhline(0, color='k', linestyle='--', linewidth=0.8)
axs[1].axhline(Udc/2, color='gray', linestyle=':', linewidth=0.6)
axs[1].set_xlabel('时间 (ms)')
axs[1].set_ylabel('端电压 (V)')
axs[1].set_title('SVPWM 端电压 (马鞍波)')
axs[1].legend(loc='upper right', fontsize=8)
axs[1].grid(True, alpha=0.3)
axs[1].set_xlim(0, 12)

# --- 第三张图：相电压 ---
axs[2].plot(t*1000, Ua, label='A相', linewidth=1.0)
axs[2].plot(t*1000, Ub, label='B相', linewidth=1.0)
axs[2].plot(t*1000, Uc, label='C相', linewidth=1.0)
axs[2].axhline(Um, color='r', linestyle='--', linewidth=0.8, label=f'+Um = {Um:.1f}V')
axs[2].axhline(-Um, color='r', linestyle='--', linewidth=0.8, label=f'-Um = {Um:.1f}V')
axs[2].axhline(0, color='k', linestyle='-', linewidth=0.6)
axs[2].set_xlabel('时间 (ms)')
axs[2].set_ylabel('相电压 (V)')
axs[2].set_title('相电压 (SPWM和SVPWM完全相同)')
axs[2].legend(loc='upper right', fontsize=8)
axs[2].grid(True, alpha=0.3)
axs[2].set_xlim(0, 12)

# ========== 参数信息框（缩小字体） ==========
info_text = (
    f"$U_{{dc}}$ = {Udc} V\n"
    f"$U_m$ = {Um:.1f} V\n"
    f"$f$ = {f} Hz\n"
    f"$f_s$ = {fs/1000:.1f} kHz"
)

fig.text(0.07, 0.99, info_text, transform=fig.transFigure,
         fontsize=9, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))

plt.tight_layout(rect=[0, 0, 1, 0.94])

# ========== 保存 ==========
plt.savefig('images/spwm_svpwm_comparison.png', dpi=300, bbox_inches='tight')
print("图片已保存至: images/spwm_svpwm_comparison.png")

plt.show()

# ========== 终端输出 ==========
print("="*50)
print(f"直流母线电压 Udc = {Udc} V")
print(f"相电压幅值 Um = {Um:.2f} V (SVPWM线性区极限)")
print(f"基波频率 f = {f} Hz")
print(f"开关频率 fs = {fs} Hz")
print("="*50)
print(f"SPWM端电压峰值: {np.max(np.abs(Uag_spwm)):.2f} V")
print(f"SVPWM端电压峰值: {np.max(np.abs(Uag_svpwm)):.2f} V")
print("="*50)
print("结论: 端电压波形不同，但相电压波形完全相同")