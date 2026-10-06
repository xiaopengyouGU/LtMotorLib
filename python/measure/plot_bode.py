import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import warnings

# 设置中文字体 - 使用微软雅黑
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 忽略字体警告
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')

def read_data_file(filepath):
    """
    读取数据文件，跳过表头行
    文件格式: frequency, amp, phase_lag
    """
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # 跳过表头行
            if 'frequency' in line.lower() or 'amp' in line.lower():
                continue
            # 解析数据行
            try:
                parts = line.split(',')
                if len(parts) >= 3:
                    freq = float(parts[0].strip())
                    amp = float(parts[1].strip())
                    phase_lag = float(parts[2].strip())
                    data.append([freq, amp, phase_lag])
            except ValueError:
                continue
    
    if not data:
        raise ValueError("未解析到有效数据，请检查文件格式是否为: frequency, amp, phase_lag")
    
    data = np.array(data)
    return data[:, 0], data[:, 1], data[:, 2]

def find_crossing_frequency(freq, gain_db, target_db):
    """
    寻找增益曲线穿越目标值的频率
    使用线性插值
    """
    # 找到第一个低于目标值的点
    idx = np.where(gain_db < target_db)[0]
    
    if len(idx) == 0:
        # 所有点都高于目标值
        print(f"警告: 所有频点均未低于 {target_db}dB，取最高频率点")
        return freq[-1]
    elif idx[0] == 0:
        # 第一个点就低于目标值
        print(f"警告: 第一个频点已低于 {target_db}dB")
        return freq[0]
    else:
        # 线性插值
        i = idx[0]
        f1, f2 = freq[i-1], freq[i]
        g1, g2 = gain_db[i-1], gain_db[i]
        f_cross = f1 + (f2 - f1) * (target_db - g1) / (g2 - g1)
        return f_cross

def plot_bode(freq, amp, phase_lag, amp_ref=0.3, save_path=None):
    """
    绘制Bode图并计算关键参数
    """
    # 1. 转换幅值为dB
    gain_db = 20 * np.log10(amp / amp_ref)
    
    # 2. 相位处理
    phase_deg = -phase_lag  # 负号后，表示实际相位滞后
    
    # 3. 寻找 -3dB 带宽
    target_db = -3.0
    f_cross = find_crossing_frequency(freq, gain_db, target_db)
    
    # 4. 寻找相位裕度
    # 先在增益曲线中找到 0dB 穿越点
    idx_0db = np.where(gain_db < 0)[0]
    
    phase_margin = None
    phase_at_cross = None
    f_0db = None
    
    if len(idx_0db) == 0:
        print("警告: 所有频点增益均大于 0dB，系统可能不稳定")
        f_0db = freq[-1]
    elif idx_0db[0] == 0:
        print("警告: 第一个频点已低于 0dB，增益穿越频率可能低于最低测试频率")
        f_0db = freq[0]
    else:
        # 线性插值求 0dB 穿越频率
        i = idx_0db[0]
        f1, f2 = freq[i-1], freq[i]
        g1, g2 = gain_db[i-1], gain_db[i]
        f_0db = f1 + (f2 - f1) * (0 - g1) / (g2 - g1)
        
        # 在该频率处插值相位
        phase_at_cross = np.interp(f_0db, freq, phase_deg)
        phase_margin = 180 + phase_at_cross
    
    # 5. 绘制Bode图
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # 幅频特性
    ax1.semilogx(freq, gain_db, 'b-s', linewidth=1.5, markersize=4)
    ax1.grid(True, which='both', linestyle='--', alpha=0.6)
    
    # 标注 -3dB 线
    ax1.axhline(y=target_db, color='r', linestyle='--', linewidth=1)
    ax1.axvline(x=f_cross, color='g', linestyle='--', linewidth=1)
    ax1.plot(f_cross, target_db, 'ro', markersize=10, markeredgewidth=2)
    ax1.text(f_cross * 0.4, target_db - 0.5, 
             f'带宽: {f_cross:.1f} Hz', 
             fontsize=10, color='r', fontweight='bold')
    
    ax1.set_xlabel('频率 (Hz)')
    ax1.set_ylabel('增益 (dB)')
    ax1.set_title('Bode 图 - 幅频特性')
    ax1.legend(['实测增益', '-3dB 线', '带宽频率'], loc='best')
    
    # 相频特性
    ax2.semilogx(freq, phase_deg, 'r-s', linewidth=1.5, markersize=4)
    ax2.grid(True, which='both', linestyle='--', alpha=0.6)
    
    # 标注增益穿越频率（0dB点）处的相位
    if phase_margin is not None:
        ax2.axvline(x=f_0db, color='g', linestyle='--', linewidth=1)
        ax2.plot(f_0db, phase_at_cross, 'ro', markersize=10, markeredgewidth=2)
        ax2.text(f_0db * 1.1, phase_at_cross + 5,
                f'相位裕度: {phase_margin:.1f}°',
                fontsize=10, color='r', fontweight='bold')
    
    # 标注 -180° 线（稳定边界）
    ax2.axhline(y=-180, color='k', linestyle='--', linewidth=0.5)
    
    ax2.set_xlabel('频率 (Hz)')
    ax2.set_ylabel('相位 (度)')
    ax2.set_title('Bode 图 - 相频特性')
    ax2.legend(['实测相位', '增益穿越频率'], loc='best')
    
    plt.tight_layout()
    
    # 6. 输出关键参数到控制台
    print('\n========== 测试结果 ==========')
    print(f'参考幅值: {amp_ref:.3f}')
    print(f'-3dB 带宽: {f_cross:.1f} Hz')
    if phase_margin is not None:
        print(f'增益穿越频率: {f_0db:.1f} Hz')
        print(f'相位裕度: {phase_margin:.1f}°')
    print('================================\n')
    
    # 7. 保存图片
    if save_path:
        # 确保目录存在
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f'图片已保存至: {save_path}')
    
    plt.show()
    
    return {
        'gain_db': gain_db,
        'phase_deg': phase_deg,
        'bandwidth': f_cross,
        'phase_margin': phase_margin,
        'gain_crossing_freq': f_0db
    }

def main():
    # 清空工作区（清理matplotlib图形）
    plt.close('all')
    
    # 数据目录
    data_dir = Path('./datas')
    if not data_dir.exists():
        print(f'警告: 目录 {data_dir} 不存在，创建目录')
        data_dir.mkdir(parents=True, exist_ok=True)
        print('请将数据文件 data.txt 放入 ./datas/ 目录')
        return
    
    # 查找数据文件
    files = list(data_dir.glob('data.txt'))
    if not files:
        print('在 ./datas/ 文件夹下未找到 data.txt 文件')
        print('请确保文件名为 data.txt')
        return
    
    # 读取数据
    filepath = files[0]
    print(f'正在读取: {filepath.name}')
    
    try:
        freq, amp, phase_lag = read_data_file(filepath)
        print(f'成功读取 {len(freq)} 个数据点')
    except Exception as e:
        print(f'读取文件失败: {e}')
        return
    
    # 参考幅值
    amp_ref = 0.3
    
    # 绘制Bode图
    save_path = Path('image/bode0.png')
    results = plot_bode(freq, amp, phase_lag, amp_ref, save_path)
    
    # 打印结果摘要
    if results:
        print('\n结果摘要:')
        print(f'  带宽: {results["bandwidth"]:.1f} Hz')
        if results['phase_margin'] is not None:
            print(f'  相位裕度: {results["phase_margin"]:.1f}°')
            print(f'  增益穿越频率: {results["gain_crossing_freq"]:.1f} Hz')

if __name__ == '__main__':
    main()