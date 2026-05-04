#!/usr/bin/env python3
"""位置误差统计脚本

期望输入 CSV (header): timestamp,commanded_mm,measured_mm
输出 RMSE、MAE、最大误差和标准差。

用法示例：
  python3 tools/position_analysis.py data/position_log.csv
"""
import sys
import csv
import math


def analyze(path):
    commanded = []
    measured = []
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                cmd = float(row['commanded_mm'])
                meas = float(row['measured_mm'])
            except Exception:
                continue
            commanded.append(cmd)
            measured.append(meas)

    if not commanded:
        print('没有读取到数据，请检查 CSV 格式与列名')
        return

    errors = [m - c for c, m in zip(commanded, measured)]
    n = len(errors)
    mae = sum(abs(e) for e in errors) / n
    mse = sum(e*e for e in errors) / n
    rmse = math.sqrt(mse)
    max_err = max(errors, key=abs)
    mean = sum(errors) / n
    var = sum((e - mean)**2 for e in errors) / n
    std = math.sqrt(var)

    print(f'样本数: {n}')
    print(f'RMSE: {rmse:.5f} mm')
    print(f'MAE: {mae:.5f} mm')
    print(f'最大绝对误差: {max_err:.5f} mm')
    print(f'平均误差: {mean:.5f} mm')
    print(f'标准差: {std:.5f} mm')


def main():
    if len(sys.argv) < 2:
        print('用法: python3 tools/position_analysis.py data/position_log.csv')
        return
    analyze(sys.argv[1])


if __name__ == '__main__':
    main()
