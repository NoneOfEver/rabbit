#!/usr/bin/env python3
"""简单能耗与续航估算脚本

用法示例:
  python3 tools/energy_estimate.py --capacity-wh 18.5 --avg-power-mw 110

输出：估算运行小时数与示例能耗分配。
"""
import argparse


def main():
    p = argparse.ArgumentParser(description="估算电池续航时间")
    p.add_argument("--capacity-wh", type=float, default=18.5, help="电池容量，Wh，默认 18.5")
    p.add_argument("--avg-power-mw", type=float, default=110.0, help="平均功耗，mW，默认 110")
    args = p.parse_args()

    capacity = args.capacity_wh
    avg_power_w = args.avg_power_mw / 1000.0

    if avg_power_w <= 0:
        print("平均功耗必须大于 0")
        return

    hours = capacity / avg_power_w

    print(f"电池容量: {capacity:.2f} Wh")
    print(f"平均功耗: {args.avg_power_mw:.2f} mW ({avg_power_w:.3f} W)")
    print(f"估算运行时间: {hours:.1f} 小时 ({hours/24:.2f} 天)")

    # 示例分配（只是示例占比）
    controller_pct = 0.35
    sensing_pct = 0.15
    comms_pct = 0.10
    driver_pct = 0.40

    print("\n示例能耗分配（估算）:")
    print(f"  控制器: {controller_pct*100:.0f}% -> {(args.avg_power_mw*controller_pct):.2f} mW")
    print(f"  传感/采样: {sensing_pct*100:.0f}% -> {(args.avg_power_mw*sensing_pct):.2f} mW")
    print(f"  通信(BLE): {comms_pct*100:.0f}% -> {(args.avg_power_mw*comms_pct):.2f} mW")
    print(f"  驱动器/执行器: {driver_pct*100:.0f}% -> {(args.avg_power_mw*driver_pct):.2f} mW")


if __name__ == '__main__':
    main()
