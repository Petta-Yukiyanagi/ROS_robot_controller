import 'dart:math';
import '../models/battery_sample.dart';

class BatteryEstimator {
  /// 直近10分の % 勾配から充電完了ETAを推定（%は0..1スケール）。
  static Duration? estimateToFull(List<BatterySample> samples) {
    final pts = samples
        .where((s) => s.percentage != null)
        .map((s) => Point<double>(
              s.t.millisecondsSinceEpoch / 1000.0, // sec
              s.percentage!,
            ))
        .toList();
    if (pts.length < 5) return null;

    final latest = pts.last.x;
    final filtered = pts.where((p) => latest - p.x <= 10 * 60).toList();
    if (filtered.length < 5) return null;

    final n = filtered.length.toDouble();
    final sumX = filtered.fold(0.0, (a, p) => a + p.x);
    final sumY = filtered.fold(0.0, (a, p) => a + p.y);
    final sumXX = filtered.fold(0.0, (a, p) => a + p.x * p.x);
    final sumXY = filtered.fold(0.0, (a, p) => a + p.x * p.y);
    final denom = max(n * sumXX - sumX * sumX, 1e-9);
    final slope = (n * sumXY - sumX * sumY) / denom; // %/sec (0..1)

    if (!slope.isFinite || slope <= 0) return null;
    final yNow = filtered.last.y;
    final remainSec = (1.0 - yNow) / slope;
    if (!remainSec.isFinite || remainSec <= 0) return null;
    return Duration(seconds: remainSec.round());
  }

  /// sensor_msgs/msg/BatteryState.power_supply_status の簡易ラベル
  static String statusText(int? s) {
    switch (s) {
      case 1: return 'Charging';
      case 2: return 'Discharging';
      case 3: return 'Not charging';
      case 4: return 'Full';
      default: return 'Unknown';
    }
  }
}







