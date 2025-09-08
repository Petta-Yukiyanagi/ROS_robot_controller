// lib/pages/battery_page.dart
import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';

import '../models/battery_sample.dart';
import '../services/battery_estimator.dart';
import '../services/ros_service.dart';

class BatteryPage extends StatefulWidget {
  final RosService ros;
  const BatteryPage({super.key, required this.ros});
  @override
  State<BatteryPage> createState() => _BatteryPageState();
}

class _BatteryPageState extends State<BatteryPage> {
  final List<BatterySample> _samples = [];
  StreamSubscription<BatterySample>? _sub;

  // 表示ウィンドウ（変更可）
  static const _windows = <String, Duration>{
    '1m': Duration(minutes: 1),
    '5m': Duration(minutes: 5),
    '15m': Duration(minutes: 15),
    '60m': Duration(minutes: 60),
  };
  String _winKey = '5m';
  Duration get _win => _windows[_winKey] ?? const Duration(minutes: 5);

  @override
  void initState() {
    super.initState();
    // 受信したデータを保持（グラフは時間窓で「切り抜き」）
    _sub = widget.ros.batteryStream.listen((s) {
      // 何かしらフィールドが入っているレコードのみ保持
      final hasBatteryFields = s.percentage != null ||
          s.voltage != null ||
          s.current != null ||
          s.charge != null ||
          s.capacity != null ||
          s.powerSupplyStatus != null;
      if (hasBatteryFields || s.chargingStateCode != null) {
        _samples.add(s);
      }

      // 履歴は最大24h保持（描画時にウィンドウでスライス）
      final cutoff = DateTime.now().subtract(const Duration(hours: 24));
      _samples.removeWhere((e) => e.t.isBefore(cutoff));

      if (mounted) setState(() {});
    });
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final latest = _latestFull();
    final eta = BatteryEstimator.estimateToFull(_samples);
    return Scaffold(
      appBar: AppBar(title: const Text('Battery Dashboard')),
      body: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // ───── 上部の概要カード + 時間窓チップ ─────
            Card(
              child: Padding(
                padding: const EdgeInsets.all(12.0),
                child: Wrap(
                  spacing: 16,
                  runSpacing: 8,
                  crossAxisAlignment: WrapCrossAlignment.center,
                  children: [
                    _kv('Status', BatteryEstimator.statusText(latest?.powerSupplyStatus)),
                    _kv('Charging code', latest?.chargingStateCode?.toString() ?? '—'),
                    _kv('Voltage', latest?.voltage != null ? '${latest!.voltage!.toStringAsFixed(2)} V' : '—'),
                    _kv('Current', latest?.current != null ? '${latest!.current!.toStringAsFixed(3)} A' : '—'),
                    _kv('Percent', latest?.percentage != null ? '${(latest!.percentage! * 100).toStringAsFixed(0)} %' : '—'),
                    _kv('ETA full', eta != null ? _fmtDur(eta) : '—'),
                    const SizedBox(width: 8),
                    Wrap(
                      spacing: 6,
                      children: _windows.keys.map((k) {
                        return ChoiceChip(
                          label: Text(k),
                          selected: _winKey == k,
                          onSelected: (_) => setState(() => _winKey = k),
                        );
                      }).toList(),
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 12),

            // ───── グラフ（時間窓で切り抜き & 自動スケール） ─────
            Expanded(
              child: ListView(
                children: [
                  _chartCard(
                    'Percentage',
                    _lineData(
                      pick: (s) => s.percentage == null ? null : s.percentage! * 100.0,
                      unit: '%',
                      preferredRange: const _PreferredRange(min: 0, max: 100, addPadding: true),
                    ),
                  ),
                  _chartCard(
                    'Voltage',
                    _lineData(pick: (s) => s.voltage, unit: 'V'),
                  ),
                  _chartCard(
                    'Current',
                    _lineData(pick: (s) => s.current, unit: 'A'),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _kv(String k, String v) => Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text('$k: ', style: const TextStyle(fontWeight: FontWeight.bold)),
          Text(v),
        ],
      );

  String _fmtDur(Duration d) {
    final h = d.inHours;
    final m = d.inMinutes % 60;
    return h > 0 ? '${h}h ${m}m' : '${d.inMinutes}m';
  }

  BatterySample? _latestFull() {
    for (var i = _samples.length - 1; i >= 0; i--) {
      final s = _samples[i];
      if (s.percentage != null ||
          s.voltage != null ||
          s.current != null ||
          s.powerSupplyStatus != null ||
          s.chargingStateCode != null) {
        return s;
      }
    }
    return null;
  }

  /// 指定 selector で y を取り出して LineChartData を構築
  LineChartData _lineData({
    required double? Function(BatterySample) pick,
    required String unit,
    _PreferredRange? preferredRange,
  }) {
    final now = DateTime.now();
    final from = now.subtract(_win);

    // 時間窓で切り抜き
    final windowed = _samples.where((s) => !s.t.isBefore(from)).toList();
    // X: -window..0（分）に固定（右端が現在時刻）
    final minX = -_win.inMinutes.toDouble();
    const maxX = 0.0;

    // 連続セグメント（大きなギャップは線を途切れさせる）
    final segments = _buildSegments(
      windowed: windowed,
      pick: pick,
      now: now,
      gapThreshold: const Duration(seconds: 15),
    );

    // Y の自動スケール
    final allY = segments.expand((s) => s).map((p) => p.y).toList();
    final yScale = _computeYScale(allY, preferredRange: preferredRange);

    return LineChartData(
      minX: minX,
      maxX: maxX,
      minY: yScale.min,
      maxY: yScale.max,
      gridData: FlGridData(
        show: true,
        horizontalInterval: _niceStep((yScale.max - yScale.min) / 5),
        drawVerticalLine: true,
      ),
      titlesData: FlTitlesData(
        leftTitles: AxisTitles(
          sideTitles: SideTitles(
            showTitles: true,
            reservedSize: 44,
            getTitlesWidget: (v, _) => Text(v.toStringAsFixed(_digitsForStep(yScale.max - yScale.min))),
          ),
        ),
        rightTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
        topTitles: const AxisTitles(sideTitles: SideTitles(showTitles: false)),
        bottomTitles: AxisTitles(
          sideTitles: SideTitles(
            showTitles: true,
            reservedSize: 24,
            getTitlesWidget: (v, _) => _bottomTickLabel(v),
          ),
        ),
      ),
      lineBarsData: segments
          .map((spots) => LineChartBarData(
                spots: spots,
                isCurved: true,
                barWidth: 2,
                dotData: const FlDotData(show: false),
              ))
          .toList(),
      borderData: FlBorderData(show: true),
    );
  }

  /// 時間のラベル（-window..0 分）
  Widget _bottomTickLabel(double v) {
    final m = v.abs().toStringAsFixed(0);
    return Text('$m m', style: const TextStyle(fontSize: 10));
  }

  /// 連続セグメントを作る（データギャップで分割）
  List<List<FlSpot>> _buildSegments({
    required List<BatterySample> windowed,
    required double? Function(BatterySample) pick,
    required DateTime now,
    required Duration gapThreshold,
  }) {
    final points = <({DateTime t, double y})>[];
    for (final s in windowed) {
      final y = pick(s);
      if (y == null || !y.isFinite) continue;
      points.add((t: s.t, y: y));
    }
    points.sort((a, b) => a.t.compareTo(b.t));

    final segments = <List<FlSpot>>[];
    List<FlSpot>? cur;
    DateTime? prevT;

    for (final p in points) {
      final x = -now.difference(p.t).inMilliseconds / 60000.0;
      if (prevT == null || p.t.difference(prevT).abs() <= gapThreshold) {
        cur ??= <FlSpot>[];
        cur.add(FlSpot(x, p.y));
      } else {
        // 新しいセグメント
        if (cur != null && cur.isNotEmpty) segments.add(cur);
        cur = <FlSpot>[FlSpot(x, p.y)];
      }
      prevT = p.t;
    }
    if (cur != null && cur.isNotEmpty) segments.add(cur);
    return segments;
  }

  /// Y スケールを計算（必要に応じて 0–100% の制限や余白付与）
  _YScale _computeYScale(List<double> ys, {_PreferredRange? preferredRange}) {
    if (ys.isEmpty) {
      if (preferredRange != null) {
        return _YScale(preferredRange.min, preferredRange.max);
      }
      return const _YScale(0, 1);
    }
    double minY = ys.reduce(math.min);
    double maxY = ys.reduce(math.max);

    // 希望レンジ（例：percent 0..100）
    if (preferredRange != null) {
      minY = minY.clamp(preferredRange.min, preferredRange.max);
      maxY = maxY.clamp(preferredRange.min, preferredRange.max);
    }

    if (minY == maxY) {
      // フラットな線：±5% 余白（または固定幅）
      final pad = (maxY.abs() + 1) * 0.05;
      minY -= pad;
      maxY += pad;
      if (minY == maxY) {
        minY -= 1;
        maxY += 1;
      }
    } else {
      // 全体の8%を余白に
      final pad = (maxY - minY) * 0.08;
      minY -= pad;
      maxY += pad;
    }

    if (preferredRange != null && preferredRange.addPadding) {
      // さらに端で 0..100 を超えないようにトリミング
      minY = minY.clamp(preferredRange.min, preferredRange.max);
      maxY = maxY.clamp(preferredRange.min, preferredRange.max);
    }
    return _YScale(minY, maxY);
  }

  // 目盛間隔を綺麗な値に
  double _niceStep(double raw) {
    if (!raw.isFinite || raw <= 0) return 1;
    final exp = (math.log(raw) / math.ln10).floor();
    final base = math.pow(10, exp).toDouble();
    final fraction = raw / base;
    double nice;
    if (fraction < 1.5) {
      nice = 1;
    } else if (fraction < 3) {
      nice = 2;
    } else if (fraction < 7) {
      nice = 5;
    } else {
      nice = 10;
    }
    return nice * base;
  }

  int _digitsForStep(double span) {
    if (!span.isFinite || span <= 0) return 0;
    if (span >= 50) return 0;
    if (span >= 5) return 1;
    return 2;
  }

  Widget _chartCard(String title, LineChartData data) => Card(
        child: Padding(
          padding: const EdgeInsets.all(12.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Text(title, style: const TextStyle(fontWeight: FontWeight.bold)),
              const SizedBox(height: 8),
              SizedBox(height: 220, child: LineChart(data)),
            ],
          ),
        ),
      );
}

class _PreferredRange {
  final double min;
  final double max;
  final bool addPadding;
  const _PreferredRange({required this.min, required this.max, this.addPadding = false});
}

class _YScale {
  final double min;
  final double max;
  const _YScale(this.min, this.max);
}
