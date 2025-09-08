// lib/services/ros_service.dart
import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:roslibdart/roslibdart.dart';
import '../models/battery_sample.dart';

/// ROS 接続＆I/O の薄いサービス層。
/// - /cmd_vel publish
/// - /roomba/battery_state 購読（BatteryState）
/// - /charging_state 購読（Int32）
class RosService with ChangeNotifier {
  Ros? _ros;
  Topic? _cmdVel;
  Topic? _batteryTopic;
  Topic? _chargingStateTopic;

  bool _connected = false;
  bool get connected => _connected;

  String _host = '192.168.10.115';
  int _wsPort = 9090;
  String get wsUrl => 'ws://$_host:$_wsPort';

  // 最新サンプル（単発）
  final _batteryCtrl = StreamController<BatterySample>.broadcast();
  Stream<BatterySample> get batteryStream => _batteryCtrl.stream;

  // 履歴（グラフ用）
  final _historyCtrl = StreamController<List<BatterySample>>.broadcast();
  Stream<List<BatterySample>> get batteryHistoryStream => _historyCtrl.stream;
  final List<BatterySample> _history = <BatterySample>[];

  // /battery_state と /charging_state をマージする用
  BatterySample? _last;

  Future<void> connect({String? host, int? port}) async {
    if (host != null) _host = host;
    if (port != null) _wsPort = port;

    _connected = false;
    notifyListeners();

    await close();

    _ros = Ros(url: wsUrl);
    _ros!.connect(); // fire-and-forget

    // /cmd_vel
    _cmdVel = Topic(
      ros: _ros!,
      // ROS2 + rosbridge 形式の type（環境に合わせてここは調整可能）
      name: '/cmd_vel',
      type: 'geometry_msgs/msg/Twist',
      queueSize: 10,
    )..advertise();

    // //battery_state
    _batteryTopic = Topic(
      ros: _ros!,
      name: '/battery_state',
      type: 'sensor_msgs/msg/BatteryState',
      queueSize: 1,
    )..subscribe((msg) {
        final now = DateTime.now();

        // percentage は 0..1 または 0..100 のことがある。未知/NaN/負値は null に丸める。
        double? pct;
        final raw = msg['percentage'];
        if (raw != null) {
          final n = (raw is int) ? raw.toDouble() : (raw as num).toDouble();
          if (n.isFinite && n >= 0) {
            pct = n > 1.0 ? (n > 100.0 ? 1.0 : n / 100.0) : n;
          }
        }

        final sample = BatterySample(
          t: now,
          percentage: pct,
          voltage: (msg['voltage'] as num?)?.toDouble(),
          current: (msg['current'] as num?)?.toDouble(),
          charge: (msg['charge'] as num?)?.toDouble(),
          capacity: (msg['capacity'] as num?)?.toDouble(),
          powerSupplyStatus: (msg['power_supply_status'] as int?),
          // chargingStateCode は別トピックで入るのでここでは触らない
        );

        _handleIncomingSample(sample);
        return Future.value();
      });

    // /charging_state（Int32）— ここで得た状態も BatterySample に合流
    _chargingStateTopic = Topic(
      ros: _ros!,
      name: '/charging_state',
      type: 'std_msgs/Int32',
      queueSize: 1,
    )..subscribe((msg) {
        final now = DateTime.now();
        final sample = BatterySample(
          t: now,
          chargingStateCode: msg['data'] as int?,
        );
        _handleIncomingSample(sample);
        return Future.value();
      });

    _connected = true;
    notifyListeners();
  }

  /// /cmd_vel publish
  void publishCmdVel(double linear, double angular) {
    if (!_connected || _cmdVel == null) return;
    final twist = {
      'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
      'angular': {'x': 0.0, 'y': 0.0, 'z': angular},
    };
    try {
      _cmdVel!.publish(twist);
    } catch (_) {}
  }

  void stop() => publishCmdVel(0, 0);

  /// 切断・クリーンアップ
  Future<void> close() async {
    try {
      _cmdVel?.unadvertise();
    } catch (_) {}
    try {
      _batteryTopic?.unsubscribe();
      _chargingStateTopic?.unsubscribe();
    } catch (_) {}
    try {
      await _ros?.close();
    } catch (_) {}
    _ros = null;
    _last = null;
    _history.clear();
  }

  @override
  void dispose() {
    try {
      _batteryCtrl.close();
    } catch (_) {}
    try {
      _historyCtrl.close();
    } catch (_) {}
    close();
    super.dispose();
  }

  // ------------------------------------------------------------
  // 内部：サンプルのマージ＆履歴管理
  // ------------------------------------------------------------
  void _handleIncomingSample(BatterySample incoming) {
    // 直近の値とマージ（未指定フィールドは既存値を保持）
    final merged = _merge(_last, incoming);
    _last = merged;

    // 最新サンプルを単発ストリームに流す
    _batteryCtrl.add(merged);

    // 履歴に追加（グラフは UI 側で時間窓を切り抜くが、履歴は軽量保持）
    _history.add(merged);

    // 履歴の上限（古過ぎるものを間引く）。ここでは最大 24h だけ保持。
    final cutoff = DateTime.now().subtract(const Duration(hours: 24));
    while (_history.isNotEmpty && _history.first.t.isBefore(cutoff)) {
      _history.removeAt(0);
    }

    // リストのコピーを流す（参照のまま渡さない）
    _historyCtrl.add(List<BatterySample>.unmodifiable(_history));
  }

  BatterySample _merge(BatterySample? prev, BatterySample cur) {
    if (prev == null) return cur;
    return BatterySample(
      t: cur.t, // タイムスタンプは新しいもの
      percentage: cur.percentage ?? prev.percentage,
      voltage: cur.voltage ?? prev.voltage,
      current: cur.current ?? prev.current,
      charge: cur.charge ?? prev.charge,
      capacity: cur.capacity ?? prev.capacity,
      powerSupplyStatus: cur.powerSupplyStatus ?? prev.powerSupplyStatus,
      chargingStateCode: cur.chargingStateCode ?? prev.chargingStateCode,
    );
  }
}
