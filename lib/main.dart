import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:roslibdart/roslibdart.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ROS Robot Controller',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        useMaterial3: true,
      ),
      home: const RobotControlPage(),
    );
  }
}

class RobotControlPage extends StatefulWidget {
  const RobotControlPage({super.key});
  @override
  State<RobotControlPage> createState() => _RobotControlPageState();
}

class _RobotControlPageState extends State<RobotControlPage> {
  // ROS connection
  late Ros _ros;
  Topic? _cmdVel;
  bool _connected = false;

  // You can change this from the UI
  String _ipAddress = '192.168.1.100';
  final TextEditingController _ipController = TextEditingController();

  // Movement parameters
  double _linearSpeed = 0.2; // m/s
  double _angularSpeed = 0.5; // rad/s

  // Battery status (if available)
  double _batteryLevel = 0.0;

  // Repeat sender for D-pad
  Timer? _repeatTimer;

  @override
  void initState() {
    super.initState();
    _ipController.text = _ipAddress;
    _initRos();
  }

  @override
  void dispose() {
    // 念のため停止を送ってからクローズ
    try {
      _publishCmdVel(0, 0);
    } catch (_) {}
    _repeatTimer?.cancel();
    try {
      _ros.close();
    } catch (_) {}
    _ipController.dispose();
    super.dispose();
  }

  // 接続処理
  void _initRos() {
    _connected = false;
    _cmdVel = null;

    _ros = Ros(url: 'ws://$_ipAddress:9090');

    try {
      _ros.connect(); // 戻り値は void
      // 接続直後にトピックをセットアップ
      _setupTopics();
      if (!mounted) return;
      setState(() => _connected = true);
    } catch (e) {
      if (!mounted) return;
      setState(() => _connected = false);
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Failed to connect: $e')),
      );
    }
  }

  void _setupTopics() {
    // /cmd_vel publisher
    final cmd = Topic(
      ros: _ros,
      name: '/cmd_vel',
      type: 'geometry_msgs/Twist',
      queueSize: 10,
    );
    cmd.advertise();
    _cmdVel = cmd;

    // バッテリー購読（存在すれば受け取る）
    final batteryTopic = Topic(
      ros: _ros,
      name: '/battery_state',
      type: 'sensor_msgs/BatteryState',
      queueSize: 1,
    );

    try {
      batteryTopic.subscribe((message) {
        final p = message['percentage'];
        if (p != null) {
          final val = (p is int) ? p.toDouble() : (p as num).toDouble();
          setState(() => _batteryLevel = val > 1.0 ? val / 100.0 : val);
        }
        return Future.value();
      });
    } catch (_) {}
  }

  void _publishCmdVel(double linear, double angular) {
    if (!_connected || _cmdVel == null) return;

    final twistMsg = {
      'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
      'angular': {'x': 0.0, 'y': 0.0, 'z': angular},
    };

    try {
      _cmdVel!.publish(twistMsg);
    } catch (_) {}
  }

  // --- D-Pad: 押している間は 10Hz で送信 ---
  void _startRepeating(double linear, double angular) {
    _repeatTimer?.cancel();
    _publishCmdVel(linear, angular); // すぐ一発
    _repeatTimer =
        Timer.periodic(const Duration(milliseconds: 100), (_) => _publishCmdVel(linear, angular));
  }

  void _stopRepeating() {
    _repeatTimer?.cancel();
    _repeatTimer = null;
    _handleStop();
  }

  // Stop the robot
  void _handleStop() {
    _publishCmdVel(0.0, 0.0);
  }

  // Joystickからの入力を処理
  void _handleJoystickChangedFromDetails(StickDragDetails details) {
    final distance = sqrt(details.x * details.x + details.y * details.y).clamp(0.0, 1.0);
    final angleRadians = atan2(details.y, details.x);
    final angleDegrees = angleRadians * 180 / pi;
    final adjustedDegrees = angleDegrees + 90;
    _handleJoystickChanged(adjustedDegrees, distance);
  }

  void _handleJoystickChanged(double degrees, double distance) {
    final angleRadians = (90 - degrees) * (pi / 180);
    final linear = _linearSpeed * distance * cos(angleRadians);
    final angular = _angularSpeed * distance * sin(angleRadians);
    _publishCmdVel(linear, angular);
  }

  void _updateIpAddress() {
    final newIp = _ipController.text.trim();
    if (newIp.isEmpty) return;

    setState(() => _ipAddress = newIp);

    try {
      _repeatTimer?.cancel();
      _publishCmdVel(0, 0);
      _ros.close();
    } catch (_) {}
    _initRos();
  }

  Widget _buildDirectionPad() {
    const iconSize = 40.0;
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        GestureDetector(
          onTapDown: (_) => _startRepeating(_linearSpeed, 0),
          onTapUp: (_) => _stopRepeating(),
          onTapCancel: _stopRepeating,
          child: const Icon(Icons.arrow_upward, size: iconSize),
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            GestureDetector(
              onTapDown: (_) => _startRepeating(0, _angularSpeed),
              onTapUp: (_) => _stopRepeating(),
              onTapCancel: _stopRepeating,
              child: const Icon(Icons.arrow_back, size: iconSize),
            ),
            const SizedBox(width: 50),
            GestureDetector(
              onTapDown: (_) => _startRepeating(0, -_angularSpeed),
              onTapUp: (_) => _stopRepeating(),
              onTapCancel: _stopRepeating,
              child: const Icon(Icons.arrow_forward, size: iconSize),
            ),
          ],
        ),
        GestureDetector(
          onTapDown: (_) => _startRepeating(-_linearSpeed, 0),
          onTapUp: (_) => _stopRepeating(),
          onTapCancel: _stopRepeating,
          child: const Icon(Icons.arrow_downward, size: iconSize),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('ROS Robot Controller'),
        backgroundColor: _connected ? Colors.green : Colors.red,
      ),
      body: Column(
        children: [
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Row(
              children: [
                Expanded(
                  child: TextField(
                    controller: _ipController,
                    decoration: const InputDecoration(
                      labelText: 'ROS Bridge IP Address',
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
                const SizedBox(width: 8.0),
                ElevatedButton(
                  onPressed: _updateIpAddress,
                  child: const Text('Connect'),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 8.0),
            child: Text(
              'Status: ${_connected ? 'Connected' : 'Disconnected'}',
              style: TextStyle(
                fontSize: 18,
                fontWeight: FontWeight.bold,
                color: _connected ? Colors.green : Colors.red,
              ),
            ),
          ),
          if (_batteryLevel > 0)
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 8.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Icon(Icons.battery_full),
                  const SizedBox(width: 8.0),
                  Text(
                    'Battery: ${(_batteryLevel * 100).toInt()}%',
                    style: const TextStyle(fontSize: 16),
                  ),
                ],
              ),
            ),
          const SizedBox(height: 16.0),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: Row(
              children: [
                const Text('Linear Speed:'),
                Expanded(
                  child: Slider(
                    value: _linearSpeed,
                    min: 0.05,
                    max: 0.5,
                    divisions: 9,
                    label: _linearSpeed.toStringAsFixed(2),
                    onChanged: (value) => setState(() => _linearSpeed = value),
                  ),
                ),
                Text('${_linearSpeed.toStringAsFixed(2)} m/s'),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: Row(
              children: [
                const Text('Angular Speed:'),
                Expanded(
                  child: Slider(
                    value: _angularSpeed,
                    min: 0.1,
                    max: 1.0,
                    divisions: 9,
                    label: _angularSpeed.toStringAsFixed(2),
                    onChanged: (value) => setState(() => _angularSpeed = value),
                  ),
                ),
                Text('${_angularSpeed.toStringAsFixed(2)} rad/s'),
              ],
            ),
          ),
          const SizedBox(height: 16.0),
          Expanded(
            child: Row(
              children: [
                Expanded(child: _buildDirectionPad()),
                Expanded(
                  child: Center(
                    child: GestureDetector(
                      onPanEnd: (_) => _handleStop(),
                      child: Joystick(
                        listener: (StickDragDetails details) =>
                            _handleJoystickChangedFromDetails(details),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: ElevatedButton.icon(
              onPressed: _handleStop,
              icon: const Icon(Icons.stop_circle, color: Colors.red),
              label: const Text('EMERGENCY STOP', style: TextStyle(fontSize: 18)),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.red.shade50,
                foregroundColor: Colors.red.shade900,
                minimumSize: const Size.fromHeight(60),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
