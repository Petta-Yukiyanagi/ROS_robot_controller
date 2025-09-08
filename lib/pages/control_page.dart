// lib/pages/control_page.dart
import 'dart:math' as math;
import 'package:flutter/material.dart';

import '../services/ros_service.dart';
import '../widgets/video_panel.dart';
import '../widgets/joystick_pad.dart';

class ControlPage extends StatefulWidget {
  final RosService ros;
  const ControlPage({super.key, required this.ros});

  @override
  State<ControlPage> createState() => _ControlPageState();
}

class _ControlPageState extends State<ControlPage> {
  final _hostCtl = TextEditingController(text: '192.168.10.115');
  final _portCtl = TextEditingController(text: '9090');

  // 2本同時視聴用URL
  final _url1Ctl = TextEditingController(text: 'rtsp://192.168.10.115:8555/usb');
  final _url2Ctl = TextEditingController(text: 'rtsp://192.168.10.115:8555/pi');

  // 小型コントローラ（速度スケール）
  double _linear = 0.2;   // m/s
  double _angular = 0.5;  // rad/s

  @override
  void dispose() {
    _hostCtl.dispose();
    _portCtl.dispose();
    _url1Ctl.dispose();
    _url2Ctl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      resizeToAvoidBottomInset: true,
      appBar: AppBar(
        title: const Text('Control'),
        backgroundColor: widget.ros.connected ? Colors.green : Colors.red,
        actions: [
          IconButton(
            icon: const Icon(Icons.link),
            tooltip: 'Connect',
            onPressed: () async {
              final host = _hostCtl.text.trim();
              final port = int.tryParse(_portCtl.text.trim()) ?? 9090;
              try {
                await widget.ros.connect(host: host, port: port);
                if (!mounted) return;
                ScaffoldMessenger.of(context).showSnackBar(
                  SnackBar(content: Text(widget.ros.connected ? 'Connected' : 'Failed to connect')),
                );
                setState(() {});
              } catch (e) {
                if (!mounted) return;
                ScaffoldMessenger.of(context).showSnackBar(
                  SnackBar(content: Text('ROS connect error: $e')),
                );
              }
            },
          ),
          IconButton(
            icon: const Icon(Icons.stop_circle),
            tooltip: 'STOP',
            onPressed: widget.ros.stop,
          ),
        ],
      ),

      // ───────────────────────────────────────────────────────────
      // 本文 + 右下固定ジョイスティック（オーバーレイ）
      // ───────────────────────────────────────────────────────────
      body: Stack(
        children: [
          // === メイン本文（スクロール領域：ジョイスティック除く） ===
          SafeArea(
            child: LayoutBuilder(
              builder: (context, c) {
                final screenH = c.maxHeight;
                final screenW = c.maxWidth;
                final isWide = screenW >= 720 || screenW > screenH;

                // オーバーフロー対策：各映像の最大高さ
                final videoMaxH = isWide
                    ? math.max(160.0, screenH * 0.40)
                    : math.max(140.0, screenH * 0.26);

                return SingleChildScrollView(
                  padding: EdgeInsets.fromLTRB(
                    12, 12, 12,
                    // 右下の固定ジョイスティックと被らないように少し余白を足す
                    MediaQuery.of(context).viewInsets.bottom + 240,
                  ),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      // ===== ROS 接続 =====
                      Wrap(
                        spacing: 8,
                        runSpacing: 8,
                        crossAxisAlignment: WrapCrossAlignment.center,
                        children: [
                          SizedBox(
                            width: 220,
                            child: TextField(
                              controller: _hostCtl,
                              decoration: const InputDecoration(labelText: 'ROS host (ws)'),
                            ),
                          ),
                          SizedBox(
                            width: 120,
                            child: TextField(
                              controller: _portCtl,
                              decoration: const InputDecoration(labelText: 'Port'),
                              keyboardType: TextInputType.number,
                            ),
                          ),
                          Text(
                            widget.ros.connected ? 'Connected' : 'Disconnected',
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                              color: widget.ros.connected ? Colors.green : Colors.red,
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(height: 8),

                      // ===== Stream URL（host/portの直下）=====
                      Wrap(
                        spacing: 8,
                        runSpacing: 8,
                        children: [
                          SizedBox(
                            width: math.min(480, screenW - 24),
                            child: TextField(
                              controller: _url1Ctl,
                              decoration: const InputDecoration(labelText: 'Stream #1 URL (RTSP)'),
                              onChanged: (_) => setState(() {}),
                            ),
                          ),
                          SizedBox(
                            width: math.min(480, screenW - 24),
                            child: TextField(
                              controller: _url2Ctl,
                              decoration: const InputDecoration(labelText: 'Stream #2 URL (RTSP)'),
                              onChanged: (_) => setState(() {}),
                            ),
                          ),
                        ],
                      ),

                      const SizedBox(height: 10),

                      // ===== 速度スライダー（コンパクト） =====
                      SliderTheme(
                        data: SliderTheme.of(context).copyWith(
                          trackHeight: 2,
                          thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 8),
                          overlayShape: const RoundSliderOverlayShape(overlayRadius: 14),
                        ),
                        child: Column(
                          children: [
                            Row(
                              children: [
                                const SizedBox(width: 70, child: Text('Linear', textAlign: TextAlign.right)),
                                Expanded(
                                  child: Slider(
                                    value: _linear, min: 0.05, max: 0.5, divisions: 9,
                                    label: _linear.toStringAsFixed(2),
                                    onChanged: (v) => setState(() => _linear = v),
                                  ),
                                ),
                                SizedBox(
                                  width: 42,
                                  child: Text(_linear.toStringAsFixed(2), textAlign: TextAlign.right),
                                ),
                              ],
                            ),
                            Row(
                              children: [
                                const SizedBox(width: 70, child: Text('Angular', textAlign: TextAlign.right)),
                                Expanded(
                                  child: Slider(
                                    value: _angular, min: 0.1, max: 1.0, divisions: 9,
                                    label: _angular.toStringAsFixed(2),
                                    onChanged: (v) => setState(() => _angular = v),
                                  ),
                                ),
                                SizedBox(
                                  width: 42,
                                  child: Text(_angular.toStringAsFixed(2), textAlign: TextAlign.right),
                                ),
                              ],
                            ),
                          ],
                        ),
                      ),

                      const SizedBox(height: 10),

                      // ===== 2ストリーム同時表示 =====
                      LayoutBuilder(
                        builder: (_, b) {
                          final sideBySide = b.maxWidth >= 640;
                          final children = [
                            _VideoBox(url: _url1Ctl.text.trim(), maxH: videoMaxH),
                            _VideoBox(url: _url2Ctl.text.trim(), maxH: videoMaxH),
                          ];
                          return sideBySide
                              ? Row(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Expanded(child: children[0]),
                                    const SizedBox(width: 10),
                                    Expanded(child: children[1]),
                                  ],
                                )
                              : Column(
                                  children: [
                                    children[0],
                                    const SizedBox(height: 10),
                                    children[1],
                                  ],
                                );
                        },
                      ),
                    ],
                  ),
                );
              },
            ),
          ),

          // === 右下固定ジョイスティック（オーバーレイ） ===
          Positioned(
            right: 16,
            bottom: 16 + MediaQuery.of(context).padding.bottom,
            child: IgnorePointer(
              ignoring: false, // ← ジョイスティックは操作可能
              child: SizedBox(
                width: 220,
                height: 220,
                child: Material(
                  elevation: 6,
                  color: Colors.transparent,
                  borderRadius: BorderRadius.circular(16),
                  child: JoystickPad(
                    onCmd: widget.ros.publishCmdVel,
                    onStop: widget.ros.stop,
                    linearSpeed: _linear,
                    angularSpeed: _angular,
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _VideoBox extends StatelessWidget {
  final String url;
  final double maxH;
  const _VideoBox({required this.url, required this.maxH});

  @override
  Widget build(BuildContext context) {
    return ConstrainedBox(
      constraints: BoxConstraints(maxHeight: maxH),
      child: AspectRatio(
        aspectRatio: 4 / 3,
        child: VideoPanel(url: url), // ← HwAcc は VideoPanel 内で指定
      ),
    );
  }
}
