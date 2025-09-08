// lib/widgets/joystick_pad.dart
import 'dart:math' as math;
import 'package:flutter/material.dart';

class JoystickPad extends StatefulWidget {
  const JoystickPad({
    super.key,
    required this.onCmd,
    required this.onStop,
    required this.linearSpeed,
    required this.angularSpeed,
    this.aspectRatio = 1.0,
  });

  /// (linear, angular) を連続発行
  final void Function(double linear, double angular) onCmd;
  final VoidCallback onStop;

  /// 速度スケール
  final double linearSpeed;
  final double angularSpeed;

  /// レイアウト比（正方形でOK）
  final double aspectRatio;

  @override
  State<JoystickPad> createState() => _JoystickPadState();
}

class _JoystickPadState extends State<JoystickPad> {
  Offset _knob = Offset.zero; // -1..1 の正規化オフセット

  void _emit() {
    final lin = (-_knob.dy).clamp(-1.0, 1.0) * widget.linearSpeed; // 前後
    final ang = (_knob.dx).clamp(-1.0, 1.0) * widget.angularSpeed; // 旋回（右＋）
    widget.onCmd(lin, ang);
  }

  void _onPan(Offset localPos, Size size) {
    final cx = size.width / 2.0;
    final cy = size.height / 2.0;
    final dx = localPos.dx - cx;
    final dy = localPos.dy - cy;

    // 半径（少なくとも1pxでNaN回避）
    final maxR = math.max(1.0, math.min(cx, cy));
    var v = Offset(dx / maxR, dy / maxR);

    // 単位円にクランプ
    final len = v.distance;
    if (!len.isFinite || len.isNaN) {
      v = Offset.zero;
    } else if (len > 1.0) {
      v = v / len;
    }

    setState(() => _knob = v);
    _emit();
  }

  void _reset() {
    setState(() => _knob = Offset.zero);
    widget.onStop();
  }

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: widget.aspectRatio,
      child: LayoutBuilder(
        builder: (ctx, bc) {
          final size = Size(bc.maxWidth, bc.maxHeight);
          final radius = math.min(size.width, size.height) / 2.0;

          // ノブをやや大きめに（掴みやすさ重視）
          final knobR = math.max(30.0, radius * 0.35);

          final center = Offset(size.width / 2, size.height / 2);
          final knobCenter =
              center + Offset(_knob.dx * (radius - knobR), _knob.dy * (radius - knobR));

          return GestureDetector(
            behavior: HitTestBehavior.opaque, // 領域全体をタッチ可（中心にシビアじゃない）
            onPanStart: (d) => _onPan(d.localPosition, size),
            onPanUpdate: (d) => _onPan(d.localPosition, size),
            onPanEnd: (_) => _reset(),
            onPanCancel: _reset,
            child: Stack(
              children: [
                // ベース円＋リング
                Positioned.fill(child: CustomPaint(painter: _BasePainter())),
                // ガイド十字
                Positioned.fill(child: CustomPaint(painter: _CrossPainter())),

                // ノブ（影付き・大きめ）
                Positioned(
                  left: (knobCenter.dx - knobR).clamp(0.0, size.width - knobR * 2),
                  top: (knobCenter.dy - knobR).clamp(0.0, size.height - knobR * 2),
                  width: knobR * 2,
                  height: knobR * 2,
                  child: DecoratedBox(
                    decoration: BoxDecoration(
                      color: Colors.blue.shade500.withOpacity(0.95),
                      shape: BoxShape.circle,
                      boxShadow: const [
                        BoxShadow(blurRadius: 12, spreadRadius: 2, color: Colors.black26),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}

class _BasePainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final r = math.min(size.width, size.height) / 2.0;
    final c = Offset(size.width / 2, size.height / 2);

    if (!(r.isFinite && !r.isNaN)) return;

    final fill = Paint()
      ..style = PaintingStyle.fill
      ..shader = RadialGradient(
        colors: [Colors.grey.shade200, Colors.grey.shade300],
      ).createShader(Rect.fromCircle(center: c, radius: r));

    canvas.drawCircle(c, r, fill);

    final ring = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..color = Colors.black12;

    canvas.drawCircle(c, r * 0.33, ring);
    canvas.drawCircle(c, r * 0.66, ring);
    canvas.drawCircle(c, r, ring);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

class _CrossPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final c = Offset(size.width / 2, size.height / 2);
    final ok = [c.dx, c.dy, size.width, size.height].every((v) => v.isFinite && !v.isNaN);
    if (!ok) return;

    final p = Paint()
      ..color = Colors.black12
      ..strokeWidth = 2;

    canvas.drawLine(Offset(c.dx, 0), Offset(c.dx, size.height), p);
    canvas.drawLine(Offset(0, c.dy), Offset(size.width, c.dy), p);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
