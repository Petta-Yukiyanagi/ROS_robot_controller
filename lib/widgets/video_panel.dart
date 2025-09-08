import 'package:flutter/material.dart';
import 'package:flutter_vlc_player/flutter_vlc_player.dart';

class VideoPanel extends StatefulWidget {
  final String url;
  const VideoPanel({super.key, required this.url});

  @override
  State<VideoPanel> createState() => _VideoPanelState();
}

class _VideoPanelState extends State<VideoPanel>
    with AutomaticKeepAliveClientMixin {
  VlcPlayerController? _ctrl;
  String _currentUrl = '';

  Future<void> _create(String url) async {
    _currentUrl = url;
    _ctrl = VlcPlayerController.network(
      url,
      hwAcc: HwAcc.full,
      autoPlay: true,
      options: VlcPlayerOptions(
        rtp: VlcRtpOptions([":rtsp-tcp"]), // RTSPはTCPで安定化
        // 必要なら遅延バッファを少し確保（コメント解除）
        // advanced: VlcAdvancedOptions([VlcAdvancedOptions.networkCaching(200)]),
      ),
    );
    if (mounted) setState(() {}); // VlcPlayerをツリーに乗せる
  }

  Future<void> _switch(String url) async {
    if (_ctrl == null) {
      await _create(url);
      return;
    }
    if (url == _currentUrl) return;

    _currentUrl = url;
    final old = _ctrl;
    _ctrl = null;
    if (mounted) setState(() {}); // いったんローディング表示へ

    try { await old?.stop(); } catch (_) {}
    try { await old?.dispose(); } catch (_) {}

    await _create(url);
  }

  @override
  void initState() {
    super.initState();
    if (widget.url.isNotEmpty) {
      _create(widget.url);
    }
  }

  @override
  void didUpdateWidget(covariant VideoPanel oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.url.isNotEmpty) {
      _switch(widget.url);
    }
  }

  @override
  void dispose() {
    try { _ctrl?.stop(); } catch (_) {}
    try { _ctrl?.dispose(); } catch (_) {}
    _ctrl = null;
    super.dispose();
  }

  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);

    if (widget.url.isEmpty) {
      return const Center(child: Text('No stream URL'));
    }

    // ★ isInitialized では判定しない（VlcPlayerが自分で初期化を進める）
    if (_ctrl == null) {
      return const Center(child: CircularProgressIndicator());
    }

    return ClipRRect(
      borderRadius: BorderRadius.circular(10),
      child: VlcPlayer(
        key: ValueKey(_currentUrl), // URLが変わったらネイティブビューも作り直す
        controller: _ctrl!,
        aspectRatio: 4 / 3,
        placeholder: const Center(child: CircularProgressIndicator()),
      ),
    );
  }
}
