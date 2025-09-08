import 'package:flutter/foundation.dart' show kIsWeb;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:video_player/video_player.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'package:flutter_vlc_player/flutter_vlc_player.dart';

class VideoPanel extends StatefulWidget {
  final String defaultUrl1;
  final String defaultUrl2;
  const VideoPanel({super.key, required this.defaultUrl1, required this.defaultUrl2});

  @override
  State<VideoPanel> createState() => _VideoPanelState();
}

enum _ViewMode { split, leftOnly, rightOnly }

class _VideoPanelState extends State<VideoPanel> {
  static const _kUrl1 = 'video_panel_url1';
  static const _kUrl2 = 'video_panel_url2';

  late final TextEditingController _url1 = TextEditingController(text: widget.defaultUrl1);
  late final TextEditingController _url2 = TextEditingController(text: widget.defaultUrl2);

  _ViewMode _mode = _ViewMode.split;

  @override
  void initState() {
    super.initState();
    _loadPrefs();
  }

  Future<void> _loadPrefs() async {
    final p = await SharedPreferences.getInstance();
    setState(() {
      _url1.text = p.getString(_kUrl1) ?? _url1.text;
      _url2.text = p.getString(_kUrl2) ?? _url2.text;
    });
  }

  Future<void> _savePrefs() async {
    final p = await SharedPreferences.getInstance();
    await p.setString(_kUrl1, _url1.text.trim());
    await p.setString(_kUrl2, _url2.text.trim());
  }

  @override
  void dispose() {
    _url1.dispose();
    _url2.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final url1 = _url1.text.trim();
    final url2 = _url2.text.trim();

    return Column(
      children: [
        // 入力＋操作列
        Row(
          children: [
            Expanded(
              child: TextField(
                controller: _url1,
                decoration: const InputDecoration(labelText: 'Stream #1 URL (rtsp / m3u8 / mjpg)'),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: TextField(
                controller: _url2,
                decoration: const InputDecoration(labelText: 'Stream #2 URL (rtsp / m3u8 / mjpg)'),
              ),
            ),
            const SizedBox(width: 8),
            FilledButton.icon(
              onPressed: () async { await _savePrefs(); setState(() {}); },
              icon: const Icon(Icons.play_arrow),
              label: const Text('Play'),
            ),
          ],
        ),
        const SizedBox(height: 8),

        // 表示切替
        Align(
          alignment: Alignment.centerLeft,
          child: ToggleButtons(
            borderRadius: BorderRadius.circular(8),
            isSelected: [
              _mode == _ViewMode.split,
              _mode == _ViewMode.leftOnly,
              _mode == _ViewMode.rightOnly,
            ],
            onPressed: (i) {
              setState(() {
                _mode = [_ViewMode.split, _ViewMode.leftOnly, _ViewMode.rightOnly][i];
              });
            },
            children: const [
              Padding(padding: EdgeInsets.symmetric(horizontal: 12), child: Text('Split')),
              Padding(padding: EdgeInsets.symmetric(horizontal: 12), child: Text('#1のみ')),
              Padding(padding: EdgeInsets.symmetric(horizontal: 12), child: Text('#2のみ')),
            ],
          ),
        ),
        const SizedBox(height: 8),

        // ビデオ領域
        Expanded(
          child: switch (_mode) {
            _ViewMode.leftOnly  => _FullscreenableBox(url: url1),
            _ViewMode.rightOnly => _FullscreenableBox(url: url2),
            _                 => Row(
              children: [
                Expanded(child: _FullscreenableBox(url: url1)),
                const SizedBox(width: 8),
                Expanded(child: _FullscreenableBox(url: url2)),
              ],
            ),
          },
        ),
      ],
    );
  }
}

enum _SourceType { rtsp, hls, mjpeg, unknown }

_SourceType _detect(String url) {
  final u = url.toLowerCase();
  if (u.startsWith('rtsp://')) return _SourceType.rtsp;
  if (u.contains('.m3u8')) return _SourceType.hls;
  if (u.endsWith('.mjpg') || u.endsWith('.mjpeg') || u.contains('mjpeg')) return _SourceType.mjpeg;
  return _SourceType.unknown;
}

/// タップで全画面に遷移できるラッパ
class _FullscreenableBox extends StatelessWidget {
  final String url;
  const _FullscreenableBox({required this.url});

  @override
  Widget build(BuildContext context) {
    if (url.isEmpty) return const _Hint('URL を入力してください');

    return GestureDetector(
      onTap: () {
        Navigator.of(context).push(MaterialPageRoute(builder: (_) => _FullscreenPage(url: url)));
      },
      child: _PlayerBox(url: url),
    );
  }
}

class _PlayerBox extends StatelessWidget {
  final String url;
  const _PlayerBox({required this.url});

  @override
  Widget build(BuildContext context) {
    if (url.isEmpty) return const _Hint('URL を入力してください');
    final kind = _detect(url);

    if (kIsWeb) {
      // Webは RTSP 非対応。HLS or MJPEG を再生
      switch (kind) {
        case _SourceType.hls:
          return _VideoPlayerTile(url: url);
        case _SourceType.mjpeg:
          return _MjpegTile(url: url);
        case _SourceType.rtsp:
          return const _Hint('Web では RTSP は再生できません（HLS または MJPEG を使用）');
        default:
          return const _Hint('拡張子から判別できません（.m3u8 / .mjpg 推奨）');
      }
    }

    // Android / iOS / Windows
    switch (kind) {
      case _SourceType.rtsp:
        return _VlcTile(url: url);
      case _SourceType.hls:
        return _VideoPlayerTile(url: url);
      case _SourceType.mjpeg:
        return _MjpegTile(url: url);
      default:
        return const _Hint('URL の種類を判別できません（rtsp / m3u8 / mjpg のいずれかで）');
    }
  }
}

class _MjpegTile extends StatelessWidget {
  final String url;
  const _MjpegTile({required this.url});
  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(12),
      child: Mjpeg(
        isLive: true, stream: url, fit: BoxFit.cover,
        error: (ctx, err, st) => _Hint('MJPEG 再生エラー: $err'),
      ),
    );
  }
}

class _VideoPlayerTile extends StatefulWidget {
  final String url;
  const _VideoPlayerTile({required this.url});

  @override
  State<_VideoPlayerTile> createState() => _VideoPlayerTileState();
}

class _VideoPlayerTileState extends State<_VideoPlayerTile> {
  VideoPlayerController? _c;

  @override
  void initState() {
    super.initState();
    _init();
  }

  @override
  void didUpdateWidget(covariant _VideoPlayerTile oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.url != widget.url) {
      _disposeCtrl();
      _init();
    }
  }

  Future<void> _init() async {
    _c = VideoPlayerController.networkUrl(Uri.parse(widget.url))
      ..setLooping(true);
    await _c!.initialize();
    if (!mounted) return;
    setState(() {});
    _c?.play();
  }

  void _disposeCtrl() {
    _c?.dispose();
    _c = null;
  }

  @override
  void dispose() { _disposeCtrl(); super.dispose(); }

  @override
  Widget build(BuildContext context) {
    if (_c == null || !_c!.value.isInitialized) {
      return const Center(child: CircularProgressIndicator());
    }
    if (_c!.value.hasError) {
      return _Hint('Video error: ${_c!.value.errorDescription}');
    }
    final ar = _c!.value.aspectRatio == 0 ? (16 / 9) : _c!.value.aspectRatio;
    return ClipRRect(
      borderRadius: BorderRadius.circular(12),
      child: AspectRatio(aspectRatio: ar, child: VideoPlayer(_c!)),
    );
  }
}

class _VlcTile extends StatefulWidget {
  final String url;
  const _VlcTile({required this.url});

  @override
  State<_VlcTile> createState() => _VlcTileState();
}

class _VlcTileState extends State<_VlcTile> {
  VlcPlayerController? _c;

  @override
  void initState() {
    super.initState();
    _createController(widget.url);
  }

  @override
  void didUpdateWidget(covariant _VlcTile oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.url != widget.url) {
      _disposeCtrl();
      _createController(widget.url);
    }
  }

  void _createController(String url) {
    _c = VlcPlayerController.network(
      url,
      autoPlay: true,
      hwAcc: HwAcc.auto,
      options: VlcPlayerOptions(
        advanced: VlcAdvancedOptions([
          VlcAdvancedOptions.networkCaching(600),
        ]),
        rtp: VlcRtpOptions([
          VlcRtpOptions.rtpOverRtsp(true), // TCP優先
        ]),
      ),
    );
  }

  void _disposeCtrl() {
    _c?.stop();
    _c?.dispose();
    _c = null;
  }

  @override
  void dispose() {
    _disposeCtrl();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return ClipRRect(
      borderRadius: BorderRadius.circular(12),
      child: AspectRatio(
        aspectRatio: 16 / 9,
        child: _c == null
            ? const Center(child: CircularProgressIndicator())
            : VlcPlayer(
                controller: _c!,
                aspectRatio: 16 / 9,
                placeholder: const Center(child: CircularProgressIndicator()),
              ),
      ),
    );
  }
}

class _Hint extends StatelessWidget {
  final String text;
  const _Hint(this.text);
  @override
  Widget build(BuildContext context) {
    return Card(
      child: Center(
        child: Padding(
          padding: const EdgeInsets.all(12.0),
          child: Text(text, textAlign: TextAlign.center),
        ),
      ),
    );
  }
}

/// 全画面ページ
class _FullscreenPage extends StatefulWidget {
  final String url;
  const _FullscreenPage({required this.url});

  @override
  State<_FullscreenPage> createState() => _FullscreenPageState();
}

class _FullscreenPageState extends State<_FullscreenPage> {
  @override
  void initState() {
    super.initState();
    // 全画面＆ランドスケープ推奨
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }

  @override
  void dispose() {
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    SystemChrome.setPreferredOrientations(DeviceOrientation.values);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Stack(
          children: [
            Center(child: _PlayerBox(url: widget.url)),
            Positioned(
              top: 12, left: 12,
              child: IconButton.filled(
                style: IconButton.styleFrom(backgroundColor: Colors.black54),
                onPressed: () => Navigator.of(context).pop(),
                icon: const Icon(Icons.close),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
