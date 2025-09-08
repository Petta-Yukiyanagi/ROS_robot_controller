import 'package:flutter/material.dart';
import 'package:video_player/video_player.dart';
import 'package:chewie/chewie.dart';

class HlsPlayer extends StatefulWidget {
  final String url;
  const HlsPlayer({super.key, required this.url});
  @override State<HlsPlayer> createState() => _HlsPlayerState();
}

class _HlsPlayerState extends State<HlsPlayer> {
  late final VideoPlayerController _video;
  ChewieController? _chewie;
  bool _initing = true;
  String? _error;

  @override void initState() { super.initState(); _init(); }
  Future<void> _init() async {
    try {
      _video = VideoPlayerController.networkUrl(Uri.parse(widget.url));
      await _video.initialize();
      _chewie = ChewieController(
        videoPlayerController: _video,
        autoPlay: true,
        looping: true,
        allowMuting: true,
        allowFullScreen: true,
        aspectRatio: _video.value.aspectRatio == 0 ? 16/9 : _video.value.aspectRatio,
      );
      setState(() => _initing = false);
    } catch (e) { setState(() => _error = e.toString()); }
  }

  @override void dispose() { _chewie?.dispose(); _video.dispose(); super.dispose(); }

  @override Widget build(BuildContext context) {
    if (_error != null) return Center(child: Text('Video error: '));
    if (_initing) return const Center(child: CircularProgressIndicator());
    return Chewie(controller: _chewie!);
  }
}


