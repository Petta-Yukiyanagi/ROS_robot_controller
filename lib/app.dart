import 'package:flutter/material.dart';
import 'services/ros_service.dart' as ros;
import 'pages/control_page.dart' as ctrl;
import 'pages/battery_page.dart' as bat;

class App extends StatefulWidget {
  const App({super.key});
  @override
  State<App> createState() => _AppState();
}

class _AppState extends State<App> {
  int _index = 0;
  late final ros.RosService _rosService;
  late final List<Widget> _pages;

  @override
  void initState() {
    super.initState();
    _rosService = ros.RosService();
    _pages = [
      ctrl.ControlPage(ros: _rosService),
      bat.BatteryPage(ros: _rosService),
    ];
  }

  @override
  void dispose() {
    _rosService.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ROS Robot Controller',
      theme: ThemeData(useMaterial3: true, colorSchemeSeed: Colors.blue),
      home: Scaffold(
        body: IndexedStack(
          index: _index,
          children: _pages,
        ),
        bottomNavigationBar: NavigationBar(
          selectedIndex: _index,
          destinations: const [
            NavigationDestination(icon: Icon(Icons.sports_esports), label: 'Control'),
            NavigationDestination(icon: Icon(Icons.battery_full), label: 'Battery'),
          ],
          onDestinationSelected: (i) => setState(() => _index = i),
        ),
      ),
      debugShowCheckedModeBanner: false,
    );
  }
}
