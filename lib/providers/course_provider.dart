import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

class CourseProvider with ChangeNotifier {
  List<Map<String, dynamic>> _evaluations = [];

  List<Map<String, dynamic>> get evaluations => _evaluations;

  CourseProvider() {
    loadEvaluations();
  }

  Future<void> loadEvaluations() async {
    final prefs = await SharedPreferences.getInstance();
    final evaluationsJson = prefs.getString('evaluations');
    if (evaluationsJson != null) {
      try {
        _evaluations = List<Map<String, dynamic>>.from(json.decode(evaluationsJson));
      } catch (e) {
        _evaluations = [];
      }
    }
    notifyListeners();
  }

  Future<void> addEvaluation(String courseId, double rating, String comment) async {
    // Create a new evaluation record with a timestamp
    final evaluation = {
      'courseId': courseId,
      'rating': rating,
      'comment': comment,
      'timestamp': DateTime.now().toIso8601String(),
    };
    _evaluations.add(evaluation);
    await saveEvaluations();
    notifyListeners();
  }

  Future<void> saveEvaluations() async {
    final prefs = await SharedPreferences.getInstance();
    final evaluationsJson = json.encode(_evaluations);
    await prefs.setString('evaluations', evaluationsJson);
  }
}
