import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/course_provider.dart';
import 'add_evaluation_screen.dart';

class CourseEvaluationScreen extends StatelessWidget {
  const CourseEvaluationScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final courseProvider = Provider.of<CourseProvider>(context);
    return Scaffold(
      appBar: AppBar(
        title: const Text('Course Evaluations'),
      ),
      body: courseProvider.evaluations.isEmpty
          ? const Center(child: Text("No evaluations yet."))
          : ListView.builder(
              itemCount: courseProvider.evaluations.length,
              itemBuilder: (context, index) {
                final evaluation = courseProvider.evaluations[index];
                return ListTile(
                  title: Text(evaluation['courseId'] ?? ''),
                  subtitle: Text("Rating: ${evaluation['rating']}, Comment: ${evaluation['comment']}"),
                  trailing: Text(evaluation['timestamp'] ?? ""),
                );
              },
            ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          Navigator.of(context).push(
            MaterialPageRoute(builder: (_) => const AddEvaluationScreen()),
          );
        },
        child: const Icon(Icons.add),
      ),
    );
  }
}
