import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/course_provider.dart';

class AddEvaluationScreen extends StatefulWidget {
  const AddEvaluationScreen({super.key});

  @override
  State<AddEvaluationScreen> createState() => _AddEvaluationScreenState();
}

class _AddEvaluationScreenState extends State<AddEvaluationScreen> {
  final _formKey = GlobalKey<FormState>();
  final TextEditingController _courseIdController = TextEditingController();
  final TextEditingController _commentController = TextEditingController();
  double _rating = 3.0;

  @override
  void dispose() {
    _courseIdController.dispose();
    _commentController.dispose();
    super.dispose();
  }

  void _submit() {
    if (!_formKey.currentState!.validate()) return;

    // Providerへの書き込みは listen 不要なので context.read を使う
    context.read<CourseProvider>().addEvaluation(
          _courseIdController.text.trim(),
          _rating,
          _commentController.text.trim(),
        );

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('評価を保存しました')),
    );

    Navigator.of(context).pop(true); // 必要なら true を返してリロードトリガに
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Add Evaluation')),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Form(
          key: _formKey,
          child: Column(
            children: [
              TextFormField(
                controller: _courseIdController,
                decoration: const InputDecoration(
                  labelText: 'Course ID',
                  border: OutlineInputBorder(),
                ),
                textInputAction: TextInputAction.next,
                validator: (v) =>
                    (v == null || v.trim().isEmpty) ? 'Course ID を入力してください' : null,
              ),
              const SizedBox(height: 12),
              TextFormField(
                controller: _commentController,
                decoration: const InputDecoration(
                  labelText: 'Comment (任意)',
                  border: OutlineInputBorder(),
                ),
                minLines: 2,
                maxLines: 4,
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  const Text('Rating'),
                  Expanded(
                    child: Slider(
                      value: _rating,
                      min: 1,
                      max: 5,
                      divisions: 4,
                      label: _rating.toStringAsFixed(1),
                      onChanged: (value) => setState(() => _rating = value),
                    ),
                  ),
                  Text(_rating.toStringAsFixed(1)),
                ],
              ),
              const SizedBox(height: 8),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton(
                  onPressed: _submit,
                  child: const Text('Submit Evaluation'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
