import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / 'ur7e_tools' / 'workcell_ui.py'


def load_routing_symbols():
    tree = ast.parse(UI.read_text(encoding='utf-8'))
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if any(name in {'RIGHT_OPEN_LOOP_TASKS'} for name in names):
                selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name == 'direct_experiment_launch_spec':
            selected.append(node)
    module = ast.Module(body=selected, type_ignores=[])
    ns = {}
    exec(compile(module, str(UI), 'exec'), ns)
    return ns


class TestLeftDirectUIRouting(unittest.TestCase):
    def test_right_tasks_route_to_canonical_backend(self):
        fn = load_routing_symbols()['direct_experiment_launch_spec']
        for label, task in [('Compound', 'compound'), ('Polishing', 'polishing'), ('Full', 'full')]:
            with self.subTest(label=label):
                spec = fn((label, 'Right', 'Open-loop', 'Task Space'), False)
                self.assertEqual(spec['module'], 'experiment_backend.right_hand')
                self.assertEqual(spec['task'], task)
                self.assertEqual(spec['mode'], 'simulation')

    def test_left_only_full_routes_to_single_protocol(self):
        fn = load_routing_symbols()['direct_experiment_launch_spec']
        full = fn(('Full', 'Left', 'Open-loop', 'Task Space'), True)
        self.assertEqual(full['module'], 'experiment_backend.left_hand_direct')
        self.assertEqual(full['protocol'], 'single')
        self.assertEqual(full['mode'], 'real')
        self.assertIsNone(fn(('Compound', 'Left', 'Open-loop', 'Task Space'), True))
        self.assertIsNone(fn(('Polishing', 'Left', 'Open-loop', 'Task Space'), True))

    def test_analysis_browser_includes_direct_left_trials(self):
        text = UI.read_text(encoding='utf-8')
        self.assertIn('project_root / "results" / "direct_left"', text)


if __name__ == '__main__':
    unittest.main()
