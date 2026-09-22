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
            if 'DIRECT_EXPERIMENT_BACKENDS' in names:
                selected.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name == 'direct_experiment_launch_spec':
            selected.append(node)
    module = ast.Module(body=selected, type_ignores=[])
    ns = {}
    exec(compile(module, str(UI), 'exec'), ns)
    return ns


class TestLeftDirectUIRouting(unittest.TestCase):
    def test_right_routing_is_unchanged(self):
        fn = load_routing_symbols()['direct_experiment_launch_spec']
        one = fn(('Full Direct - One Rep', 'Right', 'Open-loop', 'Task Space'), False)
        demo = fn(('Full Direct - Demo', 'Right', 'Open-loop', 'Task Space'), True)
        self.assertEqual(one['module'], 'experiment_backend.right_hand_direct_single')
        self.assertEqual(demo['module'], 'experiment_backend.right_hand_direct_demo')

    def test_both_left_task_labels_route_to_same_backend(self):
        fn = load_routing_symbols()['direct_experiment_launch_spec']
        one = fn(('Full Direct - One Rep', 'Left', 'Open-loop', 'Task Space'), False)
        demo = fn(('Full Direct - Demo', 'Left', 'Open-loop', 'Task Space'), True)
        self.assertEqual(one['module'], 'experiment_backend.left_hand_direct')
        self.assertEqual(demo['module'], 'experiment_backend.left_hand_direct')
        self.assertEqual(one['protocol'], 'single')
        self.assertEqual(demo['protocol'], 'demo')

    def test_analysis_browser_includes_direct_left_trials(self):
        text = UI.read_text(encoding='utf-8')
        self.assertIn('project_root / "results" / "direct_left"', text)


if __name__ == '__main__':
    unittest.main()
