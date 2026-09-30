import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import test_experiment_play as play_tests
from PySide6.QtWidgets import QMessageBox, QInputDialog
from ur7e_tools import workcell_ui as ui
from ur7e_tools import human_reference_ui as replay_ui


class ParticipantDirectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        play_tests.ExperimentPlayTests.setUpClass.__func__(cls)

    def setUp(self):
        play_tests.ExperimentPlayTests.setUp(self)
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        for gesture in ['compound', 'polishing']:
            ref = self.root / 'P77' / gesture / 'reference'
            ref.mkdir(parents=True)
            (ref / 'robot1_task_space_ik.csv').write_text('time_s,shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint\n0,0,0,0,0,0,0\n1,0,0,0,0,0,0\n')
            (ref / 'tension_summary.json').write_text(json.dumps({
                'scalar_tension_valid': False, 'force_axis': None, 'sign': None,
                'per_rep': [{'rep': 1, **{axis: {'n': 270, 'mean': v} for axis, v in
                    [('Fx_N', 1.96), ('Fy_N', -6.72), ('Fz_N', 3.28)]}}]}))
        self.window.participant_reference_root = lambda: self.root
        # Restore the real participant helper methods overridden by the base UI test.
        self.window.participant_reference_helpers = (
            ui.WorkcellUI.participant_reference_helpers.__get__(self.window, ui.WorkcellUI)
        )
        self.window.participant_reference_config = (
            ui.WorkcellUI.participant_reference_config.__get__(self.window, ui.WorkcellUI)
        )
        self.window.update_experiment_controls()

    def select_participant_variant(self):
        self.window.experiment_task_combo.setCurrentText('Full')
        self.assertEqual(self.window._selected_experiment()[0], 'Full')

    def test_participant_cancel_does_not_confirm_or_launch(self):
        self.select_participant_variant()
        with patch.object(QInputDialog, 'getItem', return_value=('', False)), patch.object(
                QMessageBox, 'question') as confirm, patch.object(self.window.experiment_process, 'start') as start:
            self.window.run_experiment_button.click()
        confirm.assert_not_called()
        start.assert_not_called()
        self.assertIn('participant selection cancelled', self.window.log_output.toPlainText())

    def test_both_hands_select_before_real_confirmation_and_pass_id(self):
        self.select_participant_variant()
        for hand in ['Right', 'Left']:
            with self.subTest(hand=hand):
                self.window.experiment_hand_combo.setCurrentText(hand)
                events = []

                def select(*args):
                    events.append(args[1])
                    if args[1] == 'Select participant':
                        self.assertEqual(args[3], ['P77'])
                        return 'P77', True
                    self.assertEqual(args[1], 'Select start state')
                    self.assertIn('home_home', args[3])
                    return 'home_home', True

                def confirm(*args):
                    events.append('confirm')
                    self.assertIn('P77', args[2])
                    return QMessageBox.Yes

                with patch.object(QInputDialog, 'getItem', side_effect=select), patch.object(
                        QMessageBox, 'question', side_effect=confirm), patch.object(
                        self.window.experiment_process, 'start') as start:
                    self.window.run_experiment_button.click()

                command = start.call_args.args[1][1]
                if hand == 'Right':
                    self.assertEqual(events, ['Select participant', 'Select start state', 'confirm'])
                    self.assertIn('experiment_backend.right_hand', command)
                    self.assertIn('--task full --start home_home --participant P77', command)
                else:
                    self.assertEqual(events, ['Select participant', 'confirm'])
                    self.assertIn('experiment_backend.left_hand_direct', command)
                    self.assertIn('--protocol single --reference-participant P77', command)
                self.assertNotIn('--speed-scale', command)
                self.window.update_experiment_controls()
        self.assertIn('Scalar participant target: NOT DEFINED', self.window.log_output.toPlainText())

    def test_missing_reference_blocks_ready_and_launch(self):
        self.select_participant_variant()
        (self.root / 'P77/polishing/reference/tension_summary.json').unlink()
        self.window.update_experiment_controls()
        self.assertFalse(self.window.run_experiment_button.isEnabled())
        self.assertNotEqual(self.window.experiment_status_label.text(), 'READY')
        with patch.object(QMessageBox, 'critical') as error, patch.object(self.window.experiment_process, 'start') as start:
            self.window.run_selected_experiment()
        start.assert_not_called()
        self.assertIn('participant', self.window.log_output.toPlainText().lower())

    def test_removed_file_during_selection_gives_error_without_launch(self):
        self.select_participant_variant()
        def select(*args):
            (self.root / 'P77/polishing/reference/tension_summary.json').unlink()
            return 'P77', True
        with patch.object(QInputDialog, 'getItem', side_effect=select), patch.object(
                QMessageBox, 'critical') as error, patch.object(self.window.experiment_process, 'start') as start:
            self.window.run_experiment_button.click()
        error.assert_called_once()
        start.assert_not_called()

    def test_analysis_selector_identifies_participant_variant(self):
        meta = {'trial_kind': 'full_direct_protocol', 'direct_protocol': 'single',
                'experiment_variant': 'participant-selected', 'reference_participant': 'P77'}
        label = self.window._analysis_trial_label(Path('/tmp/trial_test'), meta)
        self.assertIn('Full Direct - Participant', label)
        self.assertIn('P77', label)
        meta.pop('experiment_variant')
        self.assertIn('Full Direct - One Rep', self.window._analysis_trial_label(Path('/tmp/trial_test'), meta))

    def test_existing_dataset_resolver_and_scan_accept_external_dataset_root(self):
        gesture = self.root / 'P77/compound'
        self.assertEqual(replay_ui.resolve_participant_directory(gesture / 'reference', self.root), self.root / 'P77')
        self.assertTrue(replay_ui.scan_gesture_dataset(gesture)['ik_ready'])

    def test_replay_box_updates_with_gesture_and_missing_summary(self):
        with patch.object(replay_ui, 'QSoundEffect', None):
            panel = replay_ui.HumanReferencePanel(lambda: 'Real Robot(s)')
        self.addCleanup(panel.deleteLater)
        panel.health_timer.stop()
        panel.dataset_root = self.root
        panel.participant.setText('P77')
        panel.gesture.setCurrentText('Compound')
        panel.reload_selected_dataset()
        self.assertFalse(panel.initial_force_toggle.isChecked())
        self.assertTrue(panel.initial_force_label.isHidden())
        self.assertEqual(panel.initial_force_toggle.text(), '▸ INITIAL FORCE REFERENCE')
        panel.initial_force_toggle.click()
        self.assertFalse(panel.initial_force_label.isHidden())
        self.assertEqual(panel.initial_force_toggle.text(), '▾ INITIAL FORCE REFERENCE')
        self.assertNotIn('Saved initial window', panel.initial_force_label.text())
        self.assertNotIn('INITIAL FORCE REFERENCE', panel.initial_force_label.text())
        self.assertIn('Fx:', panel.initial_force_label.text())
        self.assertIn('1.96', panel.initial_force_label.text())
        self.assertIn('270', panel.initial_force_label.text())
        self.assertIn('Scalar target: Not defined', panel.initial_force_label.text())
        summary_path = self.root / 'P77/compound/reference/tension_summary.json'
        summary = json.loads(summary_path.read_text())
        summary.update(scalar_tension_valid=True, force_axis='Fy_N', sign=-1, mean_N=6.72)
        summary_path.write_text(json.dumps(summary))
        panel.reload_selected_dataset()
        self.assertIn('Scalar target: 6.72 N (mean_N)', panel.initial_force_label.text())
        (self.root / 'P77/polishing/reference/tension_summary.json').unlink()
        panel.initial_force_toggle.click()
        panel.gesture.setCurrentText('Polishing')
        self.assertFalse(panel.initial_force_toggle.isChecked())
        self.assertTrue(panel.initial_force_label.isHidden())
        panel.initial_force_toggle.click()
        self.assertEqual(panel.initial_force_label.text(), 'Initial force reference: unavailable')
        self.assertFalse(panel.initial_force_label.isHidden())
        panel.participant.setText('MissingParticipant')
        panel.participant.editingFinished.emit()
        self.assertFalse(panel.initial_force_toggle.isChecked())
        self.assertTrue(panel.initial_force_label.isHidden())
        panel.initial_force_toggle.click()
        self.assertEqual(panel.initial_force_label.text(), 'Initial force reference: unavailable')
        panel.initial_force_toggle.click()
        self.assertTrue(panel.initial_force_label.isHidden())


if __name__ == '__main__':
    unittest.main()
