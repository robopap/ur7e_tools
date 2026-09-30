"""Exercise real Qt wiring without connecting ROS or launching robot commands."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import unittest
from unittest.mock import MagicMock, patch

from PySide6.QtCore import QProcess, QTimer
from PySide6.QtWidgets import QApplication, QMessageBox, QInputDialog
from ur7e_tools import workcell_ui as ui


class ExperimentPlayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        # Keep actual widgets, process signals, and handlers. Only replace ROS
        # and background monitoring; no workcell launch or hardware commands.
        with patch.object(ui.rclpy, 'ok', return_value=True), patch.object(
                ui, 'WrenchListenerNode'), patch.object(ui.threading.Thread, 'start'), patch.object(
                ui.WorkcellUI, 'apply_startup_preflight_report'):
            self.window = ui.WorkcellUI()
        self.addCleanup(self.window.deleteLater)
        for timer in self.window.findChildren(QTimer):
            timer.stop()
        self.window.experiment_task_combo.setCurrentText('Full')
        self.window.experiment_hand_combo.setCurrentText('Left')
        self.window.status_label.setProperty('systemState', 'RUNNING')
        self.window.robot_ready = {'robot1': True, 'robot2': True}
        helpers = MagicMock()
        helpers.discover_reference_participants.return_value = ['P01']
        helpers.participant_reference_log_lines.return_value = ['Participant: P01']
        self.window.participant_reference_helpers = lambda: helpers
        self.window.participant_reference_config = lambda participant, hand: {
            'reference_participant': participant,
        }
        self.window.update_experiment_controls()

    def test_readiness_recovery_refreshes_play_and_status(self):
        self.window.robot_ready['robot1'] = False
        self.window.update_robot_ready_summary()
        self.assertFalse(self.window.run_experiment_button.isEnabled())
        self.assertNotEqual(self.window.experiment_status_label.text(), 'READY')
        self.window.robot_ready['robot1'] = True
        self.window.update_robot_ready_summary()
        self.assertTrue(self.window.run_experiment_button.isEnabled())
        self.assertEqual(self.window.experiment_status_label.text(), 'READY')

    def test_process_transitions_refresh_play(self):
        for name in ['home_process', 'gripper_process', 'analysis_process', 'experiment_process']:
            with self.subTest(process=name):
                process = getattr(self.window, name)
                # A harmless child exercises actual Starting/Running/NotRunning signals.
                process.start('/bin/sleep', ['0.03'])
                try:
                    self.assertFalse(self.window.run_experiment_button.isEnabled())
                    self.assertNotEqual(self.window.experiment_status_label.text(), 'READY')
                finally:
                    process.waitForFinished(2000)
                self.assertTrue(self.window.run_experiment_button.isEnabled())

    def test_click_signal_confirms_and_builds_exact_left_command_without_ui_speed(self):
        with patch.object(QInputDialog, 'getItem', return_value=('P01', True)), patch.object(
                QMessageBox, 'question', return_value=QMessageBox.Yes) as confirm, patch.object(
                self.window.experiment_process, 'start') as start:
            self.window.run_experiment_button.click()
        confirm.assert_called_once()
        expected = ('source /opt/ros/humble/setup.bash'
                    ' && source /home/caor/ros2_ws/install/setup.bash'
                    ' && cd /home/caor/phd_polishing_experiments'
                    ' && exec setsid /usr/bin/python3 -u -m experiment_backend.left_hand_direct'
                    ' --mode real --protocol single --reference-participant P01')
        start.assert_called_once_with('/bin/bash', ['-lc', expected])
        log = self.window.log_output.toPlainText()
        for message in ['[PLAY] Experiment Play clicked', '[PLAY] Runtime gates passed',
                        '[PLAY] Selection:', '[PLAY] Direct launch:', '[PLAY] Launching:', expected]:
            self.assertIn(message, log)

    def test_right_direct_launch_and_confirmation_are_preserved(self):
        self.window.experiment_hand_combo.setCurrentText('Right')
        with patch.object(QInputDialog, 'getItem', side_effect=[('P01', True), ('home_home', True)]), patch.object(
                QMessageBox, 'question', return_value=QMessageBox.Yes) as confirm, patch.object(
                self.window.experiment_process, 'start') as start:
            self.window.run_experiment_button.click()
        confirm.assert_called_once()
        command = start.call_args.args[1][1]
        self.assertIn('/usr/bin/python3 -u -m experiment_backend.right_hand --mode real', command)
        self.assertIn('--task full --start home_home --participant P01', command)
        self.assertNotIn('--speed-scale', command)
        self.assertNotIn('--protocol', command)

    def test_each_runtime_gate_logs_actual_reason_without_launch(self):
        cases = [('system_state', 'STOPPED'), ('setup', 'Single UR3')]
        cases += [(name, QProcess.Starting) for name in [
            'experiment_process', 'analysis_process', 'home_process', 'gripper_process']]
        for name, value in cases:
            with self.subTest(gate=name):
                if name == 'system_state':
                    target, attr = self.window, 'system_state'
                elif name == 'setup':
                    target, attr = self.window.setup_combo, 'currentText'
                else:
                    target, attr = getattr(self.window, name), 'state'
                with patch.object(target, attr, return_value=value), patch.object(
                        QMessageBox, 'question') as confirm, patch.object(self.window.experiment_process, 'start') as start:
                    self.window.run_selected_experiment()
                    confirm.assert_not_called()
                    start.assert_not_called()
                    self.assertIn(f'[PLAY BLOCKED] {name}=', self.window.log_output.toPlainText())

    def test_readiness_loss_during_confirmation_blocks_launch(self):
        def answer(*args):
            self.window.robot_ready['robot2'] = False
            return QMessageBox.Yes
        with patch.object(QInputDialog, 'getItem', return_value=('P01', True)), patch.object(
                QMessageBox, 'question', side_effect=answer), patch.object(
                self.window.experiment_process, 'start') as start:
            self.window.run_experiment_button.click()
        start.assert_not_called()
        self.assertIn('[PLAY BLOCKED] robot_ready=', self.window.log_output.toPlainText())

    def test_real_gripper_gate_and_declined_confirmation(self):
        with patch.object(self.window, '_dual_gripper_type', return_value='robotiq'):
            self.window.update_experiment_controls()
            self.assertFalse(self.window.run_experiment_button.isEnabled())
            self.assertNotEqual(self.window.experiment_status_label.text(), 'READY')
        self.window.update_experiment_controls()
        with patch.object(QInputDialog, 'getItem', return_value=('P01', True)), patch.object(
                QMessageBox, 'question', return_value=QMessageBox.No), patch.object(
                self.window.experiment_process, 'start') as start:
            self.window.run_experiment_button.click()
        start.assert_not_called()
        self.assertIn('[PLAY BLOCKED] real confirmation declined', self.window.log_output.toPlainText())

    def test_failed_qprocess_start_logs_error_string_and_restores_controls(self):
        self.window.active_experiment = self.window._selected_experiment()
        self.window.experiment_process.start('/no/such/experiment-test-command', [])
        self.window.experiment_process.waitForStarted(1000)
        log = self.window.log_output.toPlainText()
        self.assertIn('[PLAY ERROR]', log)
        self.assertIn('FailedToStart', log)
        self.assertIn(self.window.experiment_process.errorString(), log)
        self.assertIsNone(self.window.active_experiment)
        self.assertTrue(self.window.run_experiment_button.isEnabled())


if __name__ == '__main__':
    unittest.main()
