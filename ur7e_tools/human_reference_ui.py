#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import shlex
import struct
import sys
import wave
from pathlib import Path

from PySide6.QtCore import QProcess, QTimer, Qt, QUrl
from PySide6.QtGui import QColor, QPainter, QPixmap
from PySide6.QtWidgets import (
    QFileDialog,
    QAbstractItemView,
    QComboBox,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMessageBox,
    QPushButton,
    QStyle,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ur7e_tools.participant_reference_helpers import reference_helpers

try:
    from PySide6.QtMultimedia import QSoundEffect
except Exception:
    QSoundEffect = None


GESTURES = {"compound", "polishing"}
REQUIRED_REP_FILES = ("metadata.json", "nansense_raw.csv")


def _write_tone(
    path: Path,
    frequency: float,
    duration_s: float,
    volume: float = 0.35,
    rate: int = 44100,
):
    path.parent.mkdir(parents=True, exist_ok=True)
    n = int(rate * duration_s)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        frames = bytearray()
        for i in range(n):
            env = min(1.0, i / (0.01 * rate + 1), (n - i) / (0.02 * rate + 1))
            v = int(
                32767
                * volume
                * env
                * math.sin(2 * math.pi * frequency * i / rate)
            )
            frames += struct.pack("<h", v)
        w.writeframes(frames)


def ensure_cue_assets(asset_dir: Path):
    tin = asset_dir / "countdown_tin.wav"
    go = asset_dir / "go_tone.wav"
    done = asset_dir / "done_tone.wav"
    if not tin.exists():
        _write_tone(tin, 880, 0.12)
    if not go.exists():
        _write_tone(go, 440, 0.35)
    if not done.exists():
        _write_tone(done, 660, 0.22)
    return tin, go, done


def resolve_gesture_directory(selected_path: Path, dataset_root: Path):
    """Resolve a selected gesture/rep/reference folder to its gesture folder."""
    selected = Path(selected_path).expanduser()
    root = Path(dataset_root).expanduser()

    if selected.is_file():
        selected = selected.parent

    try:
        selected_resolved = selected.resolve()
        root_resolved = root.resolve()
    except OSError:
        return None

    # Accept the gesture folder itself or any descendant such as rep_03/reference.
    candidate = selected_resolved
    while True:
        try:
            candidate.relative_to(root_resolved)
        except ValueError:
            break

        if candidate.name.lower() in GESTURES:
            return candidate
        if candidate == root_resolved:
            break
        candidate = candidate.parent

    # Also accept a participant folder when it contains exactly one gesture.
    try:
        selected_resolved.relative_to(root_resolved)
    except ValueError:
        return None

    gesture_dirs = [
        selected_resolved / name
        for name in sorted(GESTURES)
        if (selected_resolved / name).is_dir()
    ]
    if len(gesture_dirs) == 1:
        return gesture_dirs[0]

    return None


def resolve_participant_directory(selected_path: Path, dataset_root: Path):
    """Reuse the shared participant/descendant dataset resolver."""
    try:
        helpers = reference_helpers()
        return helpers.resolve_participant_directory(selected_path, dataset_root)
    except (OSError, ImportError):
        return None


def participant_id_from_directory(participant_dir: Path):
    """Recover one participant ID from repetition metadata, else folder name."""
    participant_dir = Path(participant_dir).expanduser()
    ids = []
    for gesture in sorted(GESTURES):
        for rep in (1,):
            metadata = _read_rep_metadata(participant_dir / gesture / f"rep_{rep:02d}")
            participant = str(metadata.get("participant", "")).strip()
            if participant:
                ids.append(participant)

    unique = sorted(set(ids))
    if len(unique) == 1:
        return unique[0]
    return participant_dir.name


def _read_rep_metadata(rep_dir: Path):
    path = rep_dir / "metadata.json"
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return {}
    return value if isinstance(value, dict) else {}


def scan_gesture_dataset(gesture_dir: Path):
    """Return persistent Human Reference state derived only from disk."""
    gesture_dir = Path(gesture_dir).expanduser()
    folder_participant = gesture_dir.parent.name
    folder_gesture = gesture_dir.name.lower()

    occupied_reps = []
    valid_reps = []
    blocked_incomplete_reps = []
    metadata_participants = []
    metadata_gestures = []
    replay_dirs = []

    for rep in (1,):
        rep_dir = gesture_dir / f"rep_{rep:02d}"
        if not rep_dir.exists():
            continue

        occupied_reps.append(rep)
        metadata = _read_rep_metadata(rep_dir)
        participant = str(metadata.get("participant", "")).strip()
        gesture = str(metadata.get("gesture", "")).strip().lower()
        if participant:
            metadata_participants.append(participant)
        if gesture in GESTURES:
            metadata_gestures.append(gesture)

        valid = rep_dir.is_dir() and all(
            (rep_dir / filename).is_file() for filename in REQUIRED_REP_FILES
        )
        if valid:
            valid_reps.append(rep)
        else:
            blocked_incomplete_reps.append(rep)

        if (rep_dir / "replay.html").is_file():
            replay_dirs.append(rep_dir)

    participant = folder_participant
    unique_participants = sorted(set(metadata_participants))
    if len(unique_participants) == 1:
        participant = unique_participants[0]

    gesture = folder_gesture
    unique_gestures = sorted(set(metadata_gestures))
    if len(unique_gestures) == 1:
        gesture = unique_gestures[0]

    recordings_complete = valid_reps == [1]

    if recordings_complete:
        current_rep = 1
    elif blocked_incomplete_reps:
        # Preserve occupied/incomplete data: point at it and block START TRIAL
        # instead of silently skipping to a later repetition.
        current_rep = blocked_incomplete_reps[0]
    else:
        current_rep = 1

    reference_dir = gesture_dir / "reference"
    reference_ready = (
        reference_dir / "human_motion_reference_bimanual.csv"
    ).is_file()
    try:
        filename = reference_helpers().MOTION_REFERENCE_FILENAME
        ik_ready = (reference_dir / filename).is_file()
    except (OSError, ImportError):
        ik_ready = False

    return {
        "gesture_dir": gesture_dir,
        "participant": participant,
        "gesture": gesture,
        "occupied_reps": occupied_reps,
        "valid_reps": valid_reps,
        "blocked_incomplete_reps": blocked_incomplete_reps,
        "recordings_complete": recordings_complete,
        "current_rep": current_rep,
        "reference_ready": reference_ready,
        "ik_ready": ik_ready,
        "replay_dir": replay_dirs[-1] if replay_dirs else None,
    }


class HumanReferencePanel(QFrame):
    def __init__(
        self,
        mode_provider,
        nansense_ready_provider=None,
        nansense_alignment_ready_provider=None,
        nansense_alignment_provider=None,
        robot2_ft_ready_provider=None,
        parent=None,
        project_root=None,
    ):
        super().__init__(parent)
        self.mode_provider = mode_provider
        self.nansense_ready_provider = nansense_ready_provider
        self.nansense_alignment_ready_provider = nansense_alignment_ready_provider
        self.nansense_alignment_provider = nansense_alignment_provider
        self.robot2_ft_ready_provider = robot2_ft_ready_provider
        self.project_root = Path(
            project_root or "~/phd_polishing_experiments"
        ).expanduser()
        self.dataset_root = self.project_root / "results" / "human_reference"

        self.current_rep = 1
        self.record_process = None
        self.build_process = None
        self.train_process = None
        self.sim_process = None
        self.last_trial_dir = None
        self._trial_active = False
        self._build_active = False
        self._train_active = False
        self._sim_active = False
        self._train_log_tail = []
        self._sim_log_tail = []
        self._recordings_complete = False
        self._reference_ready = False
        self._ik_ready = False
        self._blocked_incomplete_reps = []
        self._loaded_gesture_dir = None

        self.setObjectName("humanReferencePanel")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(7, 6, 7, 6)
        lay.setSpacing(5)

        row = QHBoxLayout()
        title = QLabel("Human Reference")
        title.setStyleSheet("font-weight: 600;")
        row.addWidget(title)
        row.addStretch(1)
        self.status = QLabel("WAIT NANSENSE")
        row.addWidget(self.status)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        row2.setContentsMargins(0, 0, 0, 0)
        row2.setSpacing(6)

        # Left: compact participant icon + flexible participant field.
        participant_icon = QLabel()
        participant_icon.setFixedSize(20, 20)

        participant_pixmap = QPixmap(18, 18)
        participant_pixmap.fill(Qt.transparent)
        painter = QPainter(participant_pixmap)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#dfe3e7"))
        painter.drawEllipse(6, 1, 6, 6)
        painter.drawEllipse(3, 8, 12, 9)
        painter.end()

        participant_icon.setPixmap(participant_pixmap)
        participant_icon.setAlignment(Qt.AlignCenter)
        participant_icon.setToolTip("Participant / user")
        row2.addWidget(participant_icon)

        self.participant = QLineEdit("P01")
        self.participant.setMinimumWidth(60)
        row2.addWidget(self.participant, 1)

        # The only visible flexible gap in this row.
        row2.addStretch(1)

        # One compact right-side block: Gesture + folder + dataset path.
        right_block = QHBoxLayout()
        right_block.setContentsMargins(0, 0, 0, 0)
        right_block.setSpacing(6)

        right_block.addWidget(QLabel("Gesture:"))
        self.gesture = QComboBox()
        self.gesture.addItems(["Compound", "Polishing"])
        self.gesture.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToContents
        )
        right_block.addWidget(self.gesture)

        self.dataset_btn = QPushButton()
        self.dataset_btn.setFixedWidth(34)
        self.dataset_btn.setIcon(
            self.style().standardIcon(QStyle.SP_DirOpenIcon)
        )
        self.dataset_btn.setToolTip(
            "Open an existing participant/gesture Human Reference dataset."
        )
        right_block.addWidget(self.dataset_btn)

        self.dataset_path_label = QLabel("")
        self.dataset_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.dataset_path_label.setToolTip(
            "Current Human Reference dataset folder."
        )
        right_block.addWidget(self.dataset_path_label)

        row2.addLayout(right_block)

        lay.addLayout(row2)

        row3 = QHBoxLayout()
        self.rep_label = QLabel("1 RECORDING")
        row3.addWidget(self.rep_label)
        self.zero_world_x = QCheckBox("ZERO WORLD X (Robot1 Z)")
        self.zero_world_x.setToolTip(
            "Build-time option only: set RightHand World X displacement to zero. "
            "Raw NANSENSE, synchronized data, and LeftHand motion remain unchanged."
        )
        row3.addWidget(self.zero_world_x)
        row3.addStretch(1)
        self.start_btn = QPushButton("START TRIAL")
        self.start_btn.setEnabled(False)
        self.build_btn = QPushButton("BUILD REFERENCE")
        self.build_btn.setEnabled(False)

        self.train_btn = QPushButton("TRAIN MODEL")
        self.train_btn.setEnabled(False)
        self.train_btn.setToolTip(
            "Train a 100 ms fixed-beta/fixed-phi SARIMAX wrench predictor "
            "from exactly three selected participants."
        )

        row3.addWidget(self.start_btn)
        row3.addWidget(self.build_btn)
        row3.addWidget(self.train_btn)
        lay.addLayout(row3)

        row4 = QHBoxLayout()
        self.replay_btn = QPushButton("OPEN REPLAY")
        self.replay_btn.setEnabled(False)
        self.sim_btn = QPushButton("RUN SIMULATION")
        self.sim_btn.setEnabled(False)
        row4.addWidget(self.replay_btn)
        row4.addWidget(self.sim_btn)
        lay.addLayout(row4)

        self.initial_force_toggle = QToolButton()
        self.initial_force_toggle.setText("▸ INITIAL FORCE REFERENCE")
        self.initial_force_toggle.setCheckable(True)
        self.initial_force_toggle.setAutoRaise(True)
        self.initial_force_toggle.toggled.connect(self._toggle_initial_force_summary)
        lay.addWidget(self.initial_force_toggle)

        self.initial_force_label = QLabel("Initial force reference: unavailable")
        self.initial_force_label.setWordWrap(True)
        self.initial_force_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.initial_force_label.setFrameStyle(QFrame.StyledPanel)
        self.initial_force_label.hide()
        lay.addWidget(self.initial_force_label)

        self.start_btn.clicked.connect(self.start_trial)
        self.build_btn.clicked.connect(self.build_reference)
        self.train_btn.clicked.connect(self.train_forecast_model)
        self.replay_btn.clicked.connect(self.open_replay)
        self.sim_btn.clicked.connect(self.run_selected_gesture)
        self.dataset_btn.clicked.connect(self.browse_dataset)
        self.participant.editingFinished.connect(self.reload_selected_dataset)
        self.gesture.currentIndexChanged.connect(self.reload_selected_dataset)

        asset_dir = Path(__file__).resolve().parent / "assets"
        self.tin_path, self.go_path, self.done_path = ensure_cue_assets(asset_dir)
        self._sounds = []
        if QSoundEffect:
            for p in (self.tin_path, self.go_path, self.done_path):
                s = QSoundEffect(self)
                s.setSource(QUrl.fromLocalFile(str(p)))
                s.setVolume(0.8)
                self._sounds.append(s)

        self.reload_selected_dataset()

        self.health_timer = QTimer(self)
        self.health_timer.setInterval(100)
        self.health_timer.timeout.connect(self.refresh_input_health)
        self.health_timer.start()
        self.refresh_input_health()

    def _provider_ready(self, provider):
        if provider is None:
            return False
        try:
            return bool(provider())
        except Exception:
            return False

    def input_health(self):
        nansense_ready = self._provider_ready(self.nansense_ready_provider)
        alignment_ready = self._provider_ready(
            self.nansense_alignment_ready_provider
        )
        real_mode = self._mode() == "real"
        robot2_ft_ready = (
            self._provider_ready(self.robot2_ft_ready_provider)
            if real_mode
            else True
        )
        return {
            "nansense_ready": nansense_ready,
            "nansense_alignment_ready": alignment_ready,
            "robot2_ft_ready": robot2_ft_ready,
            "ready": nansense_ready and alignment_ready and robot2_ft_ready,
        }

    def _selected_gesture_dir(self):
        participant = self.participant.text().strip()
        gesture = self.gesture.currentText().strip().lower()
        return self.dataset_root / participant / gesture

    def reload_selected_dataset(self, *_args):
        if self._trial_active or self._build_active:
            return

        gesture_dir = self._selected_gesture_dir()
        state = scan_gesture_dataset(gesture_dir)
        self._apply_disk_state(state)
        self.refresh_input_health()

    def _apply_disk_state(self, state):
        self._loaded_gesture_dir = state["gesture_dir"]
        self.current_rep = int(state["current_rep"])
        self._recordings_complete = bool(state["recordings_complete"])
        self._reference_ready = bool(state["reference_ready"])
        self._ik_ready = bool(state["ik_ready"])
        self._blocked_incomplete_reps = list(
            state["blocked_incomplete_reps"]
        )
        self.last_trial_dir = state["replay_dir"]
        self.refresh_initial_force_summary()

        if self._recordings_complete:
            self.rep_label.setText("RECORDING COMPLETE")
        else:
            self.rep_label.setText("1 RECORDING")

        self.replay_btn.setEnabled(bool(self.last_trial_dir))
        self.build_btn.setEnabled(
            self._recordings_complete and not self._build_active
        )
        self.train_btn.setEnabled(
            self._forecast_training_ready() and not self._train_active
        )
        self.sim_btn.setEnabled(not self._sim_active)

        if self._loaded_gesture_dir is not None:
            try:
                display_path = self._loaded_gesture_dir.relative_to(
                    self.project_root
                ).as_posix()
            except ValueError:
                display_path = str(self._loaded_gesture_dir)
            self.dataset_path_label.setText(display_path)
            self.dataset_path_label.setToolTip(
                str(self._loaded_gesture_dir)
            )
            self.dataset_btn.setToolTip(
                "Choose participant dataset folder.\n"
                f"Current: {self._loaded_gesture_dir}"
            )

    def refresh_input_health(self):
        if (
            self._trial_active
            or self._build_active
            or self._train_active
            or self._sim_active
        ):
            return

        self.sim_btn.setText(
            "RUN REAL" if self._mode() == "real" else "RUN SIMULATION"
        )
        self.sim_btn.setEnabled(not self._sim_active)
        self.replay_btn.setEnabled(bool(self.last_trial_dir))

        if self._blocked_incomplete_reps:
            rep = self._blocked_incomplete_reps[0]
            self.start_btn.setEnabled(False)
            self.build_btn.setEnabled(False)
            self.status.setText(f"INCOMPLETE REP {rep}")
            return

        if self._reference_ready:
            self.start_btn.setEnabled(False)
            self.build_btn.setEnabled(self._recordings_complete)
            self.status.setText("REFERENCE READY")
            return

        if self._recordings_complete:
            self.start_btn.setEnabled(False)
            self.build_btn.setEnabled(True)
            self.status.setText("RECORDING READY")
            return

        self.build_btn.setEnabled(False)
        health = self.input_health()
        self.start_btn.setEnabled(bool(health["ready"]))

        if not health["nansense_ready"]:
            self.status.setText("WAIT NANSENSE")
        elif not health["nansense_alignment_ready"]:
            self.status.setText("LOCK NANSENSE")
        elif not health["robot2_ft_ready"]:
            self.status.setText("WAIT ROBOT2 F/T")
        else:
            self.status.setText("READY")

    def browse_dataset(self):
        self.dataset_root.mkdir(parents=True, exist_ok=True)

        selected = QFileDialog.getExistingDirectory(
            self,
            "Select participant folder (for example P01)",
            str(self.dataset_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if not selected:
            return

        participant_dir = resolve_participant_directory(
            selected, self.dataset_root
        )
        if participant_dir is None:
            QMessageBox.information(
                self,
                "Human Reference dataset",
                "Select a participant folder such as P01.\n\n"
                "You may also select Compound/Polishing, rep_XX, or reference "
                "inside that participant; the UI will recover the participant "
                "automatically.",
            )
            return

        participant = participant_id_from_directory(participant_dir)
        self.participant.blockSignals(True)
        self.participant.setText(participant)
        self.participant.blockSignals(False)

        # Standard datasets are stored under the participant ID. If old data
        # use a different folder name, keep the folder name because it is the
        # actual on-disk location and show the recovered ID in the field only
        # when both agree.
        if participant_dir.name != participant:
            self.participant.setText(participant_dir.name)
            QMessageBox.information(
                self,
                "Participant metadata",
                f"Metadata reports participant '{participant}', but the dataset "
                f"folder is '{participant_dir.name}'. The folder name is being "
                "used as the storage ID to avoid selecting the wrong path.",
            )

        self.reload_selected_dataset()

    def _play(self, index):
        if self._sounds:
            self._sounds[index].play()
        else:
            from PySide6.QtWidgets import QApplication

            QApplication.beep()

    def _mode(self):
        return "real" if "Real" in str(self.mode_provider()) else "simulation"

    def _cmd(self, module, args, python_executable=None):
        python_executable = Path(
            python_executable or sys.executable
        ).expanduser()

        q = [
            "cd",
            shlex.quote(str(self.project_root)),
            "&&",
            shlex.quote(str(python_executable)),
            "-m",
            module,
            *[shlex.quote(str(x)) for x in args],
        ]
        return " ".join(q)

    def start_trial(self):
        if (
            self.record_process
            and self.record_process.state() != QProcess.NotRunning
        ):
            return

        if not self.input_health()["ready"]:
            self.refresh_input_health()
            QMessageBox.warning(
                self,
                "Human Reference inputs",
                "NANSENSE must be streaming and ALIGNMENT LOCKED. "
                "In Real Robot(s) mode, Robot2 internal F/T must also be LIVE.",
            )
            return

        try:
            alignment = (
                self.nansense_alignment_provider()
                if self.nansense_alignment_provider is not None
                else None
            )
        except Exception:
            alignment = None
        if not isinstance(alignment, dict) or alignment.get("locked") is not True:
            self.refresh_input_health()
            QMessageBox.warning(
                self,
                "NANSENSE alignment",
                "The scientific NANSENSE alignment is not locked. "
                "Set Yaw/Mirror in the NANSENSE widget and press LOCK ALIGNMENT.",
            )
            return

        participant = self.participant.text().strip()
        if not participant:
            QMessageBox.warning(self, "Participant", "Enter participant ID")
            return

        gesture = self.gesture.currentText().strip().lower()
        target_dir = (
            self.dataset_root
            / participant
            / gesture
            / f"rep_{self.current_rep:02d}"
        )

        # Fail closed: never ask the recorder to reuse an occupied repetition.
        if target_dir.exists():
            QMessageBox.warning(
                self,
                "Existing trial protected",
                "The target repetition folder already exists:\n\n"
                f"{target_dir}\n\n"
                "START TRIAL was blocked. Nothing was deleted or overwritten.",
            )
            self.reload_selected_dataset()
            return

        answer = QMessageBox.question(
            self,
            "Start Human Reference Trial",
            f"Participant: {participant}\n"
            f"Gesture: {self.gesture.currentText()}\n"
            "Recording: single 20 s reference\n"
            f"NANSENSE alignment: Yaw {float(alignment['yaw_deg']):.1f}° | "
            f"Mirror {'ON' if alignment['mirror_lateral'] else 'OFF'}\n\n"
            f"Target folder:\n{target_dir}\n\n"
            "This will create a new repetition folder. Existing repetition "
            "folders are protected and will never be overwritten or deleted.\n\n"
            "Are you sure you want to start this trial?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        # Re-read immediately before launching so a modified Yaw/Mirror cannot
        # slip through after the confirmation dialog.
        try:
            alignment = (
                self.nansense_alignment_provider()
                if self.nansense_alignment_provider is not None
                else None
            )
        except Exception:
            alignment = None
        if not isinstance(alignment, dict) or alignment.get("locked") is not True:
            self.refresh_input_health()
            QMessageBox.warning(
                self,
                "NANSENSE alignment",
                "Alignment changed or was unlocked before recording. LOCK ALIGNMENT again.",
            )
            return

        self._trial_active = True
        self.status.setText("PRE-TENSION 8 s")
        self.start_btn.setEnabled(False)
        self.build_btn.setEnabled(False)
        self.record_process = QProcess(self)
        self.record_process.setProcessChannelMode(QProcess.MergedChannels)
        self.record_process.readyReadStandardOutput.connect(self._record_output)
        self.record_process.finished.connect(self._record_finished)
        args = [
            "--root",
            self.dataset_root,
            "--participant",
            participant,
            "--gesture",
            gesture,
            "--rep",
            self.current_rep,
            "--mode",
            self._mode(),
            "--pre",
            8,
            "--motion",
            20,
            "--nansense-alignment-json",
            json.dumps(alignment, separators=(",", ":")),
        ]
        self.record_process.start(
            "/bin/bash",
            [
                "-lc",
                self._cmd(
                    "experiment_backend.human_reference_recorder",
                    args,
                ),
            ],
        )

    def _record_output(self):
        text = bytes(
            self.record_process.readAllStandardOutput()
        ).decode(errors="replace")
        for line in text.splitlines():
            if "EVENT T_MINUS_3" in line:
                self.status.setText("3")
                self._play(0)
            elif "EVENT T_MINUS_2" in line:
                self.status.setText("2")
                self._play(0)
            elif "EVENT T_MINUS_1" in line:
                self.status.setText("1")
                self._play(0)
            elif "EVENT GO" in line:
                self.status.setText("RECORDING")
                self._play(1)
            elif line.startswith("TRIAL_SAVED "):
                self.last_trial_dir = Path(line.split(" ", 1)[1].strip())

    def _record_finished(self, code, status):
        self._trial_active = False
        if code != 0:
            self.status.setText("RECORD ERROR")
            return

        self._play(2)
        self.reload_selected_dataset()

    def _eligible_forecast_training_participants(self):
        """Participants with one complete raw recording for the selected gesture."""
        gesture = self.gesture.currentText().strip().lower()
        required = (
            "metadata.json",
            "nansense_raw.csv",
            "robot2_ft_raw.csv",
            "synchronized_90hz.csv",
        )

        eligible = []
        if not self.dataset_root.is_dir():
            return eligible

        for participant_dir in sorted(self.dataset_root.iterdir()):
            if not participant_dir.is_dir():
                continue

            rep_dir = participant_dir / gesture / "rep_01"
            if (
                rep_dir.is_dir()
                and all((rep_dir / name).is_file() for name in required)
            ):
                eligible.append(participant_dir.name)

        return eligible

    def _forecast_training_ready(self):
        return len(self._eligible_forecast_training_participants()) >= 3

    def _select_three_training_participants(self):
        participants = self._eligible_forecast_training_participants()

        if len(participants) < 3:
            QMessageBox.warning(
                self,
                "TRAIN MODEL",
                "At least three complete participant recordings are required "
                "for the selected gesture.\n\n"
                "Each participant must contain rep_01 with:\n"
                "metadata.json\n"
                "nansense_raw.csv\n"
                "robot2_ft_raw.csv\n"
                "synchronized_90hz.csv",
            )
            return None

        dialog = QDialog(self)
        dialog.setWindowTitle(
            f"Select 3 training participants — {self.gesture.currentText()}"
        )
        layout = QVBoxLayout(dialog)

        label = QLabel(
            "Select exactly three participants to train the proactive predictor."
        )
        label.setWordWrap(True)
        layout.addWidget(label)

        participant_list = QListWidget(dialog)
        participant_list.setSelectionMode(
            QAbstractItemView.SelectionMode.MultiSelection
        )
        for participant in participants:
            participant_list.addItem(participant)
        layout.addWidget(participant_list)

        selected_label = QLabel("Selected: 0 / 3")
        layout.addWidget(selected_label)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel,
            parent=dialog,
        )
        ok_button = buttons.button(QDialogButtonBox.StandardButton.Ok)
        ok_button.setEnabled(False)
        layout.addWidget(buttons)

        def refresh_selection():
            count = len(participant_list.selectedItems())
            selected_label.setText(f"Selected: {count} / 3")
            ok_button.setEnabled(count == 3)

        participant_list.itemSelectionChanged.connect(refresh_selection)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)

        if dialog.exec() != QDialog.DialogCode.Accepted:
            return None

        selected = sorted(
            item.text().strip()
            for item in participant_list.selectedItems()
            if item.text().strip()
        )
        return selected if len(selected) == 3 else None

    def _forecast_training_id(self, participants):
        participants = sorted(str(p).strip() for p in participants)
        return (
            "_".join(participants)
            + "_60hz_100ms_1hz"
        )

    def _forecast_model_dir(self, gesture, participants):
        return (
            self.project_root
            / "results"
            / "wrench_models"
            / gesture
            / self._forecast_training_id(participants)
        )

    def _forecast_trainer_script(self):
        candidates = (
            self.project_root
            / "experiment_backend"
            / "train_live_wrench_predictor.py",
            self.project_root
            / "tools"
            / "train_live_wrench_predictor.py",
        )
        for path in candidates:
            if path.is_file():
                return path
        return None

    def train_forecast_model(self):
        if (
            self.train_process
            and self.train_process.state() != QProcess.NotRunning
        ):
            return

        gesture = self.gesture.currentText().strip().lower()
        participants = self._select_three_training_participants()
        if participants is None:
            return

        trial_dirs = [
            self.dataset_root / participant / gesture / "rep_01"
            for participant in participants
        ]

        trainer = self._forecast_trainer_script()
        if trainer is None:
            QMessageBox.warning(
                self,
                "TRAIN MODEL",
                "Training script not found.\n\nExpected one of:\n"
                f"{self.project_root / 'experiment_backend' / 'train_live_wrench_predictor.py'}\n"
                f"{self.project_root / 'tools' / 'train_live_wrench_predictor.py'}",
            )
            return

        data_python = (
            self.project_root
            / ".venv_data"
            / "bin"
            / "python3"
        )
        if not data_python.is_file():
            QMessageBox.warning(
                self,
                "TRAIN MODEL",
                f"Scientific Python environment not found:\n{data_python}",
            )
            return

        output_dir = self._forecast_model_dir(
            gesture,
            participants,
        )
        artifact = output_dir / "model_artifact.json"
        training_id = self._forecast_training_id(participants)

        if artifact.is_file():
            answer = QMessageBox.question(
                self,
                "Retrain forecast model",
                f"Training set: {training_id}\n"
                f"Gesture: {gesture}\n\n"
                f"A model already exists:\n{artifact}\n\n"
                "Retrain and replace the files in this model folder?",
                QMessageBox.StandardButton.Yes
                | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        args = []
        for trial_dir in trial_dirs:
            args.extend(["--trial-dir", trial_dir])

        # Keep the deployment preprocessing explicit and identical to the
        # validated proactive prototype: 60 Hz model, 100 ms horizon,
        # 2 Hz motion LPF, 1 Hz wrench LPF.
        args.extend(
            [
                "--horizon-ms",
                "100",
                "--model-rate-hz",
                "60",
                "--motion-cutoff-hz",
                "2.0",
                "--wrench-cutoff-hz",
                "1.0",
                "--output",
                output_dir,
            ]
        )

        if trainer.parent.name == "experiment_backend":
            # Run package code as a module so imports such as
            # "from experiment_backend..." resolve from the project root.
            trainer_entry = (
                "-m experiment_backend."
                + trainer.stem
            )
        else:
            # Fallback for a standalone tools/ copy.
            trainer_entry = shlex.quote(str(trainer))

        command = (
            f"cd {shlex.quote(str(self.project_root))} && "
            f"{shlex.quote(str(data_python))} "
            f"{trainer_entry} "
            + " ".join(shlex.quote(str(x)) for x in args)
        )

        self._train_active = True
        self._train_log_tail = []
        self._active_training_output_dir = output_dir
        self._active_training_id = training_id
        self.status.setText("TRAINING MODEL...")
        self.train_btn.setEnabled(False)

        self.train_process = QProcess(self)
        self.train_process.setProcessChannelMode(QProcess.MergedChannels)
        self.train_process.readyReadStandardOutput.connect(
            self._forecast_training_output
        )
        self.train_process.finished.connect(
            self._forecast_training_finished
        )
        self.train_process.start(
            "/bin/bash",
            ["-lc", command],
        )

    def _forecast_training_output(self):
        if self.train_process is None:
            return

        output_text = bytes(
            self.train_process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="replace",
        )

        for line in output_text.splitlines():
            clean = line.strip()
            if not clean:
                continue
            self._train_log_tail.append(clean)
            self._train_log_tail = self._train_log_tail[-40:]

            if clean.startswith("LOAD "):
                self.status.setText("TRAINING: LOADING")
            elif clean.startswith("=== "):
                self.status.setText(
                    "TRAINING " + clean.strip("= ").upper()
                )
            elif clean == "TRAINING COMPLETE":
                self.status.setText("MODEL READY")

    def _forecast_training_finished(self, code, status):
        del status
        self._train_active = False

        output_dir = getattr(
            self,
            "_active_training_output_dir",
            None,
        )
        training_id = getattr(
            self,
            "_active_training_id",
            "",
        )

        artifact = (
            output_dir / "model_artifact.json"
            if output_dir is not None
            else None
        )
        report = (
            output_dir / "training_report.json"
            if output_dir is not None
            else None
        )

        if (
            code == 0
            and artifact is not None
            and report is not None
            and artifact.is_file()
            and report.is_file()
        ):
            self.status.setText("MODEL READY")
            self.train_btn.setEnabled(
                self._forecast_training_ready()
            )
            QMessageBox.information(
                self,
                "Forecast model ready",
                f"Training: {training_id}\n\n"
                f"Model:\n{artifact}\n\n"
                f"Report:\n{report}",
            )
        else:
            self.status.setText("TRAIN ERROR")
            self.train_btn.setEnabled(
                self._forecast_training_ready()
            )
            tail = "\n".join(self._train_log_tail[-12:])
            QMessageBox.critical(
                self,
                "Forecast model training failed",
                "The training process failed."
                + (f"\n\nLast output:\n{tail}" if tail else ""),
            )

    def build_reference(self):
        if (
            self.build_process
            and self.build_process.state() != QProcess.NotRunning
        ):
            return

        self._build_active = True
        self.status.setText("BUILDING...")
        self.build_btn.setEnabled(False)
        self.build_process = QProcess(self)
        self.build_process.setProcessChannelMode(QProcess.MergedChannels)
        self.build_process.finished.connect(self._build_finished)
        args = [
            "--participant",
            self.participant.text().strip(),
            "--gesture",
            self.gesture.currentText().lower(),
        ]
        if self._mode() == "simulation":
            args.append("--simulation")
        if self.zero_world_x.isChecked():
            args.append("--zero-world-x")
        args.append("--run-ik")

        data_python = (
            self.project_root
            / ".venv_data"
            / "bin"
            / "python3"
        )

        if not data_python.exists():
            QMessageBox.warning(
                self,
                "BUILD REFERENCE",
                f"Scientific Python environment not found:\n{data_python}",
            )
            self._build_active = False
            self.build_btn.setEnabled(True)
            return

        self.build_process.start(
            "/bin/bash",
            [
                "-lc",
                self._cmd(
                    "experiment_backend.build_robot_reference",
                    args,
                    python_executable=data_python,
                ),
            ],
        )

    def _build_finished(self, code, status):
        self._build_active = False
        if code == 0:
            self.reload_selected_dataset()
        else:
            self.status.setText("BUILD ERROR")
            self.build_btn.setEnabled(self._recordings_complete)

    def _toggle_initial_force_summary(self, expanded):
        self.initial_force_toggle.setText(
            f"{'▾' if expanded else '▸'} INITIAL FORCE REFERENCE"
        )
        self.initial_force_label.setVisible(expanded)

    def refresh_initial_force_summary(self):
        path = self._selected_gesture_dir() / "reference" / "tension_summary.json"
        try:
            text = reference_helpers(self.project_root).initial_force_summary_text(path)
        except (OSError, ImportError, ValueError):
            text = "INITIAL FORCE REFERENCE\nInitial force reference: unavailable"
        # Keep the shared summary data/semantics; adapt only its presentation.
        lines = [line for line in text.splitlines() if line not in (
            "INITIAL FORCE REFERENCE",
            "Saved initial window (not the time-varying RC reference)",
        )]
        text = "\n".join(lines)
        for axis in ("Fx", "Fy", "Fz"):
            text = text.replace(f"{axis} mean:", f"{axis}:")
        self.initial_force_label.setText(text)
        self.initial_force_toggle.setChecked(False)
        self._toggle_initial_force_summary(False)
        self.initial_force_label.setToolTip(str(path))

    def open_replay(self):
        self.refresh_initial_force_summary()
        reference_replay = (
            self._selected_gesture_dir()
            / "reference"
            / "replay.html"
        )

        if reference_replay.exists():
            QProcess.startDetached(
                "xdg-open",
                [str(reference_replay)],
            )
            return

        if self.last_trial_dir and (
            self.last_trial_dir / "replay.html"
        ).exists():
            QProcess.startDetached(
                "xdg-open",
                [str(self.last_trial_dir / "replay.html")],
            )
            return

        QMessageBox.information(
            self,
            "Replay",
            "Build the final reference first to generate reference/replay.html",
        )

    def run_selected_gesture(self):
        if self.sim_process and self.sim_process.state() != QProcess.NotRunning:
            return

        gesture = self.gesture.currentText().strip().lower()
        mode = self._mode()
        reference_dir = self._selected_gesture_dir() / "reference"
        start_dir = reference_dir if reference_dir.is_dir() else self.project_root

        csv_name, _selected_filter = QFileDialog.getOpenFileName(
            self,
            f"Select {self.gesture.currentText()} Robot1 trajectory CSV",
            str(start_dir),
            "CSV trajectories (*.csv);;All files (*)",
        )
        if not csv_name:
            return

        csv_path = Path(csv_name).expanduser()
        script = self.project_root / "task_space" / "execute_single_gesture.py"
        if not csv_path.is_file() or not script.is_file():
            QMessageBox.warning(
                self,
                "Single gesture run",
                "Selected CSV or single-gesture runner is missing.",
            )
            return

        if mode == "real":
            answer = QMessageBox.question(
                self,
                "Confirm REAL robot motion",
                f"Gesture: {self.gesture.currentText()}\n"
                f"Robot CSV: {csv_path}\n\n"
                "Robot1 will be checked/moved to the gesture q0 and Robot2 "
                "to anchor over 5 s if required. The selected forward CSV "
                "will then run on the real robot. Recording stops at the "
                "forward trajectory end, before Robot1 returns to q0 in 1 s.\n\n"
                "Continue?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        args = [
            "--mode",
            mode,
            "--gesture",
            gesture,
            "--csv",
            csv_path,
            "--participant",
            self.participant.text().strip() or "unknown",
        ]
        if mode == "simulation":
            args.append("--confirm-fake-hardware")
        else:
            args.extend(["--confirm-real-hardware", "--record-bag"])

        cmd = (
            f"cd {shlex.quote(str(self.project_root))} && "
            f"{shlex.quote(sys.executable)} {shlex.quote(str(script))} "
            + " ".join(shlex.quote(str(x)) for x in args)
        )

        self._sim_active = True
        self._sim_log_tail = []
        self.status.setText(
            "REAL RUN STARTING" if mode == "real" else "SIMULATION STARTING"
        )
        self.sim_btn.setEnabled(False)

        self.sim_process = QProcess(self)
        self.sim_process.setProcessChannelMode(QProcess.MergedChannels)
        self.sim_process.readyReadStandardOutput.connect(
            self._simulation_output
        )
        self.sim_process.finished.connect(self._simulation_finished)
        self.sim_process.start("/bin/bash", ["-lc", cmd])

    def _simulation_output(self):
        text = bytes(
            self.sim_process.readAllStandardOutput()
        ).decode(errors="replace")
        prefix = "REAL" if self._mode() == "real" else "SIM"
        for line in text.splitlines():
            print(line, flush=True)
            self._sim_log_tail.append(line)
            self._sim_log_tail = self._sim_log_tail[-20:]
            if line.startswith("EVENT MOVING_TO_START"):
                self.status.setText(f"{prefix} -> START")
            elif line.startswith("EVENT RECORDING_FORWARD"):
                self.status.setText(
                    "REAL RECORDING" if prefix == "REAL" else "SIM RUNNING"
                )
            elif line.startswith("EVENT FORWARD_COMPLETE"):
                self.status.setText("FORWARD COMPLETE")
            elif line.startswith("EVENT RECORDING_STOPPED"):
                self.status.setText("RECORDING STOPPED")
            elif line.startswith("EVENT RETURNING_TO_Q0"):
                self.status.setText("RETURN -> q0")
            elif line.startswith("EVENT COMPLETE"):
                self.status.setText(f"{prefix} COMPLETE")

    def _simulation_finished(self, code, status):
        prefix = "REAL" if self._mode() == "real" else "SIM"
        if code == 0:
            self.status.setText(f"{prefix} COMPLETE")
        else:
            self.status.setText(f"{prefix} ERROR ({code})")
            tail = "\n".join(self._sim_log_tail[-12:]) or "No runner output captured."
            QMessageBox.warning(
                self,
                "Single gesture run failed",
                f"Runner exited with code {code}.\n\n{tail}",
            )
        QTimer.singleShot(2500, self._release_simulation_status)

    def _release_simulation_status(self):
        self._sim_active = False
        self.refresh_input_health()
