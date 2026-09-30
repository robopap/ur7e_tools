"""Load the experiment repository's standard-library reference helpers lazily."""
from functools import lru_cache
import importlib.util
from pathlib import Path


@lru_cache(maxsize=4)
def reference_helpers(project_root=None):
    project_root = project_root or '~/phd_polishing_experiments'
    path = Path(project_root).expanduser().resolve() / 'experiment_backend' / 'participant_references.py'
    spec = importlib.util.spec_from_file_location('_direct_participant_references', path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Participant reference helpers unavailable: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
