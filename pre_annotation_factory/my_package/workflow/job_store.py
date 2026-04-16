from __future__ import annotations

import json
from pathlib import Path

from .job_models import JobState


class JobStore:
    def __init__(self, workspace_root: Path):
        self.workspace_root = Path(workspace_root)
        self.jobs_root = self.workspace_root / "jobs"

    def initialize_job_dir(self, job_id: str) -> Path:
        job_dir = self.jobs_root / job_id
        for relative in (
            ".",
            "logs",
            "artifacts",
            "artifacts/extracted_bags",
            "artifacts/raw_origin",
            "artifacts/xtreme_upload",
            "artifacts/xtreme_export",
            "artifacts/final_dataset",
        ):
            (job_dir / relative).mkdir(parents=True, exist_ok=True)
        return job_dir

    def save_state(self, job_id: str, state: JobState) -> None:
        path = self.jobs_root / job_id / "state.json"
        path.write_text(state.to_json(), encoding="utf-8")

    def load_state(self, job_id: str) -> JobState:
        path = self.jobs_root / job_id / "state.json"
        return JobState.from_dict(json.loads(path.read_text(encoding="utf-8")))
