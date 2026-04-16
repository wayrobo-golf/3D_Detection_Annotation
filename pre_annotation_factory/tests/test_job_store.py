from pathlib import Path

from my_package.workflow.job_models import JobState
from my_package.workflow.job_store import JobStore


def test_job_store_creates_workspace_and_persists_state(tmp_path: Path):
    store = JobStore(tmp_path)

    state = JobState(
        job_id="job-001",
        status="created",
        current_step="created",
    )

    job_dir = store.initialize_job_dir("job-001")
    store.save_state("job-001", state)

    loaded = store.load_state("job-001")

    assert job_dir == tmp_path / "jobs" / "job-001"
    assert (job_dir / "artifacts").is_dir()
    assert loaded.status == "created"
    assert loaded.current_step == "created"
