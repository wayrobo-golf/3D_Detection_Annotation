from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from my_package.workflow.job_models import load_job_manifest
from my_package.workflow.job_store import JobStore
from my_package.workflow.pipeline import WorkflowPipeline


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="annotationctl")
    subparsers = parser.add_subparsers(dest="command", required=True)

    submit_parser = subparsers.add_parser("submit")
    submit_parser.add_argument("manifest")

    status_parser = subparsers.add_parser("status")
    status_parser.add_argument("job_id")
    status_parser.add_argument("--workspace", required=True)

    finalize_parser = subparsers.add_parser("finalize")
    finalize_parser.add_argument("job_id")
    finalize_parser.add_argument("--workspace", required=True)

    return parser


def run_submit(manifest_path: Path) -> int:
    manifest = load_job_manifest(manifest_path)
    pipeline = WorkflowPipeline(job_store=JobStore(manifest.workspace.root_dir))
    job_id = pipeline.submit(manifest)

    job_dir = pipeline.job_store.jobs_root / job_id
    shutil.copy2(manifest_path, job_dir / "job.yaml")

    state = pipeline.job_store.load_state(job_id)
    print(f"job_id={job_id}")
    print(f"status={state.status}")
    return 0


def run_status(workspace: Path, job_id: str) -> int:
    store = JobStore(workspace)
    state = store.load_state(job_id)
    print(f"job_id={state.job_id}")
    print(f"status={state.status}")
    print(f"current_step={state.current_step}")
    return 0


def run_finalize(workspace: Path, job_id: str) -> int:
    pipeline = WorkflowPipeline(job_store=JobStore(workspace))
    pipeline.finalize(job_id)
    state = pipeline.job_store.load_state(job_id)
    print(f"job_id={state.job_id}")
    print(f"status={state.status}")
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "submit":
        return run_submit(Path(args.manifest))
    if args.command == "status":
        return run_status(Path(args.workspace), args.job_id)
    if args.command == "finalize":
        return run_finalize(Path(args.workspace), args.job_id)

    parser.error(f"unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
