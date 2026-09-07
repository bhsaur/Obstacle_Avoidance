"""Exercise edit-lock ownership through the CLI without ROS or simulator state."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'tools' / 'collab.py'


def run(state_dir, *arguments, script=SCRIPT, cwd=None):
    """Run an isolated helper without creating bytecode or test caches."""
    return subprocess.run(
        [sys.executable, '-B', str(script), '--state-dir', str(state_dir), *arguments],
        cwd=cwd, capture_output=True, text=True, check=False, timeout=10,
    )


def acquire(state_dir, agent='codex'):
    """Claim editing for a test task."""
    return run(state_dir, 'claim', '--agent', agent, '--task', 'Review changes')


def test_status_is_read_only(tmp_path):
    state_dir = tmp_path / 'missing'
    result = run(state_dir, 'status')
    assert result.returncode == 0
    assert json.loads(result.stdout) == {'state': 'available'}
    assert not state_dir.exists()


def test_claim_status_release_and_fresh_token(tmp_path):
    first = acquire(tmp_path)
    assert first.returncode == 0, first.stderr
    token = first.stdout.strip()
    assert len(token) == 32
    record = json.loads((tmp_path / 'edit-lock.json').read_text())
    assert record['token'] == token
    result = run(tmp_path, 'status')
    assert result.returncode == 0
    info = json.loads(result.stdout)
    assert info['state'] == 'held'
    assert info['agent'] == 'codex'
    assert info['task'] == 'Review changes'
    assert 'token' not in info
    assert run(tmp_path, 'release', '--token', token).returncode == 0
    assert not (tmp_path / 'edit-lock.json').exists()
    second = acquire(tmp_path, 'claude')
    assert second.returncode == 0
    assert second.stdout.strip() != token


@pytest.mark.parametrize('agent', ['codex', 'claude'])
def test_existing_claim_rejected_even_for_same_agent(tmp_path, agent):
    assert acquire(tmp_path).returncode == 0
    path = tmp_path / 'edit-lock.json'
    original = path.read_bytes()
    assert acquire(tmp_path, agent).returncode != 0
    assert path.read_bytes() == original


def test_wrong_or_old_token_cannot_release(tmp_path):
    token = acquire(tmp_path).stdout.strip()
    path = tmp_path / 'edit-lock.json'
    original = path.read_bytes()
    assert run(tmp_path, 'release', '--token', 'wrong').returncode != 0
    assert run(tmp_path, 'release', '--token', 'wrong-\u03b1').returncode != 0
    assert path.read_bytes() == original
    assert run(tmp_path, 'release', '--token', token).returncode == 0
    assert acquire(tmp_path, 'claude').returncode == 0
    replacement = path.read_bytes()
    assert run(tmp_path, 'release', '--token', token).returncode != 0
    assert path.read_bytes() == replacement


def test_concurrent_claims_have_exactly_one_winner(tmp_path):
    agents = ['codex', 'claude'] * 6
    with ThreadPoolExecutor(max_workers=len(agents)) as pool:
        results = list(pool.map(lambda agent: acquire(tmp_path, agent), agents))
    winners = [result for result in results if result.returncode == 0]
    assert len(winners) == 1
    token = winners[0].stdout.strip()
    assert json.loads((tmp_path / 'edit-lock.json').read_text())['token'] == token
    assert run(tmp_path, 'release', '--token', token).returncode == 0


@pytest.mark.parametrize('content', [b'', b'{', b'{}', b'[]', b'\xff'])
def test_corrupt_lock_blocks_claim_and_release(tmp_path, content):
    path = tmp_path / 'edit-lock.json'
    path.write_bytes(content)
    assert run(tmp_path, 'status').returncode != 0
    assert acquire(tmp_path).returncode != 0
    assert run(tmp_path, 'release', '--token', 'anything').returncode != 0
    assert path.read_bytes() == content


def test_dangling_symlink_remains_occupied(tmp_path):
    path = tmp_path / 'edit-lock.json'
    path.symlink_to(tmp_path / 'missing.json')
    assert run(tmp_path, 'status').returncode != 0
    assert acquire(tmp_path).returncode != 0
    assert run(tmp_path, 'release', '--token', 'anything').returncode != 0
    assert path.is_symlink()


def test_default_path_anchored_to_script_not_current_directory(tmp_path):
    project = tmp_path / 'project'
    tools = project / 'tools'
    tools.mkdir(parents=True)
    script = tools / 'collab.py'
    shutil.copyfile(SCRIPT, script)
    unrelated = tmp_path / 'elsewhere'
    unrelated.mkdir()
    result = subprocess.run(
        [sys.executable, '-B', str(script), 'claim', '--agent', 'codex',
         '--task', 'Check default directory'],
        cwd=unrelated, capture_output=True, text=True, check=False, timeout=10,
    )
    assert result.returncode == 0, result.stderr
    assert (project / '.collaboration' / 'edit-lock.json').is_file()
    assert not (unrelated / '.collaboration').exists()


def test_blank_task_does_not_create_lock(tmp_path):
    result = run(tmp_path, 'claim', '--agent', 'codex', '--task', '   ')
    assert result.returncode != 0
    assert not (tmp_path / 'edit-lock.json').exists()
