#!/usr/bin/env python3
"""Advisory edit coordination for Codex and Claude in one Linux workspace.

This lock records who may edit; it does not prevent other programs from writing.
Locks never expire automatically. A crashed owner's lock requires a handoff and
deliberate recovery after confirming that owner has stopped editing.
"""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import secrets
import sys


DEFAULT_STATE_DIR = Path(__file__).resolve().parents[1] / '.collaboration'


class LockError(Exception):
    """A lock could not safely be acquired, inspected, or released."""


def read_record(stream):
    """Read a complete lock record, rejecting partial or malformed state."""
    try:
        record = json.load(stream)
    except (ValueError, UnicodeError) as error:
        raise LockError('Lock is unreadable; treat it as occupied.') from error
    if (
        not isinstance(record, dict)
        or record.get('version') != 1
        or record.get('agent') not in ('codex', 'claude')
        or not isinstance(record.get('task'), str)
        or not record['task'].strip()
        or not isinstance(record.get('created_at'), str)
        or not record['created_at'].strip()
        or not isinstance(record.get('token'), str)
        or re.fullmatch(r'[0-9a-f]{32}', record['token']) is None
    ):
        raise LockError('Lock is malformed; treat it as occupied.')
    return record


def status(path):
    """Inspect without creating directories or modifying the lock."""
    try:
        with path.open(encoding='utf-8') as stream:
            record = read_record(stream)
    except FileNotFoundError:
        # A dangling symlink is still an occupied pathname for exclusive claim.
        if path.is_symlink():
            raise LockError('Lock is a dangling symlink; treat it as occupied.')
        return {'state': 'available'}
    return {'state': 'held', 'agent': record['agent'], 'task': record['task'],
            'created_at': record['created_at']}


def claim(path, agent, task):
    """Create the lock exclusively and return the unique ownership token."""
    if not task.strip():
        raise LockError('Task must contain a description.')
    record = {
        'version': 1,
        'agent': agent,
        'task': task.strip(),
        'created_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'token': secrets.token_hex(16),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open('x', encoding='utf-8') as stream:
            json.dump(record, stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
    except FileExistsError as error:
        raise LockError('Edit lock already exists. Run status; do not overwrite it.') from error
    # If writing fails, leave the partial file in place conservatively.
    return record['token']


def release(path, token):
    """Remove only the matching lock, serializing simultaneous releases."""
    # NOFOLLOW prevents accidentally interpreting a redirected lock as ours.
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(descriptor, encoding='utf-8') as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        record = read_record(stream)
        if not secrets.compare_digest(record['token'].encode('ascii'),
                                      token.encode('utf-8')):
            raise LockError('Token does not match; edit lock was left unchanged.')
        opened = os.fstat(stream.fileno())
        current = path.lstat()
        # A second release may have opened this inode before the first removed
        # it. Never let that second release delete a newly acquired lock.
        if (opened.st_dev, opened.st_ino) != (current.st_dev, current.st_ino):
            raise LockError('Edit lock changed; current lock was left unchanged.')
        path.unlink()


def main(argv=None):
    """Run the local edit-lock command line interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state-dir', type=Path, default=DEFAULT_STATE_DIR,
                        help='Override state directory for isolated testing.')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status', help='Read lock status without changing it.')
    acquire = commands.add_parser('claim', help='Claim exclusive editing; print token.')
    acquire.add_argument('--agent', choices=('codex', 'claude'), required=True)
    acquire.add_argument('--task', required=True)
    relinquish = commands.add_parser('release', help='Release using your claim token.')
    relinquish.add_argument('--token', required=True)
    args = parser.parse_args(argv)
    path = args.state_dir / 'edit-lock.json'
    try:
        if args.command == 'status':
            print(json.dumps(status(path), indent=2))
        elif args.command == 'claim':
            print(claim(path, args.agent, args.task))
        else:
            release(path, args.token)
            print('Released edit lock.')
    except (LockError, OSError) as error:
        print(f'collab: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
