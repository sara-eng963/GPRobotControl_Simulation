#!/usr/bin/env python3
"""Apply a delivery in a NEW checkout; leave the existing repository untouched.

Local staged/unstaged/untracked work remains in the source checkout and is not
silently copied or merged into the new checkout. No reset, stash, clean or push.
"""
import argparse
from pathlib import Path
import subprocess
import sys

BASE = '09f45623f3accb6c4099b7b6deb9f62874909878'
BRANCH = 'sara/can-replacement'


def git(*args, cwd=None):
    return subprocess.check_output(['git', '--no-optional-locks', *map(str, args)], cwd=cwd, text=True,
                                   stderr=subprocess.STDOUT).strip()


def install(source, destination, patch):
    source = Path(git('rev-parse', '--show-toplevel', cwd=source)).resolve()
    destination = destination.resolve()
    if destination == source or source in destination.parents:
        raise ValueError('Destination must be outside the existing repository')
    if destination.exists():
        raise ValueError('Destination already exists; nothing will be overwritten')
    if not patch.is_file():
        raise ValueError('Implementation patch not found: ' + str(patch))
    head = git('rev-parse', '--verify', 'refs/heads/' + BRANCH, cwd=source)
    git('merge-base', '--is-ancestor', BASE, head, cwd=source)
    dirty = git('status', '--porcelain', '--untracked-files=normal', cwd=source)
    print('Source checkout will remain unchanged.', flush=True)
    if dirty:
        print('Local work detected. It stays in the original checkout; it is NOT merged into this installation.', flush=True)
    # Independent objects: no shared .git directory and no branch changes in source.
    git('clone', '--no-hardlinks', '--branch', BRANCH, '--', source, destination)
    try:
        git('apply', '--check', patch, cwd=destination)
        git('apply', patch, cwd=destination)
    except subprocess.CalledProcessError as error:
        raise RuntimeError('Patch could not apply cleanly. Original checkout is untouched. '
                           'New checkout retained for inspection at ' + str(destination) + '\n' + error.output) from error
    print('Corrected implementation installed, uncommitted, at:', destination, flush=True)
    print('Your original local work is still at:', source, flush=True)
    print('Next: follow WSL_INSTALL.md from the ZIP, working in the NEW checkout.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--destination', required=True, type=Path)
    parser.add_argument('--patch', type=Path,
                        default=Path(__file__).resolve().with_name('unified-robot-simulation.patch'))
    ns = parser.parse_args()
    install(ns.repo.resolve(), ns.destination, ns.patch.resolve())


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError) as error:
        print('INSTALLATION REFUSED:', error, file=sys.stderr)
        sys.exit(1)
