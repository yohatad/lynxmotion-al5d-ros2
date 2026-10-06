"""Test configuration: the CoraPlex tests are only collected when CoraPlex is installed."""

import importlib.util

collect_ignore = []

_HAS_CORAPLEX = all(importlib.util.find_spec(name) is not None
                    for name in ('coraplex', 'semantic_digital_twin'))
if not _HAS_CORAPLEX:
    collect_ignore.append('test_actions.py')


def pytest_report_header(config):
    """Say whether the CoraPlex tests are part of this run."""
    if _HAS_CORAPLEX:
        return 'module11: CoraPlex found, action tests enabled'
    return 'module11: CoraPlex NOT found, action tests (test_actions.py) are not collected'
