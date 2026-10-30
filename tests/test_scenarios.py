"""Parametrized over scenarios/*.yaml — the scenario suite.

Selection conventions:
  ./t -k weekly          # recurring weekly regression suite
  ./t -k selftest        # harness self-tests (machinery correctness)
  ./t -k demo            # week-4 demo script (long)
Every run writes its result + evidence to tests/artifacts/<name>.json.
"""
import glob
import json
import os

import pytest

from runner import run

HERE = os.path.dirname(__file__)
FILES = sorted(glob.glob(os.path.join(HERE, 'scenarios', '*.yaml')))
IDS = [os.path.basename(f)[:-5] for f in FILES]


ARTIFACTS = os.path.join(HERE, 'artifacts')   # gitignored; bind-mounted to the host


@pytest.mark.parametrize('path', FILES, ids=IDS)
def test_scenario(path):
    result = run(path)
    # evidence for passing runs too (captured numbers, not prose claims)
    os.makedirs(ARTIFACTS, exist_ok=True)
    with open(os.path.join(ARTIFACTS, f"{result['name']}.json"), 'w') as f:
        json.dump(result, f, indent=1, default=str)
    if result['expect_failure']:
        # machinery self-test: this scenario MUST fail, with evidence
        assert not result['passed'], (
            f"{result['name']} was expected to FAIL but passed — "
            f"the harness cannot be trusted to detect failures")
        assert result['failures'], "expected failure evidence, got none"
    else:
        assert result['passed'], (
            f"{result['name']} FAILED:\n  " +
            "\n  ".join(result['failures']) +
            f"\n  evidence: {result['evidence']}")
