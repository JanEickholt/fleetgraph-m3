"""Parametrized over scenarios/*.yaml — the scenario suite.

Selection conventions:
  ./t -k weekly          # recurring weekly regression suite
  ./t -k selftest        # harness self-tests (machinery correctness)
"""
import glob
import os

import pytest

from runner import run, load

HERE = os.path.dirname(__file__)
FILES = sorted(glob.glob(os.path.join(HERE, 'scenarios', '*.yaml')))
IDS = [os.path.basename(f)[:-5] for f in FILES]


@pytest.mark.parametrize('path', FILES, ids=IDS)
def test_scenario(path):
    result = run(path)
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
