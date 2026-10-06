"""Data integrity and smoke tests for the module 5 vision programs."""

import os
from pathlib import Path
import pty
import re
import select
import shutil
import signal
import time

from ament_index_python.packages import get_package_prefix
import pytest

DATA = Path(__file__).resolve().parents[1] / 'data'
# Files the upstream course repository refers to but does not ship.
KNOWN_MISSING_UPSTREAM = {'Media/bricks.png'}
MEDIA_NAME = re.compile(r'^[\w./-]+\.(jpe?g|bmp|png|pgm|avi|xml|txt|sdf)$', re.IGNORECASE)


def input_files():
    return sorted(DATA.glob('*Input.txt'))


@pytest.mark.parametrize('input_file', input_files(), ids=lambda p: p.name)
def test_files_named_in_input_files_exist(input_file):
    missing = []
    for line in input_file.read_text(errors='replace').splitlines():
        for token in line.split():
            if (MEDIA_NAME.match(token) and token not in KNOWN_MISSING_UPSTREAM
                    and not (DATA / token).exists()):
                missing.append(token)
    assert not missing, f'{input_file.name} refers to missing files: {missing}'


def test_every_program_has_an_input_file():
    programs = [p.name for p in (Path(__file__).resolve().parents[1] / 'src').iterdir()]
    # programs without a data-driven input file (they take everything from the camera or ROS)
    exempt = {'imageAcquisitionFromSimulatorCamera', 'imageAcquisitionFromUSBCamera'}
    names = {p.name[:-len('Input.txt')] for p in input_files()}
    assert set(programs) - names - exempt == set()


def run_in_virtual_display(name, key=b'x', timeout=90.0):
    """Run a program under Xvfb on a pty, pressing a key until it exits; return (status, text)."""
    exe = os.path.join(get_package_prefix('module5'), 'lib', 'module5', name)
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp('xvfb-run', ['xvfb-run', '-a', '-s', '-screen 0 1280x1024x24', exe])
    output = b''
    deadline = time.time() + timeout
    status = None
    last_key = 0.0
    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.2)
        if ready:
            try:
                chunk = os.read(fd, 65536)
            except OSError:
                chunk = b''
            if not chunk:
                _, status = os.waitpid(pid, 0)
                break
            output += chunk
        if time.time() - last_key > 0.5:
            last_key = time.time()
            try:
                os.write(fd, key)
            except OSError:
                pass
        done, st = os.waitpid(pid, os.WNOHANG)
        if done:
            status = st
            break
    else:
        os.kill(pid, signal.SIGKILL)
        os.waitpid(pid, 0)
        pytest.fail(f'{name} did not finish within {timeout} s. Output:\n'
                    + output.decode(errors='replace')[-2000:])
    return status, output.decode(errors='replace')


@pytest.mark.skipif(shutil.which('xvfb-run') is None, reason='xvfb-run not installed')
@pytest.mark.parametrize('program', [
    'binaryThresholding', 'binaryThresholdingOtsu', 'colourToGreyscale', 'gaussianFiltering',
    'cannyEdgeDetection', 'sobelEdgeDetection', 'imageAcquisitionFromImageFile',
])
def test_image_programs_process_all_their_images(program):
    status, output = run_in_virtual_display(program)
    assert os.WIFEXITED(status), f'{program} crashed: status {status}\n{output[-2000:]}'
    assert os.WEXITSTATUS(status) == 0, output[-2000:]
    assert 'Error' not in output and 'error' not in output.lower().replace('terror', ''), output
