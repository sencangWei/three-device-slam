import ctypes
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def bridge(tmp_path):
    if shutil.which("cc") is None:
        pytest.skip("cc is required")
    include = tmp_path / "include"
    include.mkdir()
    (include / "extunit.h").write_text(
        """
int xu_get_len(int fd, int unit, int selector, unsigned long *size);
int xu_get_cur(int fd, int unit, int selector, unsigned long size, unsigned char *out);
""",
        encoding="utf-8",
    )
    fake = tmp_path / "fake_extunit.c"
    fake.write_text(
        """
static int get_len_calls;
static int get_cur_calls;
static unsigned long reported_size = 27;

int xu_get_len(int fd, int unit, int selector, unsigned long *size) {
    (void)fd; (void)unit; (void)selector;
    get_len_calls++;
    *size = reported_size;
    return 0;
}

int xu_get_cur(int fd, int unit, int selector, unsigned long size, unsigned char *out) {
    (void)fd; (void)unit; (void)selector;
    get_cur_calls++;
    for (unsigned long i = 0; i < size; i++) out[i] = (unsigned char)i;
    return 0;
}

int fake_get_len_calls(void) { return get_len_calls; }
int fake_get_cur_calls(void) { return get_cur_calls; }
void fake_set_reported_size(unsigned long size) { reported_size = size; }
""",
        encoding="utf-8",
    )
    output = tmp_path / "bridge.so"
    result = subprocess.run(
        [
            "cc",
            "-std=c11",
            "-shared",
            "-fPIC",
            f"-I{include}",
            str(ROOT / "native" / "2uq2_xu_bridge" / "bridge.c"),
            str(fake),
            "-o",
            str(output),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    library = ctypes.CDLL(str(output))
    library.ylx_open.argtypes = [ctypes.c_char_p]
    library.ylx_open.restype = ctypes.c_int
    library.ylx_read_imu27.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint8)]
    library.ylx_read_imu27.restype = ctypes.c_int
    library.ylx_close.argtypes = [ctypes.c_int]
    library.ylx_close.restype = ctypes.c_int
    return library


def test_bridge_validates_length_once_then_only_gets_current_data(bridge, tmp_path):
    device = tmp_path / "video0"
    device.touch()

    descriptor = bridge.ylx_open(str(device).encode())
    assert descriptor >= 0
    try:
        output = (ctypes.c_uint8 * 27)()
        assert bridge.ylx_read_imu27(descriptor, output) == 0
        assert bridge.ylx_read_imu27(descriptor, output) == 0
    finally:
        bridge.ylx_close(descriptor)

    assert bridge.fake_get_len_calls() == 1
    assert bridge.fake_get_cur_calls() == 2


def test_bridge_rejects_non_27_byte_selector_during_open(bridge, tmp_path):
    device = tmp_path / "video0"
    device.touch()
    bridge.fake_set_reported_size(26)

    assert bridge.ylx_open(str(device).encode()) == -27
    assert bridge.fake_get_len_calls() == 1
    assert bridge.fake_get_cur_calls() == 0
