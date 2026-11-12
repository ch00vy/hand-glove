"""Control Center command wiring and SDK password compatibility, without hardware."""

import io
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from config import SDK_PYTHON
from rokoko_control_gui import App, PROJECT
from sdk_network_runner import read_password


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


class ControlCenterTests(unittest.TestCase):
    def setUp(self):
        self.app = App.__new__(App)
        self.app.sdk = Value(str(PROJECT / 'sdk folder'))
        self.app.project = Value(str(PROJECT))
        self.app.driver_port = Value('12376')
        self.app.solver_port = Value('12377')
        self.app.receiver_port = Value('14141')

    def test_custom_ports_connect_driver_solver_and_viewer(self):
        driver, _, _ = self.app.cmds('driver')
        solver, _, _ = self.app.cmds('solver')
        viewer, _, _ = self.app.cmds('viewer')
        self.assertEqual(driver[driver.index('--driver-udp-port') + 1], '14141')
        self.assertEqual(driver[driver.index('--rgmp-output-addr') + 1], '127.0.0.1:12376')
        self.assertEqual(solver[solver.index('--rgmp-addr') + 1], '127.0.0.1:12376')
        self.assertEqual(solver[solver.index('--listen') + 1], '127.0.0.1:12377')
        self.assertEqual(solver[solver.index('--emit') + 1], 'both')
        self.assertEqual(viewer[viewer.index('--port') + 1], '12377')

    def test_invalid_port_is_rejected_before_launch(self):
        for invalid in ('0', '65536', 'abc'):
            self.app.receiver_port = Value(invalid)
            with self.assertRaises(ValueError):
                self.app.cmds('driver')

    def test_stop_all_cancels_pending_launches(self):
        self.app.pipeline_jobs = ['solver-job', 'viewer-job']
        self.app.root = unittest.mock.Mock()
        self.app.stop_stage = unittest.mock.Mock()
        self.app.stop_all()
        self.assertEqual(self.app.pipeline_jobs, [])
        self.assertEqual(self.app.root.after_cancel.call_count, 2)
        self.assertEqual(self.app.stop_stage.call_count, 3)

    def test_password_preserves_spaces_and_requires_input(self):
        with patch('sys.stdin', io.StringIO('  test password  \n')):
            self.assertEqual(read_password(), '  test password  ')
        with patch('sys.stdin', io.StringIO('')):
            with self.assertRaises(RuntimeError):
                read_password()

    def test_runner_supplies_getpass_without_password_in_arguments(self):
        password = 'test spaces \u00e9 & $ secret'
        with tempfile.TemporaryDirectory() as folder:
            script = Path(folder) / 'fake sdk.py'
            script.write_text(
                'import getpass, sys\n'
                'password = getpass.getpass("Wi-Fi password: ")\n'
                'assert password not in sys.argv\n'
                'assert password == "test spaces \\u00e9 & $ secret"\n'
                'print("password received")\n', encoding='utf-8')
            result = subprocess.run(
                [sys.executable, '-B', str(PROJECT / 'sdk_network_runner.py'), str(script), 'set'],
                input=password + '\n', capture_output=True, text=True, encoding='utf-8',
                env={**os.environ, 'PYTHONIOENCODING': 'utf-8'}, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'password received')
        self.assertNotIn(password, result.stdout + result.stderr)

    @unittest.skipUnless((SDK_PYTHON / 'network_configuration.py').is_file(), 'SDK not installed')
    def test_installed_sdk_parser_accepts_password_from_prompt(self):
        # Load only definitions; no RCSP connection or device configuration.
        with patch.object(sys, 'path', [str(SDK_PYTHON), *sys.path]), \
                patch.object(sys, 'dont_write_bytecode', True):
            sdk = runpy.run_path(str(SDK_PYTHON / 'network_configuration.py'))
        with patch('getpass.getpass', return_value='test password') as prompt:
            args = sdk['parse_args']([
                'set', '--ssid', 'Test network', '--receiver-address', '192.0.2.1',
                '--receiver-port', '14041', '--security', 'WPA2'])
        self.assertEqual(args.password, 'test password')
        self.assertEqual(args.receiver_port, 14041)
        prompt.assert_called_once()


if __name__ == '__main__':
    unittest.main()
