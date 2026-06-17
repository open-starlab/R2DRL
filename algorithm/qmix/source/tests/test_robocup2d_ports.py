import unittest
from unittest.mock import patch

from envs.robocup2d.process import ports


class PickPortsTest(unittest.TestCase):
    def test_locked_port_block_uses_short_probe_before_next_candidate(self):
        wait_calls = []

        def fake_wait(block, timeout, poll, hold, check_ipv6, socket_types):
            del poll, hold
            wait_calls.append((list(block), timeout))
            self.assertFalse(check_ipv6)
            self.assertEqual(socket_types, (ports.socket.SOCK_DGRAM,))
            return len(wait_calls) == 2

        with patch.object(ports, "try_lock_port_block") as mock_lock:
            with patch.object(ports, "wait_ports_free", side_effect=fake_wait):
                with patch.object(ports.os, "close") as mock_close:
                    mock_lock.side_effect = lambda base: (int(base), f"/tmp/{base}.lock")

                    result = ports.pick_ports(
                        6000,
                        6020,
                        9,
                        1,
                        2,
                        8,
                        timeout=300.0,
                        block_probe_timeout=0.25,
                    )

        self.assertEqual(result[:5], (6009, 6009, 6010, 6011, 6017))
        self.assertEqual(wait_calls[0], ([6000, 6001, 6002], 0.25))
        self.assertEqual(wait_calls[1], ([6009, 6010, 6011], 0.25))
        mock_close.assert_called_once_with(6000)

    def test_error_reports_port_scan_diagnostics(self):
        with patch.object(ports, "try_lock_port_block", return_value=(123, "/tmp/test.lock")):
            with patch.object(ports, "wait_ports_free", return_value=False):
                with patch.object(ports, "first_unbindable_port", return_value=6000):
                    with patch.object(ports.os, "close"):
                        with self.assertRaisesRegex(RuntimeError, "checked_blocks=2"):
                            ports.pick_ports(
                                6000,
                                6018,
                                9,
                                1,
                                2,
                                8,
                                timeout=300.0,
                                block_probe_timeout=0.25,
                            )


if __name__ == "__main__":
    unittest.main()
