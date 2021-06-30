#!/usr/bin/env python
# -*- coding: utf-8 -*-

from subprocess32 import Popen, STDOUT

import rospy
import signal
import rostest
import time
import SocketServer
import unittest

DEFAULT_TIMEOUT_SIGINT = 15.0
DEFAULT_TIMEOUT_SIGTERM = 2.0


class RequestHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        self.request.close()


class TestBareBones(unittest.TestCase):
    def setUp(self):
        self.process = Popen(['rosrun', 'tmr_listener',
                              'tmr_listener_node', '--ip=127.0.0.1'])
        self.server = SocketServer.TCPServer(
            ('127.0.0.1', 5890), RequestHandler, bind_and_activate=False)
        self.server.allow_reuse_address = True
        time.sleep(2)

    def tearDown(self):
        self.process.wait()
        self.server.server_close()

    def test_shutdown_normally_after_connection_established(self):
        self.server.server_bind()
        self.server.server_activate()

        self._end_node()
        self.assertEqual(self.process.returncode, 0)

    def test_shutdown_normally_before_connection_established(self):
        self._end_node()
        self.assertEqual(self.process.returncode, 0)

    def _end_node(self):
        def try_end_by(t_signum, t_timeout):
            timeout_time = time.time() + t_timeout
            self.process.send_signal(t_signum)

            retcode = self.process.poll()
            while time.time() < timeout_time and retcode is None:
                time.sleep(0.1)
                retcode = self.process.poll()
            return retcode is not None

        if not try_end_by(signal.SIGINT, DEFAULT_TIMEOUT_SIGINT):
            if not try_end_by(signal.SIGTERM, DEFAULT_TIMEOUT_SIGTERM):
                self.process.send_signal(signal.SIGKILL)


if __name__ == '__main__':
    rostest.rosrun('tmr_listener', 'test', TestBareBones)
