#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tmr_listener.srv import EthernetSlaveCmd
from threading import Thread
from subprocess import Popen

import signal
import rostest
import rospy
import time
import SocketServer
import unittest

DEFAULT_TIMEOUT_SIGINT = 15.0
DEFAULT_TIMEOUT_SIGTERM = 2.0


class RequestHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        self.request.close()


class EthernetSlaveShutdownTest(unittest.TestCase):
    def __init__(self, *args):
        rospy.init_node('EthernetSlaveShutdownTest', anonymous=True)
        super(EthernetSlaveShutdownTest, self).__init__(*args)

    def setUp(self):
        self.process = Popen(
            ['rosrun', 'tmr_listener', 'tmr_eth_slave_node', '--ip=127.0.0.1']
        )

        self.server = SocketServer.TCPServer(
            ('127.0.0.1', 5891), RequestHandler, bind_and_activate=False)
        self.server.allow_reuse_address = True

        self.service = rospy.ServiceProxy(
            '/tmr_eth_slave/tmsvr_cmd', EthernetSlaveCmd)
        self.service.wait_for_service()
        self.serivce_thread = Thread(target=self.service, kwargs={
            'id': 'Q3', 'item_list': ['TCP_Mass']})

        time.sleep(2)

    def tearDown(self):
        self.process.wait()
        self.server.server_close()

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

    def test_shutdown_normally_during_service_call(self):
        self.server.server_bind()
        self.server.server_activate()

        time.sleep(1)
        self.serivce_thread.start()

        time.sleep(1)
        self._end_node()
        self.assertEqual(self.process.returncode, 0)


if __name__ == '__main__':
    rostest.rosrun('tmr_listener', 'EthernetSlaveShutdownTest',
                   EthernetSlaveShutdownTest)
