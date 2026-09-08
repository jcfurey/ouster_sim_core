"""Run a packet congestion regression through its own bounded TCP router."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix


def main():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    endpoint = json.dumps([f'tcp/127.0.0.1:{port}'])
    env = dict(os.environ)
    for key in ('ZENOH_SESSION_CONFIG_URI', 'ZENOH_ROUTER_CONFIG_URI'):
        env.pop(key, None)
    env['RMW_IMPLEMENTATION'] = 'rmw_zenoh_cpp'
    env['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryDirectory(prefix='ouster_zenoh_burst_') as directory:
        env['ROS_LOG_DIR'] = directory
        env['ZENOH_CONFIG_OVERRIDE'] = (
            f'mode="router";listen/endpoints={endpoint};connect/endpoints=[];'
            'scouting/multicast/enabled=false'
        )
        executable = (
            Path(get_package_prefix('rmw_zenoh_cpp')) / 'lib/rmw_zenoh_cpp/rmw_zenohd'
        )
        with (Path(directory) / 'router.log').open('w+') as log:
            router = subprocess.Popen(
                [str(executable)], env=env, stdout=log, stderr=log
            )
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    try:
                        with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                            break
                    except OSError:
                        time.sleep(0.05)
                else:
                    log.seek(0)
                    raise RuntimeError('test router failed to start: ' + log.read())
                # One outbound batch and no grace period make congestion
                # reproducible with real Ouster-size messages. BLOCK traffic
                # still has a bounded wait if the transport becomes stuck.
                env['ZENOH_CONFIG_OVERRIDE'] = (
                    f'mode="client";connect/endpoints={endpoint};'
                    'scouting/multicast/enabled=false;'
                    'transport/link/tx/queue/size/data=1;'
                    'transport/link/tx/queue/congestion_control/drop/wait_before_drop=0;'
                    'transport/link/tx/queue/congestion_control/drop/max_wait_before_drop_fragments=0;'
                    'transport/link/tx/queue/congestion_control/block/wait_before_close=5000000'
                )
                # A regression may briefly suspend this owned router to force
                # BLOCK publication while checking simulation-thread progress.
                env['OUSTER_TEST_ROUTER_PID'] = str(router.pid)
                return subprocess.run(sys.argv[1:], env=env, timeout=30).returncode
            finally:
                router.send_signal(signal.SIGCONT)
                router.send_signal(signal.SIGINT)
                try:
                    router.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    router.kill()
                    router.wait()


if __name__ == '__main__':
    raise SystemExit(main())
