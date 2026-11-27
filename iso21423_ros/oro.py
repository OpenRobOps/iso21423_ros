# Copyright 2026 InOrbit, Inc.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

"""Opt-in OpenRobOps extras: customData extension source and customCommand handler."""
import logging
import threading

from iso21423 import ResourceConfig, now_timestamp, register_extension_resource
from std_msgs.msg import String

log = logging.getLogger(__name__)

RESOURCE = "customData"


def parse_key_value(line):
    """'k=v' -> (k, v) splitting on the first '='; None when there is no key."""
    k, sep, v = line.partition("=")
    return (k, v) if sep and k else None


class CustomDataSource:
    """key=value String lines on `topic`, batched per `batch_ms` window into one customData message."""

    def __init__(self, node, publisher, params):
        self.pub = publisher
        self._window = float(params.get("batch_ms", 500)) / 1000
        self._lock = threading.Lock()
        self._pending = None
        self._timer = None
        # identical re-registration is a no-op in the library, so several instances are fine
        register_extension_resource(RESOURCE, ResourceConfig(qos=1, retain=False))
        node.create_subscription(String, params.get("topic", "/inorbit/custom_data"), self._on_msg, 10)

    def _on_msg(self, msg):
        kv = parse_key_value(msg.data)
        if not kv:
            return
        with self._lock:
            if self._pending is None:
                self._pending = {}
                self._timer = threading.Timer(self._window, self._flush)
                self._timer.daemon = True
                self._timer.start()
            self._pending[kv[0]] = kv[1]

    def _flush(self):
        with self._lock:
            values, self._pending, self._timer = self._pending, None, None
        if not values:
            return
        try:  # publish outside the lock
            self.pub.publish_extension(RESOURCE, {"timestamp": now_timestamp(), "values": values})
        except Exception:
            log.exception("customData publish failed")

    def close(self):
        """Publish the pending batch now (call at shutdown so the last batch is not lost)."""
        with self._lock:
            if self._timer:
                self._timer.cancel()
        self._flush()


class CustomCommandHandler:
    """customCommand action: republish properties.command as std_msgs/String on `topic`."""

    def __init__(self, node, params):
        self._pub = node.create_publisher(String, params.get("topic", "/inorbit/custom_command"), 10)

    def execute(self, action, ctx):
        props = action.get("properties")
        if props is None:
            props = {}
        if not isinstance(props, dict):
            return ctx.aborted("MALFORMED_REQUEST", "properties must be an object")
        command = props.get("command")
        if not isinstance(command, str):
            return ctx.aborted("MALFORMED_REQUEST", "properties.command must be a string")
        log.info("customCommand -> %s", command)
        self._pub.publish(String(data=command))
        return ctx.succeeded()
