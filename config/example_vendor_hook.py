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

"""Example vendor hook: a custom ISO action `beep`, served by a small Python class.

Enable it in your params file (any path works, e.g. a file mounted into the container):

    handlers:
      beep:
        class: /path/to/example_vendor_hook.py:BeepHandler
        topic: /beep

`beep` then appears in the identity's `accepts` list. A fleet manager sends
{"type": "beep", "version": "1.0", "properties": {"duration_s": 0.5}}.
"""
from std_msgs.msg import Float32


class BeepHandler:
    def __init__(self, node, params):
        # node: the connector node (.config, .ccs, .tf_buffer, .status, .nav, .entity)
        # params: everything under handlers.beep except `class` and `enabled`
        self.pub = node.create_publisher(Float32, params.get("topic", "/beep"), 10)

    def execute(self, action, ctx):
        """Runs on its own worker thread, so blocking is fine. Return ctx.succeeded() or ctx.aborted()."""
        duration = (action.get("properties") or {}).get("duration_s", 0.5)
        if isinstance(duration, bool) or not isinstance(duration, (int, float)) or duration < 0:
            return ctx.aborted("MALFORMED_REQUEST", "properties.duration_s must be a number >= 0")
        self.pub.publish(Float32(data=float(duration)))
        if ctx.cancelled.wait(duration):  # cancelRequest (or preemption) sets ctx.cancelled
            return ctx.aborted("REJECTED", "canceled")
        return ctx.succeeded()
