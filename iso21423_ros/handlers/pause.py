"""pauseImr / resumeImr: flip node.nav state; the running move reacts for its own goal."""


class PauseHandler:
    def __init__(self, node, params):
        self.node = node

    def execute(self, action, ctx):
        self.node.nav.pause()
        return ctx.succeeded()


class ResumeHandler(PauseHandler):
    def execute(self, action, ctx):
        self.node.nav.resume()
        return ctx.succeeded()
