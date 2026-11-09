class Busy(RuntimeError):
    """The request clashes with something in progress or already running (HTTP 409)."""

    http_status = 409


class Conflict(Busy):
    """Another controller owns the command interfaces the requested one needs."""

    def __init__(self, running: list[str]) -> None:
        super().__init__(f"{', '.join(running)} running; stop it first or swap")
        self.running = running
