"""Edge-triggered menu controls, separate from the continuous driving stick."""


class ControllerNavigation:
    def __init__(self):
        self.held = set()

    def update(self, axes, buttons):
        active = set()
        def pressed(i): return i < len(buttons) and bool(buttons[i])
        x = axes[6] if len(axes) > 6 else 0
        y = axes[7] if len(axes) > 7 else 0
        if x < -.55 or pressed(4): active.add('previous')
        if x > .55 or pressed(5): active.add('next')
        if y < -.55: active.add('up')
        if y > .55: active.add('down')
        # F310 reports its face-button A as index 0.  The screen labels use A
        # for confirmation, so do not route that action through X (index 2).
        if pressed(0): active.add('action')
        if pressed(2): active.add('x_action')
        if pressed(3): active.add('alternate')
        if pressed(6): active.add('menu')
        events = [key for key in ('previous','next','up','down','action','x_action','alternate','menu')
                  if key in active and key not in self.held]
        self.held = active
        return events
