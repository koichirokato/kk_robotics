import dataclasses
import tkinter as tk


@dataclasses.dataclass
class WindowSize:
    width: int
    height: int


@dataclasses.dataclass
class Position:
    x: float
    y: float


_WINDOS_SIZE = WindowSize(200, 300)
_CANVAS_SIZE = WindowSize(200, 200)
_CIRCLE_SIZE = 20


class MousePosition(tk.Frame):
    def __init__(self):
        self._root = tk.Tk()
        super().__init__(self._root, height=_WINDOS_SIZE.height, width=_WINDOS_SIZE.width)

        self._root.title("Tk JoyStick")
        canvas = tk.Canvas(
            self._root,
            background="white",
            height=_CANVAS_SIZE.height,
            width=_CANVAS_SIZE.width,
        )
        self._canvas = canvas
        self._virtual_stick_circle = canvas.create_oval(
            _CANVAS_SIZE.width / 2 - _CIRCLE_SIZE,
            _CANVAS_SIZE.height / 2 - _CIRCLE_SIZE,
            _CANVAS_SIZE.width / 2 + _CIRCLE_SIZE,
            _CANVAS_SIZE.height / 2 + _CIRCLE_SIZE,
            tag="oval",
        )
        canvas.create_line(_CANVAS_SIZE.width / 2, 0, _CANVAS_SIZE.width / 2, _CANVAS_SIZE.height)
        canvas.create_line(0, _CANVAS_SIZE.height / 2, _CANVAS_SIZE.width, _CANVAS_SIZE.height / 2)
        canvas.grid(row=0, column=0)
        canvas.bind("<Motion>", self._mouse_callback)
        canvas.bind("<Button-1>", self._click_callback)

        self._position_label = tk.Label(self._root)
        self._position_label.grid(row=1, column=0)

        self._last_position = Position(0.0, 0.0)
        self._is_clicked = False

    def _mouse_callback(self, event):
        position_from_center = Position(
            event.x - 1 / 2 * _CANVAS_SIZE.width,
            1 / 2 * _CANVAS_SIZE.height - event.y,
        )

        if self._is_clicked:
            self._position_label["text"] = (
                str(position_from_center.x) + ", " + str(position_from_center.y)
            )
            self._canvas.move(
                self._virtual_stick_circle,
                -1 * (self._last_position.x - position_from_center.x),
                self._last_position.y - position_from_center.y,
            )
            self._last_position.x = position_from_center.x
            self._last_position.y = position_from_center.y

    def _click_callback(self, event: tk.Event) -> None:
        del event  # unused
        if self._is_clicked:
            self._canvas.config(background="red")
            self._position_label["text"] = "Stop"
            self._is_clicked = False
        else:
            self._canvas.config(background="white")
            self._is_clicked = True

    def get_position(self) -> tuple[float, float]:
        if self._is_clicked:
            return (self._last_position.x, self._last_position.y)
        else:
            return (0.0, 0.0)

    def update(self) -> None:
        self._root.update()


if __name__ == "__main__":
    import time

    controller = MousePosition()
    for _ in range(10000):
        controller.update()
        time.sleep(0.01)
