#!/usr/bin/env python3

import tkinter as tk


KEY_ACTIONS = {
    "g": "general",
    "ㅎ": "general",
    "m": "mission",
    "ㅡ": "mission",
    "p": "parking",
    "ㅔ": "parking",
    "c": "control_p",
    "ㅊ": "control_p",
    "l": "toggle_log",
    "ㅣ": "toggle_log",
}


class Selector:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("RUN")
        self.window_width = 480
        self.window_height = 400
        self.root.geometry(f"{self.window_width}x{self.window_height}")
        self.root.resizable(False, False)
        self.root.attributes("-topmost", True)
        self.root.configure(bg="#111318")
        self.root.protocol("WM_DELETE_WINDOW", self.cancel)

        content = tk.Frame(self.root, bg="#111318", padx=34, pady=26)
        content.pack(fill="both", expand=True)

        tk.Label(
            content,
            text="control_pkg에서 아두이노로 보내는 조향값 자체를 RUN",
            bg="#111318",
            fg="#F4F6FA",
            font=("DejaVu Sans", 24, "bold"),
        ).pack(anchor="w")
        tk.Label(
            content,
            text="키를 눌러 실행",
            bg="#111318",
            fg="#777D89",
            font=("DejaVu Sans", 11),
            pady=2,
        ).pack(anchor="w")

        rows = (
            ("G", "일반주행", "#45CFF5"),
            ("M", "미션주행", "#C084FC"),
            ("P", "주차", "#FACC15"),
            ("C", "CONTROL", "#4ADE80"),
            ("L", "로그 표시 ON / OFF", "#94A3B8"),
        )
        menu = tk.Frame(content, bg="#111318", pady=16)
        menu.pack(fill="x")
        for key, label, color in rows:
            row = tk.Frame(menu, bg="#1A1D24", height=47)
            row.pack(fill="x", pady=3)
            row.pack_propagate(False)
            tk.Label(
                row,
                text=key,
                width=3,
                bg="#1A1D24",
                fg=color,
                font=("DejaVu Sans Mono", 15, "bold"),
            ).pack(side="left", padx=(8, 6))
            tk.Label(
                row,
                text=label,
                bg="#1A1D24",
                fg="#D7DAE0",
                font=("DejaVu Sans", 13),
            ).pack(side="left")

        tk.Label(
            content,
            text="ESC  닫기",
            bg="#111318",
            fg="#555B66",
            font=("DejaVu Sans Mono", 10),
        ).pack(anchor="e")

        self.root.bind_all("<KeyPress>", self.on_key)
        self.root.after(80, self.focus)

    def focus(self) -> None:
        x = (self.root.winfo_screenwidth() - self.window_width) // 2
        y = (self.root.winfo_screenheight() - self.window_height) // 2
        self.root.geometry(
            f"{self.window_width}x{self.window_height}+{x}+{y}"
        )
        self.root.lift()
        self.root.focus_force()

    def on_key(self, event: tk.Event) -> None:
        if event.keysym == "Escape":
            self.cancel()
            return

        # 영문 대소문자는 물론 한글 입력기에서 만들어진 호환 자모도 받는다.
        pressed = (event.char or "").lower()
        action = KEY_ACTIONS.get(pressed)
        if action is None:
            # 한글 입력기 상태에서도 물리 키 G/M/P/C/L은 동일하게 동작한다.
            action = KEY_ACTIONS.get((event.keysym or "").lower())
        if action is not None:
            print(action, flush=True)
            self.root.destroy()

    def cancel(self) -> None:
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    Selector().run()
