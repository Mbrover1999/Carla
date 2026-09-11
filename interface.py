import queue
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
import uuid
from pathlib import Path
from tkinter import messagebox, ttk

from scenario_catalog import (
    DEFAULT_DURATION_MINUTES,
    DEFAULT_TRAFFIC_VEHICLES,
    MAX_DURATION_MINUTES,
    MIN_DURATION_MINUTES,
    SCENARIOS,
    TRAFFIC_PRESETS,
    SimulationSettings,
    get_traffic_preset_by_count,
    get_traffic_preset_by_display_name
)


PROJECT_ROOT = Path(__file__).resolve().parent

BACKGROUND = "#0B1220"
PANEL = "#111C2E"
PANEL_HOVER = "#182842"
ACCENT = "#2F80ED"
ACCENT_HOVER = "#4B94F2"
TEXT = "#F7F9FC"
MUTED = "#A9B5C7"
SUCCESS = "#39B980"
WARNING = "#E5A83B"


class CarlaInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Autonomous Driving Safety System")
        self.root.geometry("1280x820")
        self.root.minsize(1050, 700)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.selected_scenario = None
        self.simulation_process = None
        self.stop_request_path = None
        self.console_queue = queue.Queue()
        self.simulation_result = None
        self.stop_was_requested = False
        self.console_widget = None
        self.duration_value = tk.StringVar(
            value=str(DEFAULT_DURATION_MINUTES)
        )
        self.traffic_value = tk.StringVar(
            value=get_traffic_preset_by_count(
                DEFAULT_TRAFFIC_VEHICLES
            ).display_name
        )

        self._configure_styles()
        self.show_welcome()

    def _configure_styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Primary.TButton",
            background=ACCENT,
            foreground=TEXT,
            borderwidth=0,
            focusthickness=0,
            focuscolor=ACCENT,
            font=("Segoe UI", 11, "bold"),
            padding=(24, 13)
        )
        style.map(
            "Primary.TButton",
            background=[
                ("active", ACCENT_HOVER),
                ("disabled", "#33435A")
            ],
            foreground=[("disabled", "#7D899A")]
        )
        style.configure(
            "Secondary.TButton",
            background=PANEL,
            foreground=TEXT,
            borderwidth=0,
            focusthickness=0,
            focuscolor=PANEL,
            font=("Segoe UI", 11, "bold"),
            padding=(24, 13)
        )
        style.map(
            "Secondary.TButton",
            background=[
                ("active", PANEL_HOVER),
                ("disabled", "#172235")
            ],
            foreground=[("disabled", "#69768A")]
        )
        style.configure(
            "App.TSpinbox",
            fieldbackground=PANEL,
            background=PANEL,
            foreground=TEXT,
            arrowcolor=TEXT,
            bordercolor="#31415A",
            lightcolor="#31415A",
            darkcolor="#31415A",
            padding=8
        )
        style.configure(
            "App.TCombobox",
            fieldbackground=PANEL,
            background=PANEL,
            foreground=TEXT,
            arrowcolor=TEXT,
            bordercolor="#31415A",
            lightcolor="#31415A",
            darkcolor="#31415A",
            padding=8
        )
        style.map(
            "App.TCombobox",
            fieldbackground=[("readonly", PANEL)],
            foreground=[("readonly", TEXT)],
            selectbackground=[("readonly", PANEL)],
            selectforeground=[("readonly", TEXT)]
        )

    def _clear(self):
        for child in self.root.winfo_children():
            child.destroy()

    def _page(self):
        self._clear()
        page = tk.Frame(self.root, bg=BACKGROUND)
        page.pack(fill="both", expand=True, padx=58, pady=42)
        return page

    @staticmethod
    def _label(parent, text, size=12, color=TEXT, bold=False, **kwargs):
        return tk.Label(
            parent,
            text=text,
            bg=parent.cget("bg"),
            fg=color,
            font=("Segoe UI", size, "bold" if bold else "normal"),
            **kwargs
        )

    @staticmethod
    def _button(parent, text, command, primary=False, **kwargs):
        return ttk.Button(
            parent,
            text=text,
            command=command,
            cursor="hand2",
            style=(
                "Primary.TButton"
                if primary
                else "Secondary.TButton"
            ),
            **kwargs
        )

    def _header(self, page, title, subtitle=None, back_command=None):
        header = tk.Frame(page, bg=BACKGROUND)
        header.pack(fill="x", pady=(0, 30))

        if back_command is not None:
            self._button(
                header,
                "Back",
                back_command
            ).pack(side="left", padx=(0, 24))

        text_area = tk.Frame(header, bg=BACKGROUND)
        text_area.pack(side="left", fill="x", expand=True)
        self._label(
            text_area,
            title,
            size=24,
            bold=True,
            anchor="w"
        ).pack(fill="x")

        if subtitle:
            self._label(
                text_area,
                subtitle,
                size=11,
                color=MUTED,
                anchor="w"
            ).pack(fill="x", pady=(6, 0))

    def show_welcome(self):
        page = self._page()
        content = tk.Frame(page, bg=BACKGROUND)
        content.place(relx=0.5, rely=0.46, anchor="center")

        self._label(
            content,
            "AUTONOMOUS DRIVING",
            size=12,
            color=ACCENT,
            bold=True
        ).pack(pady=(0, 16))
        self._label(
            content,
            "Welcome",
            size=36,
            bold=True
        ).pack()
        self._label(
            content,
            "Explore autonomous driving and its safety systems in CARLA.",
            size=13,
            color=MUTED
        ).pack(pady=(12, 34))

        actions = tk.Frame(content, bg=BACKGROUND)
        actions.pack()
        self._button(
            actions,
            "START",
            self.show_scenarios,
            primary=True,
            width=16
        ).pack(side="left", padx=8)
        self._button(
            actions,
            "ABOUT",
            self.show_about,
            width=16
        ).pack(side="left", padx=8)

    def show_about(self):
        page = self._page()
        self._header(page, "About", back_command=self.show_welcome)

        card = tk.Frame(page, bg=PANEL, padx=34, pady=30)
        card.pack(fill="x")
        self._label(
            card,
            "Autonomous Driving Safety System",
            size=19,
            bold=True,
            anchor="w"
        ).pack(fill="x")
        self._label(
            card,
            (
                "This project combines an AI steering model with CARLA "
                "navigation, traffic awareness and independent safety "
                "layers. The system can detect obstacles, traffic lights, "
                "cross traffic, lane departures and controller inactivity."
            ),
            size=12,
            color=MUTED,
            justify="left",
            anchor="w",
            wraplength=820
        ).pack(fill="x", pady=(18, 0))

    def show_scenarios(self):
        page = self._page()
        self._header(
            page,
            "Choose a scenario",
            "Select a controlled demonstration or start with Free Drive.",
            self.show_welcome
        )

        grid = tk.Frame(page, bg=BACKGROUND)
        grid.pack(fill="both", expand=True)
        grid.grid_columnconfigure(0, weight=1, uniform="scenario")
        grid.grid_columnconfigure(1, weight=1, uniform="scenario")

        for index, scenario in enumerate(SCENARIOS):
            self._scenario_card(
                grid,
                scenario,
                index // 2,
                index % 2
            )

    def _scenario_card(self, parent, scenario, row, column):
        card = tk.Frame(
            parent,
            bg=PANEL,
            padx=24,
            pady=13,
            highlightthickness=1,
            highlightbackground="#25344C"
        )
        card.grid(
            row=row,
            column=column,
            sticky="nsew",
            padx=(0 if column == 0 else 9, 9 if column == 0 else 0),
            pady=7
        )
        parent.grid_rowconfigure(row, weight=1, minsize=148)

        title_row = tk.Frame(card, bg=PANEL)
        title_row.pack(fill="x")
        self._label(
            title_row,
            scenario.title,
            size=15,
            bold=True,
            anchor="w"
        ).pack(side="left")

        status_text = "Ready" if scenario.implemented else "Coming soon"
        status_color = SUCCESS if scenario.implemented else WARNING
        self._label(
            title_row,
            status_text,
            size=10,
            color=status_color,
            bold=True
        ).pack(side="right")

        self._label(
            card,
            scenario.description,
            size=10,
            color=MUTED,
            justify="left",
            anchor="nw",
            wraplength=390
        ).pack(fill="x", pady=(8, 10))

        button = self._button(
            card,
            "SELECT" if scenario.implemented else "NOT AVAILABLE YET",
            lambda selected=scenario: self.show_settings(selected),
            primary=scenario.implemented,
            width=20
        )
        button.pack(anchor="e", pady=(2, 0))

        if not scenario.implemented:
            button.configure(state="disabled", cursor="arrow")

    def show_settings(self, scenario):
        self.selected_scenario = scenario
        page = self._page()
        self._header(
            page,
            "Configure the run",
            scenario.title,
            self.show_scenarios
        )

        form = tk.Frame(page, bg=PANEL, padx=36, pady=32)
        form.pack(fill="x")
        form.grid_columnconfigure(1, weight=1)

        self._setting_row(
            form,
            row=0,
            title="Simulation duration",
            detail=(
                f"Choose {MIN_DURATION_MINUTES}-{MAX_DURATION_MINUTES} "
                "minutes."
            ),
            variable=self.duration_value,
            minimum=MIN_DURATION_MINUTES,
            maximum=MAX_DURATION_MINUTES,
            suffix="minutes"
        )
        self._traffic_setting_row(
            form,
            row=1
        )

        self._button(
            page,
            "REVIEW AND RUN",
            self.show_confirmation,
            primary=True
        ).pack(anchor="e", pady=(26, 0))

    def _setting_row(
        self,
        parent,
        row,
        title,
        detail,
        variable,
        minimum,
        maximum,
        suffix
    ):
        label_area = tk.Frame(parent, bg=PANEL)
        label_area.grid(
            row=row,
            column=0,
            sticky="w",
            pady=16,
            padx=(0, 40)
        )
        self._label(
            label_area,
            title,
            size=13,
            bold=True,
            anchor="w"
        ).pack(fill="x")
        self._label(
            label_area,
            detail,
            size=10,
            color=MUTED,
            anchor="w"
        ).pack(fill="x", pady=(5, 0))

        input_area = tk.Frame(parent, bg=PANEL)
        input_area.grid(row=row, column=1, sticky="e", pady=16)
        spinbox = ttk.Spinbox(
            input_area,
            from_=minimum,
            to=maximum,
            textvariable=variable,
            width=8,
            justify="center",
            style="App.TSpinbox",
            font=("Segoe UI", 12)
        )
        spinbox.pack(side="left")
        self._label(
            input_area,
            suffix,
            size=10,
            color=MUTED
        ).pack(side="left", padx=(12, 0))

    def _traffic_setting_row(self, parent, row):
        label_area = tk.Frame(parent, bg=PANEL)
        label_area.grid(
            row=row,
            column=0,
            sticky="w",
            pady=16,
            padx=(0, 40)
        )
        self._label(
            label_area,
            "Traffic density",
            size=13,
            bold=True,
            anchor="w"
        ).pack(fill="x")
        self._label(
            label_area,
            "Choose one of five predefined traffic levels.",
            size=10,
            color=MUTED,
            anchor="w"
        ).pack(fill="x", pady=(5, 0))

        combobox = ttk.Combobox(
            parent,
            textvariable=self.traffic_value,
            values=[
                preset.display_name
                for preset in TRAFFIC_PRESETS
            ],
            state="readonly",
            width=28,
            justify="left",
            style="App.TCombobox",
            font=("Segoe UI", 11)
        )
        combobox.grid(row=row, column=1, sticky="e", pady=16)

    def _read_settings(self):
        if self.selected_scenario is None:
            raise ValueError("Select a scenario first")

        try:
            duration = int(self.duration_value.get())
        except ValueError as error:
            raise ValueError("Duration must be a whole number") from error

        traffic_preset = get_traffic_preset_by_display_name(
            self.traffic_value.get()
        )

        return SimulationSettings(
            scenario_id=self.selected_scenario.scenario_id,
            duration_minutes=duration,
            traffic_vehicles=traffic_preset.vehicle_count
        ).validate()

    def show_confirmation(self):
        try:
            settings = self._read_settings()
        except ValueError as error:
            messagebox.showerror("Invalid settings", str(error))
            return

        page = self._page()
        self._header(
            page,
            "Ready to run",
            "Make sure the CARLA server is running before continuing.",
            lambda: self.show_settings(self.selected_scenario)
        )

        summary = tk.Frame(page, bg=PANEL, padx=36, pady=30)
        summary.pack(fill="x")
        values = (
            ("Scenario", self.selected_scenario.title),
            ("Duration", f"{settings.duration_minutes} minutes"),
            (
                "Traffic density",
                get_traffic_preset_by_count(
                    settings.traffic_vehicles
                ).display_name
            )
        )

        for row, (label, value) in enumerate(values):
            self._label(
                summary,
                label,
                size=11,
                color=MUTED,
                anchor="w"
            ).grid(row=row, column=0, sticky="w", pady=10)
            self._label(
                summary,
                value,
                size=12,
                bold=True,
                anchor="e"
            ).grid(row=row, column=1, sticky="e", pady=10, padx=(80, 0))

        summary.grid_columnconfigure(1, weight=1)
        self._button(
            page,
            "RUN SCENARIO",
            lambda: self.start_simulation(settings),
            primary=True
        ).pack(anchor="e", pady=(26, 0))

    def start_simulation(self, settings):
        self.console_queue = queue.Queue()
        self.simulation_result = None
        self.stop_was_requested = False
        self.stop_request_path = (
            Path(tempfile.gettempdir())
            / f"carla_stop_{uuid.uuid4().hex}.signal"
        )
        command = [
            sys.executable,
            "-u",
            str(PROJECT_ROOT / "main.py"),
            "--scenario",
            settings.scenario_id,
            "--duration-minutes",
            str(settings.duration_minutes),
            "--traffic-vehicles",
            str(settings.traffic_vehicles),
            "--stop-request-file",
            str(self.stop_request_path)
        ]

        try:
            self.simulation_process = subprocess.Popen(
                command,
                cwd=str(PROJECT_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1
            )
        except OSError as error:
            self._remove_stop_request()
            messagebox.showerror(
                "Could not start CARLA",
                str(error)
            )
            return

        threading.Thread(
            target=self._read_simulation_output,
            args=(self.simulation_process,),
            daemon=True
        ).start()
        self.show_running(settings)

    def _read_simulation_output(self, process):
        if process.stdout is None:
            return

        try:
            for line in process.stdout:
                self.console_queue.put(line.rstrip("\r\n"))
        finally:
            process.stdout.close()

    def show_running(self, settings):
        page = self._page()
        heading = tk.Frame(page, bg=BACKGROUND)
        heading.pack(fill="x", pady=(0, 16))
        heading_text = tk.Frame(heading, bg=BACKGROUND)
        heading_text.pack(side="left", fill="x", expand=True)
        self._label(
            heading_text,
            "SIMULATION RUNNING",
            size=11,
            color=SUCCESS,
            bold=True
        ).pack(anchor="w")
        self._label(
            heading_text,
            self.selected_scenario.title,
            size=24,
            bold=True,
            anchor="w"
        ).pack(fill="x", pady=(5, 0))
        self._label(
            heading_text,
            (
                f"{settings.duration_minutes} minutes  |  "
                f"{settings.traffic_vehicles} traffic vehicles"
            ),
            size=10,
            color=MUTED,
            anchor="w"
        ).pack(fill="x", pady=(5, 0))

        self.status_label = self._label(
            heading,
            "Starting CARLA client...",
            size=10,
            color=WARNING
        )
        self.status_label.pack(side="right", anchor="n", pady=(8, 0))

        event_panel = tk.Frame(
            page,
            bg=PANEL,
            padx=22,
            pady=14,
            highlightthickness=1,
            highlightbackground="#25344C"
        )
        event_panel.pack(fill="x", pady=(0, 16))
        self._label(
            event_panel,
            "CURRENT EVENT",
            size=9,
            color=MUTED,
            bold=True,
            anchor="w"
        ).pack(fill="x")
        self.current_event_label = self._label(
            event_panel,
            "Waiting for the first simulation frame...",
            size=14,
            color=WARNING,
            bold=True,
            anchor="w"
        )
        self.current_event_label.pack(fill="x", pady=(5, 0))
        self.indicator_label = self._label(
            event_panel,
            "Turn signal: Off",
            size=9,
            color=MUTED,
            anchor="w"
        )
        self.indicator_label.pack(fill="x", pady=(5, 0))

        console_header = tk.Frame(page, bg=BACKGROUND)
        console_header.pack(fill="x", pady=(0, 7))
        self._label(
            console_header,
            "SIMULATION CONSOLE",
            size=10,
            color=MUTED,
            bold=True,
            anchor="w"
        ).pack(side="left")

        console_panel = tk.Frame(
            page,
            bg="#07101D",
            highlightthickness=1,
            highlightbackground="#25344C"
        )
        console_panel.pack(fill="both", expand=True)
        scrollbar = ttk.Scrollbar(console_panel)
        scrollbar.pack(side="right", fill="y")
        self.console_widget = tk.Text(
            console_panel,
            bg="#07101D",
            fg="#CCD6E5",
            insertbackground=TEXT,
            selectbackground="#28466F",
            font=("Consolas", 9),
            relief="flat",
            borderwidth=0,
            wrap="word",
            padx=14,
            pady=12,
            state="disabled",
            yscrollcommand=scrollbar.set
        )
        self.console_widget.pack(fill="both", expand=True)
        scrollbar.configure(command=self.console_widget.yview)

        self.running_actions = tk.Frame(page, bg=BACKGROUND)
        self.running_actions.pack(fill="x", pady=(14, 0))
        self.stop_button = self._button(
            self.running_actions,
            "STOP SIMULATION",
            self.stop_simulation
        )
        self.stop_button.pack(side="right")

        self.root.after(500, self._poll_simulation)

    def _append_console_line(self, line):
        if self.console_widget is None:
            return

        self.console_widget.configure(state="normal")
        self.console_widget.insert("end", line + "\n")
        self.console_widget.see("end")
        self.console_widget.configure(state="disabled")

    def _drain_console_output(self):
        while True:
            try:
                line = self.console_queue.get_nowait()
            except queue.Empty:
                break

            self._append_console_line(line)

            if line.startswith("CURRENT_EVENT:"):
                raw_event = line.split(":", 1)[1].strip()
                self._show_current_event(raw_event)
            elif line.startswith("SCENARIO_EVENT:"):
                scenario_event = line.split(":", 1)[1].strip()
                self.current_event_label.configure(
                    text=scenario_event,
                    fg=ACCENT
                )
            elif line.startswith("TURN_SIGNAL:"):
                signal = line.split(":", 1)[1].strip()
                self.indicator_label.configure(
                    text=f"Turn signal: {signal.title()}"
                )
            elif line.startswith("SIMULATION_RESULT:"):
                self.simulation_result = line.split(":", 1)[1].strip()
            elif line.startswith("Connected to:"):
                self.status_label.configure(
                    text="Connected to CARLA",
                    fg=SUCCESS
                )

    def _show_current_event(self, raw_event):
        if raw_event in ("CLEAR", "DISABLED"):
            self.current_event_label.configure(
                text="No active safety event",
                fg=SUCCESS
            )
            return

        event_text = raw_event.replace("_", " ").title()
        urgent = any(
            word in raw_event
            for word in ("EMERGENCY", "BRAKING", "SAFE_STOP")
        )
        self.current_event_label.configure(
            text=event_text,
            fg="#F06A6A" if urgent else WARNING
        )

    def _poll_simulation(self):
        if self.simulation_process is None:
            return

        self._drain_console_output()
        exit_code = self.simulation_process.poll()

        if exit_code is None:
            self.root.after(500, self._poll_simulation)
            return

        self._drain_console_output()
        self.simulation_process = None
        self._remove_stop_request()
        self.stop_button.configure(state="disabled")

        if (
            self.simulation_result == "STOPPED"
            or self.stop_was_requested
        ):
            self.status_label.configure(
                text="Simulation stopped by user.",
                fg=WARNING
            )
        elif (
            self.simulation_result == "COMPLETED"
            or (
                self.simulation_result is None
                and exit_code == 0
            )
        ):
            self.status_label.configure(
                text="Simulation completed successfully.",
                fg=SUCCESS
            )
        else:
            self.status_label.configure(
                text=(
                    "Simulation stopped with an error. Review the console "
                    "below."
                ),
                fg="#F06A6A"
            )

        self._button(
            self.running_actions,
            "BACK TO SCENARIOS",
            self.show_scenarios,
            primary=True
        ).pack(side="left")

    def stop_simulation(self):
        if self.simulation_process is None:
            return

        self.stop_was_requested = True

        if self.stop_request_path is not None:
            self.stop_request_path.touch()

        self.status_label.configure(text="Stopping the simulation...")
        self.stop_button.configure(state="disabled")
        self.root.after(5000, self._force_stop_if_needed)

    def _force_stop_if_needed(self):
        if (
            self.simulation_process is not None
            and self.simulation_process.poll() is None
        ):
            self.simulation_process.terminate()

    def _remove_stop_request(self):
        if self.stop_request_path is None:
            return

        try:
            self.stop_request_path.unlink(missing_ok=True)
        except OSError:
            pass

        self.stop_request_path = None

    def close(self):
        if (
            self.simulation_process is not None
            and self.simulation_process.poll() is None
        ):
            should_close = messagebox.askyesno(
                "Simulation is running",
                "Stop the current simulation and close the interface?"
            )

            if not should_close:
                return

            self.stop_simulation()
            self.root.after(750, self.root.destroy)
            return

        self._remove_stop_request()
        self.root.destroy()


def main():
    root = tk.Tk()
    CarlaInterface(root)
    root.mainloop()


if __name__ == "__main__":
    main()
