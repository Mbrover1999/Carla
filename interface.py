import json
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
import uuid
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

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

BACKGROUND = "#070D18"
PANEL = "#101B2D"
PANEL_HOVER = "#172842"
PANEL_RAISED = "#152238"
ACCENT = "#3B82F6"
ACCENT_HOVER = "#60A5FA"
CYAN = "#22D3EE"
TEXT = "#F8FAFC"
MUTED = "#94A3B8"
SUCCESS = "#34D399"
WARNING = "#FBBF24"
DANGER = "#FB7185"


class CarlaInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Autonomous Driving with a Safety Layer")
        self.root.geometry("1600x980")
        self.root.minsize(1280, 820)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.selected_scenario = None
        self.simulation_process = None
        self.stop_request_path = None
        self.control_request_path = None
        self.control_buttons = []
        self.current_settings = None
        self.journey_result = None
        self.free_drive_log_path = None
        self.console_queue = queue.Queue()
        self.simulation_result = None
        self.stop_was_requested = False
        self.console_widget = None
        self.console_header = None
        self.result_panel = None
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
            padding=(28, 14)
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
        shell = tk.Frame(self.root, bg=BACKGROUND)
        shell.pack(fill="both", expand=True)
        tk.Frame(shell, bg=ACCENT, height=4).pack(fill="x")

        navigation = tk.Frame(shell, bg=BACKGROUND)
        navigation.pack(fill="x", padx=58, pady=(18, 0))
        self._label(
            navigation,
            "CARLA  /  OPEN UNIVERSITY WORKSHOP PROJECT",
            size=10,
            color=CYAN,
            bold=True
        ).pack(side="left")
        self._label(
            navigation,
            "AUTONOMOUS DRIVING PLATFORM  •  2026",
            size=9,
            color=MUTED
        ).pack(side="right")

        page = tk.Frame(shell, bg=BACKGROUND)
        page.pack(
            fill="both",
            expand=True,
            padx=58,
            pady=(20, 38)
        )
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
        content = tk.Frame(
            page,
            bg=PANEL,
            padx=76,
            pady=58,
            highlightthickness=1,
            highlightbackground="#243653"
        )
        content.place(
            relx=0.5,
            rely=0.47,
            anchor="center",
            relwidth=0.84
        )

        tk.Frame(content, bg=CYAN, width=72, height=4).pack(pady=(0, 24))

        self._label(
            content,
            "FINAL WORKSHOP PROJECT  /  CARLA 2026",
            size=11,
            color=CYAN,
            bold=True
        ).pack(pady=(0, 16))
        self._label(
            content,
            "Autonomous Driving\nwith an Independent Safety Layer",
            size=38,
            bold=True
        ).pack()
        self._label(
            content,
            "Created as a final project for a workshop at "
            "The Open University of Israel.",
            size=14,
            color=MUTED,
            wraplength=900,
            justify="center"
        ).pack(pady=(15, 28))

        capabilities = tk.Frame(content, bg=PANEL)
        capabilities.pack(pady=(0, 32))

        for index, (value, label) in enumerate((
            (f"{len(SCENARIOS):02d}", "SCENARIOS"),
            ("07", "SAFETY LAYERS"),
            ("LIVE", "MONITORING")
        )):
            chip = tk.Frame(
                capabilities,
                bg=PANEL_RAISED,
                padx=22,
                pady=11
            )
            chip.grid(row=0, column=index, padx=5)
            self._label(
                chip,
                value,
                size=14,
                color=SUCCESS,
                bold=True
            ).pack()
            self._label(
                chip,
                label,
                size=8,
                color=MUTED,
                bold=True
            ).pack(pady=(2, 0))

        actions = tk.Frame(content, bg=PANEL)
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
        self._header(
            page,
            "About the project",
            "A simple overview of the system, its safety features and the "
            "technologies used to build it.",
            self.show_welcome
        )

        card = tk.Frame(page, bg=PANEL, padx=34, pady=30)
        card.pack(fill="x")
        self._label(
            card,
            "Autonomous Driving with an Independent Safety Layer",
            size=21,
            bold=True,
            anchor="w"
        ).pack(fill="x")
        self._label(
            card,
            (
                "This system was created as a final project for a workshop "
                "at The Open University of Israel. It demonstrates a car "
                "that drives by itself in CARLA while independent safety "
                "systems continuously check the road and can correct or "
                "stop the vehicle when necessary."
            ),
            size=12,
            color=MUTED,
            justify="left",
            anchor="w",
            wraplength=1250
        ).pack(fill="x", pady=(18, 0))

        details = tk.Frame(page, bg=BACKGROUND)
        details.pack(fill="both", expand=True, pady=(18, 0))
        details.grid_columnconfigure(0, weight=1, uniform="about")
        details.grid_columnconfigure(1, weight=1, uniform="about")
        details.grid_rowconfigure(0, weight=1)

        left = tk.Frame(details, bg=BACKGROUND)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 9))
        right = tk.Frame(details, bg=BACKGROUND)
        right.grid(row=0, column=1, sticky="nsew", padx=(9, 0))

        self._about_card(
            left,
            "How it works",
            (
                "A forward RGB camera supplies images to a PyTorch CNN. "
                "The model predicts steering on normal road sections. "
                "CARLA map waypoints guide turns and intersections, and a "
                "separate controller follows 98% of the road speed limit. "
                "Before any command reaches the car, the safety layer is "
                "allowed to reduce throttle, change steering or apply the "
                "brakes."
            )
        ).pack(fill="x")

        self._about_card(
            left,
            "Safety features",
            (
                "• Speed-based obstacle warning and emergency braking\n"
                "• Predictive braking for vehicles cutting into the lane\n"
                "• Red-light detection and stopping near the stop line\n"
                "• Map-based stop-sign detection and complete stop\n"
                "• Lane-departure detection and lane-keeping correction\n"
                "• Cross-traffic prediction at intersections\n"
                "• Left and right blind-spot monitoring\n"
                "• Controller-inactivity warning and automatic Safe Stop\n"
                "• Hazard lights, repeated wake-up alarm and simulated "
                "emergency call"
            )
        ).pack(fill="x", pady=(18, 0))

        self._about_card(
            right,
            "Technologies",
            (
                "• CARLA Town10HD with a Tesla Model 3 ego vehicle\n"
                "• Python\n"
                "• PyTorch CNN with four convolution layers, trained for "
                "15 epochs on 320×180 images\n"
                "• OpenCV with an 800×600 RGB camera, 90° field of view "
                "and a nominal 20 FPS\n"
                "• NumPy, pandas and Pillow for data processing\n"
                "• Tkinter for the desktop interface\n"
                "• CARLA camera, obstacle, collision, lane and radar sensors\n"
                "• CSV logging and local WAV audio alerts"
            )
        ).pack(fill="x")

        self._about_card(
            right,
            "Demonstration and evaluation",
            (
                "The interface includes Free Drive and seven controlled "
                "safety demonstrations: a stationary obstacle, sudden lead-"
                "vehicle braking, a vehicle cut-in, cross traffic, lane "
                "departure, a red traffic light and driver inactivity. "
                "Free Drive produces a safety log and a score with a clear "
                "explanation of collisions, lane events, speed efficiency "
                "and journey completion."
            )
        ).pack(fill="x", pady=(18, 0))

    def _about_card(self, parent, title, text):
        card = tk.Frame(
            parent,
            bg=PANEL,
            padx=28,
            pady=22,
            highlightthickness=1,
            highlightbackground="#29405F"
        )
        self._label(
            card,
            title,
            size=15,
            color=CYAN,
            bold=True,
            anchor="w"
        ).pack(fill="x")
        self._label(
            card,
            text,
            size=10,
            color=MUTED,
            justify="left",
            anchor="nw",
            wraplength=570
        ).pack(fill="x", pady=(11, 0))
        return card

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
            highlightbackground="#29405F"
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
            f"{row * 2 + column + 1:02d}",
            size=10,
            color=CYAN,
            bold=True
        ).pack(side="left", padx=(0, 12))
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
            lambda selected=scenario: self.select_scenario(selected),
            primary=scenario.implemented,
            width=20
        )
        button.pack(anchor="e", pady=(2, 0))

        self._bind_card_hover(card)

        if not scenario.implemented:
            button.configure(state="disabled", cursor="arrow")

    def _bind_card_hover(self, card):
        def recolor(widget, color):
            try:
                if widget.cget("bg") in (PANEL, PANEL_HOVER):
                    widget.configure(bg=color)
            except (tk.TclError, TypeError):
                pass

            for child in widget.winfo_children():
                recolor(child, color)

        card.bind(
            "<Enter>",
            lambda _event: recolor(card, PANEL_HOVER)
        )
        card.bind(
            "<Leave>",
            lambda _event: recolor(card, PANEL)
        )

    def select_scenario(self, scenario):
        self.selected_scenario = scenario

        if scenario.scenario_id == "free_drive":
            self.show_settings(scenario)
            return

        self.show_confirmation(
            SimulationSettings(
                scenario_id=scenario.scenario_id,
                duration_minutes=scenario.demo_duration_minutes,
                traffic_vehicles=0
            ).validate()
        )

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

    def show_confirmation(self, settings=None):
        if settings is None:
            try:
                settings = self._read_settings()
            except ValueError as error:
                messagebox.showerror("Invalid settings", str(error))
                return

        is_free_drive = settings.scenario_id == "free_drive"

        page = self._page()
        self._header(
            page,
            "Ready to run",
            "Make sure the CARLA server is running before continuing.",
            (
                lambda: self.show_settings(self.selected_scenario)
                if is_free_drive
                else self.show_scenarios
            )
        )

        summary = tk.Frame(page, bg=PANEL, padx=36, pady=30)
        summary.pack(fill="x")
        if is_free_drive:
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
        else:
            values = (
                ("Scenario", self.selected_scenario.title),
                ("Mode", "Controlled demonstration"),
                ("Environment", "Empty road, automatic setup")
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
        self.current_settings = settings
        self.journey_result = None
        self.free_drive_log_path = None
        self.stop_request_path = (
            Path(tempfile.gettempdir())
            / f"carla_stop_{uuid.uuid4().hex}.signal"
        )
        self.control_request_path = (
            Path(tempfile.gettempdir())
            / f"carla_controls_{uuid.uuid4().hex}.commands"
        )
        self.control_request_path.touch()
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
            str(self.stop_request_path),
            "--control-command-file",
            str(self.control_request_path)
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
            self._remove_control_request()
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
        run_description = (
            (
                f"{settings.duration_minutes} minutes  |  "
                f"{settings.traffic_vehicles} traffic vehicles"
            )
            if settings.scenario_id == "free_drive"
            else "Controlled demonstration"
        )
        self._label(
            heading_text,
            run_description,
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

        self.result_panel = tk.Frame(
            page,
            bg=PANEL,
            padx=24,
            pady=18,
            highlightthickness=1,
            highlightbackground=ACCENT
        )

        self.console_header = tk.Frame(page, bg=BACKGROUND)
        self.console_header.pack(fill="x", pady=(0, 7))
        self._label(
            self.console_header,
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
        self.control_buttons = []

        if settings.scenario_id == "free_drive":
            controls = (
                ("INACTIVITY TEST", "TOGGLE_INACTIVITY"),
                ("LANE KEEPING", "TOGGLE_LANE_KEEPING"),
                ("NEW ROUTE", "NEW_ROUTE")
            )

            for label, command in controls:
                button = self._button(
                    self.running_actions,
                    label,
                    lambda value=command: self._send_control_command(value)
                )
                button.pack(side="left", padx=(0, 8))
                self.control_buttons.append(button)

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
            elif line.startswith("JOURNEY_RESULT:"):
                try:
                    self.journey_result = json.loads(
                        line.split(":", 1)[1].strip()
                    )
                except json.JSONDecodeError:
                    self._append_console_line(
                        "Could not read the journey result."
                    )
            elif line.startswith("FREE_DRIVE_LOG:"):
                self.free_drive_log_path = Path(
                    line.split(":", 1)[1].strip()
                )
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
        self._remove_control_request()
        self.stop_button.configure(state="disabled")

        for button in self.control_buttons:
            button.configure(state="disabled")

        if self.journey_result is not None:
            self._show_journey_result(self.journey_result)

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

        if (
            self.current_settings is not None
            and self.current_settings.scenario_id == "free_drive"
            and self.free_drive_log_path is not None
        ):
            self._button(
                self.running_actions,
                "SAVE LOG",
                self.save_free_drive_log,
                primary=True
            ).pack(side="left", padx=(10, 0))

    def _show_journey_result(self, result):
        score = result.get("score", 0)
        rating = result.get("rating", "Unknown")
        score_color = (
            SUCCESS
            if score >= 75
            else WARNING
            if score >= 60
            else DANGER
        )
        self.current_event_label.configure(
            text="Free Drive evaluation completed",
            fg=score_color
        )
        self.indicator_label.configure(
            text=(
                f"Distance {result.get('distance_km', 0):.2f} km  •  "
                f"Average {result.get('average_speed_kmh', 0):.1f} km/h  •  "
                f"Maximum {result.get('max_speed_kmh', 0):.1f} km/h"
            ),
            fg=MUTED
        )

        if self.result_panel is None or self.console_header is None:
            return

        for child in self.result_panel.winfo_children():
            child.destroy()

        self.result_panel.pack(
            fill="x",
            pady=(0, 14),
            before=self.console_header
        )
        score_area = tk.Frame(self.result_panel, bg=PANEL)
        score_area.pack(side="left", fill="y", padx=(0, 30))
        self._label(
            score_area,
            "JOURNEY SCORE",
            size=9,
            color=MUTED,
            bold=True
        ).pack(anchor="w")
        score_line = tk.Frame(score_area, bg=PANEL)
        score_line.pack(anchor="w", pady=(2, 0))
        self._label(
            score_line,
            str(score),
            size=38,
            color=score_color,
            bold=True
        ).pack(side="left")
        self._label(
            score_line,
            "/ 100",
            size=15,
            color=MUTED,
            bold=True
        ).pack(side="left", anchor="s", pady=(0, 8), padx=(5, 0))
        self._label(
            score_area,
            rating.upper(),
            size=11,
            color=score_color,
            bold=True
        ).pack(anchor="w", pady=(2, 0))

        details = tk.Frame(self.result_panel, bg=PANEL)
        details.pack(side="left", fill="both", expand=True)
        breakdown = result.get("score_breakdown", {})
        breakdown_row = tk.Frame(details, bg=PANEL)
        breakdown_row.pack(fill="x")

        components = (
            ("SAFETY", breakdown.get("safety", 0), 50),
            ("LANE DISCIPLINE", breakdown.get("lane_discipline", 0), 20),
            ("SPEED EFFICIENCY", breakdown.get("speed_efficiency", 0), 20),
            ("COMPLETION", breakdown.get("completion", 0), 10)
        )

        for column, (label, points, maximum) in enumerate(components):
            component = tk.Frame(
                breakdown_row,
                bg=PANEL_RAISED,
                padx=14,
                pady=9
            )
            component.grid(
                row=0,
                column=column,
                sticky="nsew",
                padx=(0 if column == 0 else 4, 0)
            )
            breakdown_row.grid_columnconfigure(column, weight=1)
            self._label(
                component,
                label,
                size=8,
                color=MUTED,
                bold=True
            ).pack(anchor="w")
            self._label(
                component,
                f"{points:g} / {maximum}",
                size=13,
                color=TEXT,
                bold=True
            ).pack(anchor="w", pady=(3, 0))
            progress_track = tk.Frame(
                component,
                bg="#26364F",
                height=3
            )
            progress_track.pack(fill="x", pady=(7, 0))
            progress_fill = tk.Frame(
                progress_track,
                bg=score_color,
                height=3
            )
            progress_fill.place(
                x=0,
                y=0,
                relheight=1.0,
                relwidth=max(0.0, min(float(points) / maximum, 1.0))
            )

        explanation = "  •  ".join(
            result.get("score_explanation", [])
        )
        self._label(
            details,
            explanation or "No detailed evaluation was returned.",
            size=9,
            color=MUTED,
            justify="left",
            anchor="w",
            wraplength=850
        ).pack(fill="x", pady=(11, 0))

    def save_free_drive_log(self):
        source = self.free_drive_log_path

        if source is None or not source.exists():
            messagebox.showerror(
                "Log unavailable",
                "The Free Drive log could not be found."
            )
            return

        destination = filedialog.asksaveasfilename(
            title="Save Free Drive log",
            defaultextension=".csv",
            initialfile=source.name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )

        if not destination:
            return

        try:
            shutil.copy2(source, destination)
        except OSError as error:
            messagebox.showerror("Could not save log", str(error))
            return

        messagebox.showinfo(
            "Log saved",
            f"The Free Drive log was saved to:\n{destination}"
        )

    def stop_simulation(self):
        if self.simulation_process is None:
            return

        self.stop_was_requested = True

        if self.stop_request_path is not None:
            self.stop_request_path.touch()

        self.status_label.configure(text="Stopping the simulation...")
        self.stop_button.configure(state="disabled")
        self.root.after(5000, self._force_stop_if_needed)

    def _send_control_command(self, command):
        if (
            self.simulation_process is None
            or self.control_request_path is None
        ):
            return

        try:
            with self.control_request_path.open(
                "a",
                encoding="utf-8"
            ) as stream:
                stream.write(command + "\n")
        except OSError as error:
            self._append_console_line(
                f"Could not send control command: {error}"
            )

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

    def _remove_control_request(self):
        if self.control_request_path is None:
            return

        try:
            self.control_request_path.unlink(missing_ok=True)
        except OSError:
            pass

        self.control_request_path = None

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
        self._remove_control_request()
        self.root.destroy()


def main():
    root = tk.Tk()
    CarlaInterface(root)
    root.mainloop()


if __name__ == "__main__":
    main()
