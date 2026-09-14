# Autonomous Driving with an Independent Safety Layer

This project was created as a final workshop project at the Open University.
It combines an AI steering model with CARLA navigation and an independent
safety layer for obstacle braking, cut-in prediction, lane keeping, traffic
lights, stop signs, cross traffic, blind spots and driver-inactivity response.

## Running the project

1. Start `CarlaUE4.exe` and wait until the map has fully loaded.
2. Make sure only one CARLA simulator is listening on port `2000`.
3. Activate the project's Python environment.
4. From the project directory, run:

   ```powershell
   python interface.py
   ```

5. Select **Free Drive** or one of the demonstration scenarios.

## Important OpenCV window limitation on Windows

During a simulation, keep the **CARLA RGB Camera** window open, visible and in
focus. OpenCV HighGUI requires an active HighGUI window for normal event
processing through `waitKey()`/`pollKey()`. In this project, `waitKey()` runs
on the same thread as the simulation control loop. On the tested Windows
system, moving the RGB window to another monitor, minimizing it or leaving it
inactive can therefore pause that loop for several seconds.

CARLA itself may continue simulating during that pause and retain the most
recent throttle and steering command. The visible symptoms are:

- the RGB camera image temporarily stops updating;
- the CARLA spectator camera stops following the ego vehicle;
- the ego vehicle continues using its last control command;
- the camera suddenly catches up when the Python loop resumes.

This is an OpenCV window/event-processing limitation observed with the Windows
runtime, rather than intended vehicle or safety-system behaviour. For reliable
demonstrations, use the primary monitor, keep the RGB window focused and avoid
moving or minimizing it while the simulation is running. If the view freezes,
stop the run from the interface and restart the scenario before continuing.

## Scope

The application is an educational CARLA prototype, not a production vehicle
control system. Results depend on simulator performance, graphics drivers,
hardware load and the supplied CARLA map and actors.
