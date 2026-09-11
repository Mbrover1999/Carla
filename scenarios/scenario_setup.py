from scenarios.obstacle_ahead import ObstacleAheadScenario


def setup_scenario(scenario_id, world, ego_vehicle):
    if scenario_id == "free_drive":
        print("SCENARIO_EVENT: Free Drive initialized", flush=True)
        return []

    if scenario_id == "obstacle_ahead":
        scenario = ObstacleAheadScenario()
        return scenario.setup(world, ego_vehicle)

    raise ValueError(f"No setup is available for scenario: {scenario_id}")
