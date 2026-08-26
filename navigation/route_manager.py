import heapq
import math
import random

from config import (
    NAVIGATION_APPROACH_DISTANCE_METERS,
    NAVIGATION_AUTO_NEW_ROUTE,
    NAVIGATION_DESTINATION_REACHED_METERS,
    NAVIGATION_INTERSECTION_LOOKAHEAD_METERS,
    NAVIGATION_MIN_DESTINATION_DISTANCE_METERS,
    NAVIGATION_PREFER_JUNCTION_ROUTES,
    NAVIGATION_RANDOM_SEED,
    NAVIGATION_SAMPLING_RESOLUTION_METERS
)


class RouteManager:
    AI = "AI"
    APPROACH = "APPROACH"
    INTERSECTION = "INTERSECTION"

    MANEUVERS = {
        "LEFT",
        "RIGHT",
        "STRAIGHT"
    }

    def __init__(
        self,
        world_map,
        vehicle,
        planner=None,
        random_seed=NAVIGATION_RANDOM_SEED
    ):
        self.world_map = world_map
        self.vehicle = vehicle
        self.planner = planner or self._create_planner()
        self.random = random.Random(random_seed)

        self.route = []
        self.route_index = 0
        self.destination = None
        self.route_number = 0

        self.plan_new_route()

    def plan_new_route(self):
        origin = self.vehicle.get_location()
        spawn_points = self.world_map.get_spawn_points()

        candidates = [
            spawn_point
            for spawn_point in spawn_points
            if self._distance(
                origin,
                spawn_point.location
            ) >= NAVIGATION_MIN_DESTINATION_DISTANCE_METERS
        ]

        if not candidates:
            candidates = list(spawn_points)

        self.random.shuffle(candidates)
        fallback_route = None
        fallback_destination = None

        for destination in candidates:
            try:
                route = self.planner.trace_route(
                    origin,
                    destination.location
                )
            except Exception as error:
                print(
                    "Skipping unreachable navigation destination:",
                    repr(error)
                )
                continue

            if not route:
                continue

            if fallback_route is None:
                fallback_route = list(route)
                fallback_destination = destination.location

            contains_junction = any(
                getattr(waypoint, "is_junction", False)
                for waypoint, _ in route
            )

            if (
                NAVIGATION_PREFER_JUNCTION_ROUTES
                and not contains_junction
            ):
                continue

            self._set_route(route, destination.location)
            return

        if fallback_route is not None:
            self._set_route(
                fallback_route,
                fallback_destination
            )
            return

        raise RuntimeError(
            "Could not create a route to any destination"
        )

    def _set_route(self, route, destination):
        self.route = list(route)
        self.route_index = 0
        self.destination = destination
        self.route_number += 1

        print(
            f"Navigation route {self.route_number}: "
            f"{len(self.route)} waypoints"
        )

    def update(self):
        if not self.route:
            self.plan_new_route()

        vehicle_location = self.vehicle.get_location()
        self._advance_route_index(vehicle_location)

        destination_distance = self._distance(
            vehicle_location,
            self.destination
        )

        if (
            destination_distance
            <= NAVIGATION_DESTINATION_REACHED_METERS
            and NAVIGATION_AUTO_NEW_ROUTE
        ):
            print("Navigation destination reached")
            self.plan_new_route()
            vehicle_location = self.vehicle.get_location()
            destination_distance = self._distance(
                vehicle_location,
                self.destination
            )

        maneuver, maneuver_distance = (
            self._find_next_maneuver()
        )
        junction_distance = self._find_junction_distance()

        current_waypoint = self.route[
            self.route_index
        ][0]

        if getattr(current_waypoint, "is_junction", False):
            mode = self.INTERSECTION
        elif (
            maneuver_distance is not None
            and maneuver_distance
            <= NAVIGATION_APPROACH_DISTANCE_METERS
        ) or (
            junction_distance is not None
            and junction_distance
            <= NAVIGATION_APPROACH_DISTANCE_METERS
        ):
            mode = self.APPROACH
        else:
            mode = self.AI

        return {
            "mode": mode,
            "maneuver": maneuver,
            "maneuver_distance_m": maneuver_distance,
            "junction_distance_m": junction_distance,
            "destination_distance_m": destination_distance,
            "target_waypoint": self._lookahead_waypoint(),
            "route_index": self.route_index,
            "route_length": len(self.route),
            "route_number": self.route_number
        }

    def _create_planner(self):
        try:
            from agents.navigation.global_route_planner import (
                GlobalRoutePlanner
            )
        except ImportError:
            print(
                "CARLA GlobalRoutePlanner was not found; "
                "using the built-in waypoint A* planner."
            )

            return WaypointGraphPlanner(
                self.world_map,
                NAVIGATION_SAMPLING_RESOLUTION_METERS
            )

        planner = GlobalRoutePlanner(
            self.world_map,
            NAVIGATION_SAMPLING_RESOLUTION_METERS
        )

        # Older CARLA versions require an explicit setup call.
        setup = getattr(planner, "setup", None)

        if callable(setup):
            setup()

        return planner

    def _advance_route_index(self, vehicle_location):
        search_end = min(
            len(self.route),
            self.route_index + 60
        )

        best_index = self.route_index
        best_distance = float("inf")

        for index in range(self.route_index, search_end):
            waypoint = self.route[index][0]
            distance = self._distance(
                vehicle_location,
                waypoint.transform.location
            )

            if distance < best_distance:
                best_index = index
                best_distance = distance

        self.route_index = best_index

    def _find_next_maneuver(self):
        distance = 0.0
        previous_location = self.route[
            self.route_index
        ][0].transform.location

        for index in range(
            self.route_index,
            len(self.route)
        ):
            waypoint, road_option = self.route[index]
            location = waypoint.transform.location

            if index > self.route_index:
                distance += self._distance(
                    previous_location,
                    location
                )

            option_name = self._option_name(road_option)

            if option_name in self.MANEUVERS:
                return option_name, distance

            previous_location = location

            if distance > 100.0:
                break

        return "LANEFOLLOW", None

    def _find_junction_distance(self):
        distance = 0.0
        previous_location = self.route[
            self.route_index
        ][0].transform.location

        for index in range(
            self.route_index,
            len(self.route)
        ):
            waypoint = self.route[index][0]
            location = waypoint.transform.location

            if index > self.route_index:
                distance += self._distance(
                    previous_location,
                    location
                )

            if getattr(waypoint, "is_junction", False):
                return distance

            previous_location = location

            if distance > NAVIGATION_APPROACH_DISTANCE_METERS + 10.0:
                break

        return None

    def _lookahead_waypoint(self):
        distance = 0.0
        previous_waypoint = self.route[
            self.route_index
        ][0]

        for index in range(
            self.route_index + 1,
            len(self.route)
        ):
            waypoint = self.route[index][0]
            distance += self._distance(
                previous_waypoint.transform.location,
                waypoint.transform.location
            )

            if distance >= (
                NAVIGATION_INTERSECTION_LOOKAHEAD_METERS
            ):
                return waypoint

            previous_waypoint = waypoint

        return self.route[-1][0]

    @staticmethod
    def _option_name(road_option):
        option_name = getattr(road_option, "name", None)

        if option_name is None:
            option_name = str(road_option).split(".")[-1]

        return option_name.upper()

    @staticmethod
    def _distance(first_location, second_location):
        return math.sqrt(
            (first_location.x - second_location.x) ** 2
            + (first_location.y - second_location.y) ** 2
            + (first_location.z - second_location.z) ** 2
        )


class WaypointGraphPlanner:
    def __init__(self, world_map, sampling_resolution):
        self.world_map = world_map
        self.sampling_resolution = sampling_resolution

    def trace_route(self, origin, destination):
        start = self.world_map.get_waypoint(
            origin,
            project_to_road=True
        )
        goal = self.world_map.get_waypoint(
            destination,
            project_to_road=True
        )

        if start is None or goal is None:
            return []

        start_key = self._waypoint_key(start)
        frontier = [(0.0, 0, start_key)]
        counter = 0

        came_from = {start_key: None}
        cost_so_far = {start_key: 0.0}
        waypoints = {start_key: start}
        goal_key = None

        while frontier and len(came_from) < 30000:
            _, _, current_key = heapq.heappop(frontier)
            current = waypoints[current_key]

            if self._distance(
                current.transform.location,
                goal.transform.location
            ) <= self.sampling_resolution * 1.5:
                goal_key = current_key
                break

            for next_waypoint in current.next(
                self.sampling_resolution
            ):
                next_key = self._waypoint_key(next_waypoint)
                step_cost = self._distance(
                    current.transform.location,
                    next_waypoint.transform.location
                )
                new_cost = cost_so_far[current_key] + step_cost

                if (
                    next_key in cost_so_far
                    and new_cost >= cost_so_far[next_key]
                ):
                    continue

                cost_so_far[next_key] = new_cost
                came_from[next_key] = current_key
                waypoints[next_key] = next_waypoint

                heuristic = self._distance(
                    next_waypoint.transform.location,
                    goal.transform.location
                )

                counter += 1
                heapq.heappush(
                    frontier,
                    (
                        new_cost + heuristic,
                        counter,
                        next_key
                    )
                )

        if goal_key is None:
            return []

        path = []
        current_key = goal_key

        while current_key is not None:
            path.append(waypoints[current_key])
            current_key = came_from[current_key]

        path.reverse()

        if self._waypoint_key(path[-1]) != self._waypoint_key(goal):
            path.append(goal)

        return [
            (
                waypoint,
                self._road_option(path, index)
            )
            for index, waypoint in enumerate(path)
        ]

    def _road_option(self, path, index):
        if index >= len(path) - 1:
            return "LANEFOLLOW"

        current = path[index]
        following = path[index + 1]

        if not (
            getattr(current, "is_junction", False)
            or getattr(following, "is_junction", False)
        ):
            return "LANEFOLLOW"

        entry_index = index

        while (
            entry_index > 0
            and getattr(
                path[entry_index],
                "is_junction",
                False
            )
        ):
            entry_index -= 1

        exit_index = index + 1

        while (
            exit_index < len(path) - 1
            and getattr(
                path[exit_index],
                "is_junction",
                False
            )
        ):
            exit_index += 1

        yaw_difference = self._normalize_angle(
            path[exit_index].transform.rotation.yaw
            - path[entry_index].transform.rotation.yaw
        )

        if abs(yaw_difference) < 35.0:
            return "STRAIGHT"

        return "RIGHT" if yaw_difference > 0 else "LEFT"

    def _waypoint_key(self, waypoint):
        waypoint_id = getattr(waypoint, "id", None)

        if waypoint_id is not None:
            return waypoint_id

        location = waypoint.transform.location

        return (
            round(location.x, 1),
            round(location.y, 1),
            round(location.z, 1)
        )

    @staticmethod
    def _normalize_angle(angle_degrees):
        return (
            (angle_degrees + 180.0) % 360.0
        ) - 180.0

    @staticmethod
    def _distance(first_location, second_location):
        return math.sqrt(
            (first_location.x - second_location.x) ** 2
            + (first_location.y - second_location.y) ** 2
            + (first_location.z - second_location.z) ** 2
        )
