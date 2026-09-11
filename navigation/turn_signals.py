OFF = "OFF"
LEFT = "LEFT"
RIGHT = "RIGHT"
HAZARD = "HAZARD"


def select_turn_signal(navigation_information, hazards_active=False):
    if hazards_active:
        return HAZARD

    if navigation_information is None:
        return OFF

    if navigation_information.get("mode") not in (
        "APPROACH",
        "INTERSECTION"
    ):
        return OFF

    maneuver = navigation_information.get("maneuver")

    if maneuver == LEFT:
        return LEFT

    if maneuver == RIGHT:
        return RIGHT

    return OFF

