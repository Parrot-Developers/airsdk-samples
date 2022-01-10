from fsup.missions.default.flying.stage import FLYING_STAGE as DEF_FLYING_STAGE
from fsup.missions.default.flying.manual import MANUAL_STATE


FLYING_STAGE = {
    "name": "flying",
    "class": DEF_FLYING_STAGE["class"],
    "initial": "manual",
    "children": [MANUAL_STATE],
}
