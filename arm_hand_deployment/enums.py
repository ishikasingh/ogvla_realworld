import enum


class ArmHandType(enum.Enum):
    FRANKA = "franka"  # franka arm + franka gripper
    FRANKA_ALLEGRO = "franka_allegro"  # franka arm + allegro hand
