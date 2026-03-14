from .wrapper import (
    HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
    GelloDataWrapper,
)


class IRLDataWrapper(HardwareDataWrapper):
    pass


class IRLImageDataWrapper(ImageDataWrapper):
    pass


class IRLPandaArmDataWrapper(PandaArmDataWrapper):
    pass


class IRLPandaGripperDataWrapper(PandaGripperDataWrapper):
    pass


class IRLRobotiqGripperDataWrapper(RobotiqGripperDataWrapper):
    pass


class IRLGelloDataWrapper(GelloDataWrapper):
    pass
