from enum import Enum


class ExternalAxisMountMode(str, Enum):
    POSITIONED = "positioned"
    SYNCHRONIZED = "synchronized"
