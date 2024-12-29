from typing_extensions import Self
from enum import Enum

"""
The enums below are based on string values found in the schema.xsd
"""


# This defines the tag types throughout the schema document
class ElementTags(str, Enum):
    ABSTRACTVALUE = "AbstractValue"
    ACTION = "Action"
    ACTIONSEQUENCE = "ActionSequence"
    ACTIONTYPE = "ActionType"
    ATOMICTASKS = "AtomicTasks"
    COMPARATOR = "Comparator"
    CONDITONAL = "Conditional"
    CONDITIONALACTIONS = "ConditionalActions"
    CONDITIONALEXPRESSION = "ConditionalExpression"
    HARDVALUE = "HardValue"
    PARAMETER = "Parameter"
    PARAMETERS = "Parameters"
    PRECONDITION = "Precondition"
    PRECONDITIONS = "Preconditions"
    SEQUENCE = "Sequence"
    TASKID = "TaskID"
    VARIABLENAME = "VariableName"
    VARIABLEVALUE = "VariableValue"


class ActionType(str, Enum):
    GRABOBJECT = "grabObject"
    RELEASEOBJECT = "releaseObject"
    IDENTIFYOBJECT = "identifyObject"
    NEXTBESTVIEW = "nextBestView"
    # TODO: could be that this is implied in prior action
    STOREPOINTCLOUD = "storePointCloud"
    # TODO: remove
    MOVETOLOCATION = "moveToLocation"
    TAKETHERMALPICTURE = "takeThermalPicture"
    TAKECO2READING = "takeCO2Reading"
    TAKEAMBIENTTEMPERATURE = "takeAmbientTemperature"


class ParameterTypes(str, Enum):
    GRIPPERVELOCITY = "gripperVelocity"
    OBJECTNAME = "objectName"
    OBJECTCOLOR = "objectColor"
    VIEWDISTANCE = "viewDistance"


class Conditional(str, Enum):
    LT = "lt"
    LTE = "lte"
    GT = "gt"
    GTE = "gte"
    EQ = "eq"
    NEQ = "neq"


class TaskLeaf:
    """
    Base object for defining task:
    Task object for Kinova related tasks.
    """

    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int = 0,
        conditional: Conditional = None,
    ):
        self.name: str = name
        self.action_type: ActionType = action_type
        self.depth: int = depth
        # this always goes (true, false) for condition in selecting child
        self.children: list[TaskLeaf, TaskLeaf] = [None, None]
        self.has_child: bool = False

    def set_child(self, child: Self) -> None:
        if self.children[0] is None:
            self.children[0] = child
        else:
            self.children[1] = child
        self.has_child = True

    def set_conditional(self, cond: Conditional, value: float) -> None:
        self.conditional: Conditional = cond
        self.compare_value: float = value


class ActionLeaf(TaskLeaf):
    def __init__(self, name: str, depth: int):
        super().__init__(name, depth)
