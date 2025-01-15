from abc import ABC, abstractmethod
from typing_extensions import Self
from enum import Enum

from lxml import etree


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
    RETURNSTATUS = "ReturnStatus"
    SEQUENCE = "Sequence"
    TASKID = "TaskID"
    VARIABLENAME = "VariableName"
    VARIABLEVALUE = "VariableValue"


class ActionType(str, Enum):
    GOTOPOSITION = "goToPosition"
    GRABOBJECT = "grabObject"
    IDENTIFYOBJECT = "identifyObject"
    NEXTBESTVIEW = "nextBestView"
    RELEASEOBJECT = "releaseObject"


class ParameterTypes(str, Enum):
    GRIPPERVELOCITY = "gripperVelocity"
    OBJECTNAME = "objectName"
    OBJECTCOLOR = "objectColor"
    PITCH = "pitch"
    RESOLUTION = "resolution"
    ROLL = "roll"
    VIEWDISTANCE = "viewDistance"
    X = "x"
    Y = "y"
    YAW = "yaw"
    Z = "z"


class Comparator(str, Enum):
    EQ = "eq"
    GT = "gt"
    GTE = "gte"
    LT = "lt"
    LTE = "lte"
    NEQ = "neq"


class TaskResult(str, Enum):
    TRUE = "true"
    FALSE = "false"


class Condition:
    def __init__(self, comparator: Comparator, value: float | str):
        self.comparator: Comparator = comparator
        self.value: float | str | bool = value


TRUE_BRANCH_IDX: int = 0
FALSE_BRANCH_IDX: int = 1


class TaskLeaf:
    """
    Base object for defining task:
    Task object for Kinova related tasks.
    """

    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int,
        namespace: str,
    ):
        self.name: str = name
        self.action_type: ActionType = action_type
        self.depth: int = depth

        self.next: TaskLeaf = None
        # this always goes (true, false) for condition in selecting child
        self.branches: list[TaskLeaf] = []
        self.conditionals: list[Condition] = []
        self.has_branches: bool = False
        self.has_conditionals: bool = False
        # XML namespace
        self.namespace: str = namespace

    def set_next(self, next: Self) -> None:
        self.next = next

    def add_branch(self, child: Self) -> None:
        self.branches.append(child)
        self.has_branches = True

    def add_conditional(self, condition: Condition) -> None:
        self.conditionals.append(condition)
        self.has_conditionals = True


class ActionLeaf(TaskLeaf, ABC):
    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int,
        namespace: str,
    ):
        super().__init__(name, action_type, depth, namespace)

    @abstractmethod
    def _parse_xml_parameters(self, element: etree._Element) -> None:
        pass


# BEGIN CUSTOM TASK OBJECT DEFINITIONS


class GoToPositionLeaf(ActionLeaf):
    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int,
        namespace: str,
        element: etree._Element,
    ):
        super().__init__(name, action_type, depth, namespace)
        self._parse_xml_parameters(element)

    def _parse_xml_parameters(self, element: etree._Element) -> None:
        # schema will validate that these exists. No need to verify
        x: float = float(element.find(self.namespace + ParameterTypes.X).text)

        y: float = float(element.find(self.namespace + ParameterTypes.Y).text)

        z: float = float(element.find(self.namespace + ParameterTypes.Z).text)

        roll: float = float(element.find(self.namespace + ParameterTypes.ROLL).text)

        pitch: float = float(element.find(self.namespace + ParameterTypes.PITCH).text)

        yaw: float = float(element.find(self.namespace + ParameterTypes.YAW).text)

        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.roll: float = roll
        self.pitch: float = pitch
        self.yaw: float = yaw

    def __repr__(self) -> str:
        out: str = (
            f"name: {self.name}, actionType: {self.action_type}, x: {self.x}, y: {self.y}, z: {self.z}"
        )

        return out


class IdentifyObjectLeaf(ActionLeaf):
    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int,
        namespace: str,
        element: etree._Element,
    ):
        super().__init__(name, action_type, depth, namespace)
        self._parse_xml_parameters(element)

    def _parse_xml_parameters(self, element: etree._Element) -> None:
        # schema will validate that these exists. No need to verify
        object_name: etree._Element = element.find(
            self.namespace + ParameterTypes.OBJECTNAME
        ).text

        color_elements: list[etree._Element] = element.findall(
            self.namespace + ParameterTypes.OBJECTCOLOR
        )
        colors: list[str] = []
        for c in color_elements:
            colors.append(c.text)

        self.object_name: str = object_name
        self.colors: list[str] = colors

    def __repr__(self) -> str:
        out: str = (
            f"name: {self.name}, actionType: {self.action_type}, objectName: {self.object_name}, objectColors: "
        )

        for c in self.colors:
            out += c + ","

        return out


class NextBestViewLeaf(ActionLeaf):
    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int,
        namespace: str,
        element: etree._Element,
    ):
        super().__init__(name, action_type, depth, namespace)
        self._parse_xml_parameters(element)

    def _parse_xml_parameters(self, element: etree._Element) -> None:
        # schema will validate that these exists. No need to verify
        resolution: str = element.find(self.namespace + ParameterTypes.RESOLUTION).text

        self.resolution: str = resolution

    def __repr__(self) -> str:
        out: str = (
            f"name: {self.name}, actionType: {self.action_type}, resolution: {self.resolution}"
        )

        return out


TASK_CONSTRUCTORS: dict = {
    "goToPosition": GoToPositionLeaf,
    "identifyObject": IdentifyObjectLeaf,
    "nextBestView": NextBestViewLeaf,
}
