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
    SEQUENCE = "Sequence"
    TASKID = "TaskID"
    VARIABLENAME = "VariableName"
    VARIABLEVALUE = "VariableValue"


class ActionType(str, Enum):
    GOTOLOCATION = "goToLocation"
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


class Conditional(str, Enum):
    EQ = "eq"
    GT = "gt"
    GTE = "gte"
    LT = "lt"
    LTE = "lte"
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
        depth: int,
        namespace: str,
    ):
        self.name: str = name
        self.action_type: ActionType = action_type
        self.depth: int = depth
        # this always goes (true, false) for condition in selecting child
        self.children: list[TaskLeaf, TaskLeaf] = [None, None]
        self.has_child: bool = False
        # XML namespace
        self.namespace: str = namespace

    def set_child(self, child: Self) -> None:
        if self.children[0] is None:
            self.children[0] = child
        else:
            self.children[1] = child
        self.has_child = True

    def set_conditional(self, cond: Conditional, value: float) -> None:
        self.conditional: Conditional = cond
        self.compare_value: float = value


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
            f"name: {self.name} \
            actionType: {self.action_type} \
            x: {self.x}"
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
            f"name: {self.name} \
            actionType: {self.action_type} \
            objectName: {self.object_name} \
            objectColors: "
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
            f"name: {self.name} \
            actionType: {self.action_type} \
            resolution: {self.resolution}"
        )

        return out


TASK_CONSTRUCTORS: dict = {
    "goToPosition": GoToPositionLeaf,
    "identifyObject": IdentifyObjectLeaf,
    "nextBestView": NextBestViewLeaf,
}
