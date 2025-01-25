from abc import ABC, abstractmethod
from typing import Tuple
from typing_extensions import Self
from enum import Enum
import math

from geometry_msgs.msg import Pose
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
    MOVEMENT = "movement"
    OBJECTCOLOR = "objectColor"
    OBJECTDISTANCE = "objectDistance"
    OBJECTNAME = "objectName"
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


class MovementLink(str, Enum):
    BASE_LINK = "base_link"
    END_EFFECTOR_LINK = "end_effector_link"


class Condition:
    def __init__(self, comparator: Comparator, value: float | str):
        self.comparator: Comparator = comparator
        self.value: float | str | bool = value


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
    def parse_xml_parameters(self, element: etree._Element) -> None:
        pass


# BEGIN CUSTOM TASK OBJECT DEFINITIONS


class GoToPositionLeaf(ActionLeaf):
    def __init__(
        self,
        name: str,
        action_type: ActionType,
        depth: int,
        namespace: str,
    ):
        super().__init__(name, action_type, depth, namespace)

        # default movement
        self.movement_link: str = MovementLink.BASE_LINK

        self.pose: Pose = Pose()

    def set_pose(self, pose: Pose) -> None:
        self.pose = pose

    def parse_xml_parameters(self, element: etree._Element) -> None:
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

        self.movement_link = MovementLink(
            element.find(self.namespace + ParameterTypes.MOVEMENT).text
        )

        self._generate_pose()

    def _generate_pose(self) -> None:
        self.pose.position.x = self.x
        self.pose.position.y = self.y
        self.pose.position.z = self.z

        x, y, z, w = self.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.pose.orientation.x = x
        self.pose.orientation.y = y
        self.pose.orientation.z = z
        self.pose.orientation.w = w

    @staticmethod
    def quaternion_from_euler(
        roll: float, pitch: float, yaw
    ) -> Tuple[float, float, float, float]:
        # This function is a stripped down version of the code in
        # https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
        # Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
        # the way that ROS prefers it.
        roll /= 2.0
        pitch /= 2.0
        yaw /= 2.0
        ci = math.cos(roll)
        si = math.sin(roll)
        cj = math.cos(pitch)
        sj = math.sin(pitch)
        ck = math.cos(yaw)
        sk = math.sin(yaw)
        cc = ci * ck
        cs = ci * sk
        sc = si * ck
        ss = si * sk

        # returns x, y, z, w
        return (
            cj * sc - sj * cs,
            cj * ss + sj * cc,
            cj * cs - sj * sc,
            cj * cc + sj * ss,
        )

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
    ):
        super().__init__(name, action_type, depth, namespace)

    def parse_xml_parameters(self, element: etree._Element) -> None:
        # schema will validate that these exists. No need to verify
        object_name: str = element.find(self.namespace + ParameterTypes.OBJECTNAME).text

        object_distance: float = float(
            element.find(self.namespace + ParameterTypes.OBJECTDISTANCE).text
        )

        color_elements: list[etree._Element] = element.findall(
            self.namespace + ParameterTypes.OBJECTCOLOR
        )
        colors: list[str] = []
        for c in color_elements:
            colors.append(c.text)

        self.object_name: str = object_name
        self.object_distance: float = object_distance
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
    ):
        super().__init__(name, action_type, depth, namespace)

    def parse_xml_parameters(self, element: etree._Element) -> None:
        # schema will validate that these exists. No need to verify
        resolution: str = element.find(self.namespace + ParameterTypes.RESOLUTION).text

        self.resolution: str = resolution

    def __repr__(self) -> str:
        out: str = (
            f"name: {self.name}, actionType: {self.action_type}, resolution: {self.resolution}"
        )

        return out


# constructor dictionary so other modules don't need to import and can remain code agnostic
TASK_CONSTRUCTORS: dict = {
    "goToPosition": GoToPositionLeaf,
    "identifyObject": IdentifyObjectLeaf,
    "nextBestView": NextBestViewLeaf,
}
