import logging

from lxml import etree

from .tasking import TaskLeaf, ElementTags, Conditional, TASK_CONSTRUCTORS

TRUE_BRANCH_IDX: int = 0
FALSE_BRANCH_IDX: int = 1


class BehaviorTree:
    def __init__(self, namespace: str, logger: logging.Logger):
        """
        Base behavior tree object
        """
        self.namespace: str = namespace
        self.logger: logging.Logger = logger
        self.task_root: TaskLeaf = None

    def init_behavior_tree(
        self, root: etree._Element, sequence: etree._Element
    ) -> None:
        self.xml_root: etree._Element = root

        self._define_tree(sequence)

    def execute_tasks(self) -> bool:
        if not self.task_root.has_child:
            return True

        # start with the first side of the branch
        curr: TaskLeaf = self.task_root.children[TRUE_BRANCH_IDX]
        while curr is not None:
            # TODO: execute task through action/service
            # TODO: check outcome
            # TODO: if true, next true child
            # TODO: if false, check if there is a FALSE_BRANCH_IDX child
            # TODO: if not, you've failed
            pass

        return True

    def _define_tree(
        self, sequence: etree._Element, parent: TaskLeaf = None, depth: int = 0
    ):
        """
        Defines behavior tree recursively using Leaf classes defined above.
            root
             /\
        leaf1  leaf2
                 /\
            leaf3  leaf4
        """
        for t in sequence:
            if t.tag == self.namespace + ElementTags.TASKID:
                # names of tasks to identify with atomic descriptions
                task: TaskLeaf = self._create_leaf(parent, t.text, depth)
                self.logger.debug(task.__repr__())
            # this represents the second branch of behavior as first is covered in nested conditional above ^
            elif t.tag == self.namespace + ElementTags.CONDITIONALACTIONS:
                cond: etree._Element = t.find(self.namespace + ElementTags.CONDITONAL)
                if cond is not None:
                    comp: etree._Element = cond.find(
                        self.namespace + ElementTags.COMPARATOR
                    )
                    val: etree._Element = cond.find(
                        self.namespace + ElementTags.HARDVALUE
                    )
                    if comp is not None and val is not None:
                        self.logger.debug(
                            f"Adding condition -> {comp.text}: {val.text}"
                        )
                        # set conditionals to current task
                        parent.set_conditional(Conditional(comp.text), float(val.text))
                    # recursively iterate down this branch
                    self._define_tree(
                        t.find(self.namespace + ElementTags.SEQUENCE), parent, depth + 1
                    )
            elif isinstance(t, etree._Comment):
                continue
            else:
                self.logger.error(f"Found unknown element tag: {t.tag}")

            if parent is not None:
                parent.set_child(task)
            parent = task

    def _create_leaf(self, parent: TaskLeaf | None, name: str, depth: int) -> TaskLeaf:
        """
        Finds base object in XML that contains atomic definition.
        Used before adding task to list to define what task is
            1. Find order of task
            2. Define task
            3. Add task to sequence
        """

        # TODO: use self.root to explore AtomicTasks and define TaskLeaf
        # find <AtomicTasks> from root
        atomic_tasks: etree._Element = self.xml_root.find(
            self.namespace + ElementTags.ATOMICTASKS
        )
        # find the <AtomicTask> in the list that matches the current task
        task: etree._Element = BehaviorTree._find_child(
            atomic_tasks, self.namespace + ElementTags.TASKID, name
        )

        # <Action>
        #   <Action>
        #       <ActionType>we want this string</ActionType>
        #       ...
        action_type: etree._Element = task.find(
            self.namespace + ElementTags.ACTION
        ).find(self.namespace + ElementTags.ACTIONTYPE)

        # to make this generic, the action will always be after the action type
        action: etree._Element = action_type.getnext()

        # find from the dictionary which constructor to use
        t = TASK_CONSTRUCTORS[action_type.text](
            name, action_type.text, depth, self.namespace, action
        )
        # NOTE: this is the root task of the behavior tree
        if parent is None:
            self.task_root = t
        # TODO: do something with action_type and key off TaskLeaf.action_type

        return t

    # TODO: fully implement with tree like console output
    def __repr__(self):
        """
        Print statement to visualize current tree
        """
        curr = self.task_root.children[0]
        while curr is not None:
            self.logger.debug(
                f"name: {curr.name} \
                  type: {curr.action_type} \
                  depth: {curr.depth}"
            )
            curr = curr.children[0]

    @staticmethod
    def _find_child(parent: etree._Element, tag_name: str, text: str):
        """
        Helper function to find a child based on interior text:
        parent : current Element
        tag_name: tag name you're searching for text in
        text: text you're searching for

        <root>
            <tag_name>text</tag_name>
        </root>
        """
        for c in parent:
            if c is None:
                continue
            task_id = c.find(tag_name)
            if task_id is None:
                continue
            # if you have a match
            if text == task_id.text:
                return c
