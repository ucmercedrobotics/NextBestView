from typing import Tuple
import logging

from lxml import etree

from .behavior_tree import BehaviorTree, ElementTags


class MissionDecoder:
    def __init__(self, schema_path: str, logger: logging.Logger):
        self.logger: logging.Logger = logger

        # schema related attributes
        self.namespace: str = None
        self.schema_path: str = schema_path

        self.behavior_tree: BehaviorTree = None

    def validate_output(self, xml_file: str) -> Tuple[bool, str]:
        try:
            # Parse the XML file
            with open(xml_file, "rb") as xml_file:
                xml_doc = etree.parse(xml_file)

            # Validate the XML file against the XSD schema
            schema: etree.XMLSchema = etree.XMLSchema(file=self.schema_path)
            schema.assertValid(xml_doc)
            return True, "XML is valid."

        except etree.XMLSchemaError as e:
            return False, "XML is invalid: " + str(e)
        except Exception as e:
            return False, "An error occurred: " + str(e)

    def decode_xml_mp(self, xml_path: str) -> BehaviorTree | None:
        # parse the xml file
        mp: etree._ElementTree = etree.parse(xml_path)
        # get the root element <TaskTemplate>
        root: etree._Element = mp.getroot()
        # getting default ns with None (xmlns)
        self.namespace = "{" + root.nsmap[None] + "}"
        # find <ActionSequence>
        action_sequence: etree._Element = root.find(
            self.namespace + ElementTags.ACTIONSEQUENCE
        ).find(self.namespace + ElementTags.SEQUENCE)

        self.behavior_tree = BehaviorTree(self.namespace, self.logger)
        self.behavior_tree.init_behavior_tree(root, action_sequence)

        return self.behavior_tree
