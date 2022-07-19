import typing
import xml.etree.ElementTree as Xet
from typing import List
import os
import shutil
from textwrap import dedent


class TagException(Exception):
    def __init__(self, c, n):
        self.failed_class = c
        self.failed_node = n
        self.message = f"Invalid tag in {type(self.failed_class).__name__} parse: {self.failed_node.tag}"
        super().__init__(self.message)


class ChildTagException(Exception):
    def __init__(self, c, n, ch):
        self.failed_class = c
        self.failed_node = n
        self.failed_child = ch
        self.message = f"Invalid child tag in {type(self.failed_class).__name__} -> " \
                       f"{type(self.failed_child).__name__} parse: {self.failed_child.tag}"
        super().__init__(self.message)


class NodeAttrException(Exception):
    def __init__(self, c, n, a):
        self.failed_class = c
        self.failed_node = n
        self.failed_attr = a
        self.message = f"Invalid attribute in {type(self.failed_class).__name__} parse: {self.failed_attr}"
        super().__init__(self.message)


class NodeHasChildrenException(Exception):
    def __init__(self, c, n):
        self.failed_class = c
        self.failed_node = n
        self.message = f"Node should not have children: {type(self.failed_class).__name__} -> " \
                       f"{str_node(self.failed_node)}"
        super().__init__(self.message)


class NodeHasNoChildrenException(Exception):
    def __init__(self, c, n):
        self.failed_class = c
        self.failed_node = n
        self.message = f"Node should have children: {type(self.failed_class).__name__} -> " \
                       f"{str_node(self.failed_node)}"
        super().__init__(self.message)


class NodeHasAttributesException(Exception):
    def __init__(self, c, n):
        self.failed_class = c
        self.failed_node = n
        self.message = f"Node should not have attributes: {type(self.failed_class).__name__} -> " \
                       f"{str_node(self.failed_node)}"
        super().__init__(self.message)


class NodeHasNoAttributesException(Exception):
    def __init__(self, c, n):
        self.failed_class = c
        self.failed_node = n
        self.message = f"Node should have attributes: {type(self.failed_class).__name__} -> " \
                       f"{str_node(self.failed_node)}"
        super().__init__(self.message)


def assert_node_children_and_attributes(xml_node: Xet.Element, has_child=True, has_attr=True, ignore_child=False, ignore_attr=False):
    if not ignore_child:
        if has_child and not len(xml_node):
            raise NodeHasNoChildrenException(None, xml_node)
        elif not has_child and len(xml_node):
            raise NodeHasChildrenException(None, xml_node)
    if not ignore_attr:
        if has_attr and not len(xml_node.attrib):
            raise NodeHasNoAttributesException(None, xml_node)
        elif not has_attr and len(xml_node.attrib):
            raise NodeHasAttributesException(None, xml_node)


def print_node(xml_node):
    print(str_node(xml_node))


def str_node(xml_node):
    return f"Node tag: {xml_node.tag} - Node text: {xml_node.text} - Node attrib: {xml_node.attrib}"


class FlagType:
    name = ''
    value = ''

    def __init__(self, name, value):
        self.name = name
        self.value = value


class SubDataType:
    sub_idx = ""
    name = ""
    data_type = ""
    bit_size = ""
    bit_offs = ""
    flags: List[FlagType] = []

    def __init__(self):
        self.sub_idx = ""
        self.name = ""
        self.data_type = ""
        self.bit_size = ""
        self.bit_offs = ""
        self.flags = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "SubItem":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)
        for child in xml_node:
            if child.tag == "SubIdx":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.sub_idx = child.text
            elif child.tag == "Name":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.name = child.text
            elif child.tag == "Type":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.data_type = child.text
            elif child.tag == "BitSize":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.bit_size = child.text
            elif child.tag == "BitOffs":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.bit_offs = child.text
            elif child.tag == "Flags":
                assert_node_children_and_attributes(child, has_attr=False)
                for grand_child in child:
                    assert_node_children_and_attributes(grand_child, has_child=False, has_attr=False)
                    self.flags.append(FlagType(grand_child.tag, grand_child.text))
            else:
                raise ChildTagException(self, xml_node, child)


class DataType:
    name = ""
    bit_size = ""
    base_type = ""
    array_lower_bound = ""
    array_elements = ""
    sub_items: List[SubDataType] = []

    def __init__(self):
        self.name = ""
        self.bit_size = ""
        self.base_type = ""
        self.array_lower_bound = ""
        self.array_elements = ""
        self.sub_items = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "DataType":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)

        for child in xml_node:
            if child.tag == "Name":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.name = child.text
            elif child.tag == "BitSize":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.bit_size = child.text
            elif child.tag == "SubItem":
                sub_item = SubDataType()
                sub_item.xml_parse(child)
                self.sub_items.append(sub_item)
            elif child.tag == "BaseType":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.base_type = child.text
            elif child.tag == "ArrayInfo":
                assert_node_children_and_attributes(child, has_attr=False)
                for grand_child in child:
                    if grand_child.tag == "LBound":
                        self.array_lower_bound = grand_child.text
                    elif grand_child.tag == "Elements":
                        self.array_elements = grand_child.text
                    else:
                        raise ChildTagException(self, child, grand_child)
            else:
                raise ChildTagException(self, xml_node, child)


class InfoEndItem:
    name = ""
    value = ""

    def __init__(self, name, value):
        self.name = name
        self.value = value


class InfoSubItem:
    name = ""
    value = ""
    info: List[InfoEndItem] = []

    def __init__(self):
        self.name = ""
        self.value = ""
        self.info = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "SubItem":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)

        for child in xml_node:
            if child.tag == "Name":
                self.name = child.text
            elif child.tag == "Info":
                if not len(child):
                    raise NodeHasNoChildrenException(self, child)
                for grand_child in child:
                    assert_node_children_and_attributes(grand_child, has_child=False, has_attr=False)
                    if grand_child.tag not in ["DefaultValue", "DefaultData"]:
                        raise TagException(self, grand_child)
                    self.info.append(InfoEndItem(grand_child.tag, grand_child.text))
            else:
                raise ChildTagException(self, xml_node, child)


class DictionaryObject:
    index = ""
    name = ""
    data_type = ""
    bit_size = ""
    info: List[InfoSubItem] = []
    flags: List[FlagType] = []

    def __init__(self):
        self.index = ""
        self.name = ""
        self.data_type = ""
        self.bit_size = ""
        self.info = []
        self.flags = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "Object":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)

        for child in xml_node:
            if child.tag == "Index":
                self.index = child.text
            elif child.tag == "Name":
                self.name = child.text
            elif child.tag == "Type":
                self.data_type = child.text
            elif child.tag == "BitSize":
                self.bit_size = child.text
            elif child.tag == "Info":
                assert_node_children_and_attributes(child, has_attr=False, ignore_child=True)
                for grand_child in child:
                    if grand_child.tag == "SubItem":
                        info_sub_item = InfoSubItem()
                        info_sub_item.xml_parse(grand_child)
                        self.info.append(info_sub_item)
                    elif grand_child.tag in ["DefaultData", "DefaultValue"]:
                        default_info_item = InfoSubItem()
                        default_info_item.name = grand_child.tag
                        default_info_item.value = grand_child.text
                        self.info.append(default_info_item)
                    else:
                        raise ChildTagException(self, child, grand_child)

            elif child.tag == "Flags":
                assert_node_children_and_attributes(child, has_attr=False)
                for grand_child in child:
                    assert_node_children_and_attributes(grand_child, has_child=False, has_attr=False)
                    self.flags.append(FlagType(grand_child.tag, grand_child.text))

            else:
                raise ChildTagException(self, xml_node, child)


class Dictionary:
    data_types: List[DataType] = []
    objects: List[DictionaryObject] = []

    def __init__(self):
        self.data_types = []
        self.objects = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "Dictionary":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)

        for child in xml_node:
            if child.tag == "DataTypes":
                assert_node_children_and_attributes(child, has_attr=False)
                for grand_child in child:
                    if grand_child.tag == "DataType":
                        data_type = DataType()
                        data_type.xml_parse(grand_child)
                        self.data_types.append(data_type)
                    else:
                        raise ChildTagException(self, child, grand_child)

            elif child.tag == "Objects":
                assert_node_children_and_attributes(child, has_attr=False)
                for grand_child in child:
                    if grand_child.tag == "Object":
                        dictionary_object = DictionaryObject()
                        dictionary_object.xml_parse(grand_child)
                        self.objects.append(dictionary_object)
                    else:
                        raise ChildTagException(self, child, grand_child)

            else:
                raise ChildTagException(self, xml_node, child)


class Profile:
    profile_no = ""
    dictionary: Dictionary = Dictionary()

    def __init__(self):
        self.profile_no = ""
        self.dictionary = Dictionary()

    def xml_parse(self, xml_node):
        if xml_node.tag != "Profile":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)

        for child in xml_node:
            if child.tag == "ProfileNo":
                assert_node_children_and_attributes(child, has_child=False, has_attr=False)
                self.profile_no = child.text
            elif child.tag == "Dictionary":
                self.dictionary.xml_parse(child)
            else:
                raise ChildTagException(self, xml_node, child)


class PdoEntry:
    index = ""
    sub_index = ""
    bit_len = ""
    name = ""
    data_type = ""

    def __init__(self):
        self.index = ""
        self.sub_index = ""
        self.bit_len = ""
        self.name = ""
        self.data_type = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Entry":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)

        for child in xml_node:
            if child.tag == "Index":
                self.index = child.text
            elif child.tag == "SubIndex":
                self.sub_index = child.text
            elif child.tag == "BitLen":
                self.bit_len = child.text
            elif child.tag == "Name":
                self.name = child.text
            elif child.tag == "DataType":
                self.data_type = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class PdoExclude:
    name = ''
    value = ''

    def __init__(self, name, value):
        self.name = name
        self.value = value


class PDO:
    pdo_type = ""
    fixed = ""
    sm = ""
    index = ""
    name = ""
    entries: List[PdoEntry] = []
    excludes: List[PdoExclude] = []

    def __init__(self, pdo_type):
        self.pdo_type = pdo_type
        self.fixed = ""
        self.sm = ""
        self.index = ""
        self.name = ""
        self.entries = []
        self.excludes = []

    def xml_parse(self, xml_node):
        if xml_node.tag not in ["RxPdo", "TxPdo"]:
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node)

        for attr in xml_node.attrib:
            if attr == "Fixed":
                self.fixed = xml_node.attrib[attr]
            elif attr == "Sm":
                self.sm = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)

        for child in xml_node:
            if child.tag == "Index":
                self.index = child.text
            elif child.tag == "Name":
                self.name = child.text
            elif child.tag == "Entry":
                entry = PdoEntry()
                entry.xml_parse(child)
                self.entries.append(entry)
            elif child.tag == "Exclude":
                self.excludes.append(PdoExclude(child.tag, child.text))
            else:
                raise ChildTagException(self, xml_node, child)


class DeviceSM:
    name = ""
    min_size = ""
    max_size = ""
    default_size = ""
    start_address = ""
    control_byte = ""
    enable = ""
    watchdog = ""

    def __init__(self):
        self.name = ""
        self.min_size = ""
        self.max_size = ""
        self.default_size = ""
        self.start_address = ""
        self.control_byte = ""
        self.enable = ""
        self.watchdog = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Sm":
            raise TagException(self, xml_node)
        self.name = xml_node.text

        if len(xml_node):
            raise NodeHasChildrenException(self, xml_node)

        if not len(xml_node.attrib):
            raise NodeHasNoAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "MinSize":
                self.min_size = xml_node.attrib[attr]
            elif attr == "MaxSize":
                self.max_size = xml_node.attrib[attr]
            elif attr == "DefaultSize":
                self.default_size = xml_node.attrib[attr]
            elif attr == "StartAddress":
                self.start_address = xml_node.attrib[attr]
            elif attr == "ControlByte":
                self.control_byte = xml_node.attrib[attr]
            elif attr == "Enable":
                self.enable = xml_node.attrib[attr]
            elif attr == "Watchdog":
                self.watchdog = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)


class CoEInitCmd:
    transition = ""
    index = ""
    sub_index = ""
    data = ""
    comment = ""

    def __init__(self):
        self.transition = ""
        self.index = ""
        self.sub_index = ""
        self.data = ""
        self.comment = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "InitCmd":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "Transition":
                self.transition = child.text
            elif child.tag == "Index":
                self.index = child.text
            elif child.tag == "SubIndex":
                self.sub_index = child.text
            elif child.tag == "Data":
                self.data = child.text
            elif child.tag == "Comment":
                self.comment = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class CoeMailbox:
    sdo_info = ""
    pdo_assign = ""
    pdo_config = ""
    complete_access = ""
    init_cmds: List[CoEInitCmd] = []

    def __init__(self):
        self.sdo_info = ""
        self.pdo_assign = ""
        self.pdo_config = ""
        self.complete_access = ""
        self.init_cmds = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "CoE":
            raise TagException(self, xml_node)

        # It may or may not have ImitCmd children
        if len(xml_node):
            for child in xml_node:
                if child.tag == "InitCmd":
                    init_cmd = CoEInitCmd()
                    init_cmd.xml_parse(child)
                    self.init_cmds.append(init_cmd)
                else:
                    raise ChildTagException(self, xml_node, child)

        if not len(xml_node.attrib):
            raise NodeHasNoAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "SdoInfo":
                self.sdo_info = xml_node.attrib[attr]
            elif attr == "PdoAssign":
                self.pdo_assign = xml_node.attrib[attr]
            elif attr == "PdoConfig":
                self.pdo_config = xml_node.attrib[attr]
            elif attr == "CompleteAccess":
                self.complete_access = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)


class Mailbox:
    eoe = ""
    foe = ""
    coe: CoeMailbox = CoeMailbox()

    def __init__(self):
        self.eoe = ""
        self.foe = ""
        self.coe = CoeMailbox()

    def xml_parse(self, xml_node):
        if xml_node.tag != "Mailbox":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "EoE":
                self.eoe = child.text
            elif child.tag == "CoE":
                self.coe.xml_parse(child)
            elif child.tag == "FoE":
                self.foe = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class OpMode:
    name = ""
    desc = ""
    assign_activate = ""
    cycle_time_sync0 = ""
    cycle_time_sync0_factor = ""
    shift_time_sync0 = ""

    def __init__(self):
        self.name = ""
        self.desc = ""
        self.assign_activate = ""
        self.cycle_time_sync0 = ""
        self.cycle_time_sync0_factor = ""
        self.shift_time_sync0 = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "OpMode":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "Name":
                self.name = child.text
            elif child.tag == "Desc":
                self.desc = child.text
            elif child.tag == "AssignActivate":
                self.assign_activate = child.text
            elif child.tag == "CycleTimeSync0":
                self.cycle_time_sync0 = child.text
                if "Factor" in child.attrib.keys():
                    self.cycle_time_sync0_factor = child.attrib["Factor"]
            elif child.tag == "ShiftTimeSync0":
                self.shift_time_sync0 = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class DC:
    op_modes: List[OpMode] = []

    def __init__(self):
        self.op_modes = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "Dc":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "OpMode":
                op_mode = OpMode()
                op_mode.xml_parse(child)
                self.op_modes.append(op_mode)
            else:
                raise ChildTagException(self, xml_node, child)


class EEPROMCategory:
    cat_no = ""
    data = ""

    def __init__(self):
        self.cat_no = ""
        self.data = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Category":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "CatNo":
                self.cat_no = child.text
            elif child.tag == "Data":
                self.data = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class EEPROM:
    byte_size = ""
    config_data = ""
    boot_strap = ""
    categories: List[EEPROMCategory] = []

    def __init__(self):
        self.byte_size = ""
        self.config_data = ""
        self.boot_strap = ""
        self.categories = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "Eeprom":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "ByteSize":
                self.byte_size = child.text
            elif child.tag == "ConfigData":
                self.config_data = child.text
            elif child.tag == "BootStrap":
                self.boot_strap = child.text
            elif child.tag == "Category":
                eeprom_category = EEPROMCategory()
                eeprom_category.xml_parse(child)
                self.categories.append(eeprom_category)
            else:
                raise ChildTagException(self, xml_node, child)


class DeviceType:
    device_type = ""
    product_code = ""
    revision_no = ""

    def __init__(self):
        self.device_type = ""
        self.product_code = ""
        self.revision_no = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Type":
            raise TagException(self, xml_node)
        self.device_type = xml_node.text

        if len(xml_node):
            raise NodeHasChildrenException(self, xml_node)

        if not len(xml_node.attrib):
            raise NodeHasNoAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "ProductCode":
                self.product_code = xml_node.attrib[attr]
            elif attr == "RevisionNo":
                self.revision_no = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)


class DeviceName:
    name = ""
    lc_id = ""

    def __init__(self):
        self.name = ""
        self.lc_id = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Name":
            raise TagException(self, xml_node)
        self.name = xml_node.text

        if len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)

        if not len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "LcId":
                self.lc_id = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)


class DeviceRegister:
    name = ""
    value = ""

    def __init__(self, name, value):
        self.name = name
        self.value = value


class DeviceInfo:
    registers: List[DeviceRegister] = []

    def __init__(self):
        self.registers = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "Info":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)
        for child in xml_node:
            self.registers.append(DeviceRegister(child.tag, child.text))


class DeviceEsc:
    registers: List[DeviceRegister] = []

    def __init__(self):
        self.registers = []

    def xml_parse(self, xml_node):
        if xml_node.tag != "ESC":
            raise TagException(self, xml_node)
        assert_node_children_and_attributes(xml_node, has_attr=False)
        for child in xml_node:
            self.registers.append(DeviceRegister(child.tag, child.text))


class Device:
    device_type: DeviceType = DeviceType()
    name: DeviceName = DeviceName()
    group_type = ""
    profile: Profile = Profile()
    fmmu: List[str] = []
    sm: List[DeviceSM] = []
    su = ""
    mbx: List[Mailbox] = []
    rxpdo: List[PDO] = []
    txpdo: List[PDO] = []
    dc: DC = DC()
    eeprom: EEPROM = EEPROM()
    image_16x14 = ""
    info: DeviceInfo = DeviceInfo()
    esc: DeviceEsc = DeviceEsc()
    physics = ""

    def __init__(self):
        self.device_type = DeviceType()
        self.name = DeviceName()
        self.group_type = ""
        self.profile = Profile()
        self.fmmu = []
        self.sm = []
        self.su = ""
        self.mbx = []
        self.rxpdo = []
        self.txpdo = []
        self.dc = DC()
        self.eeprom = EEPROM()
        self.image_16x14 = ""
        self.info = DeviceInfo()
        self.esc = DeviceEsc()
        self.physics = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Device":
            raise TagException(self, xml_node)

        if not len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "Physics":
                self.physics = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "Type":
                self.device_type.xml_parse(child)
            elif child.tag == "Name":
                self.name.xml_parse(child)
            elif child.tag == "GroupType":
                self.group_type = child.text
            elif child.tag == "Profile":
                self.profile.xml_parse(child)
            elif child.tag == "Fmmu":
                self.fmmu.append(child.text)
            elif child.tag == "Sm":
                sm = DeviceSM()
                sm.xml_parse(child)
                self.sm.append(sm)
            elif child.tag == "Su":
                self.su = child.text
            elif child.tag == "RxPdo":
                rxpdo = PDO("rx")
                rxpdo.xml_parse(child)
                self.rxpdo.append(rxpdo)
            elif child.tag == "TxPdo":
                txpdo = PDO("tx")
                txpdo.xml_parse(child)
                self.txpdo.append(txpdo)
            elif child.tag == "Mailbox":
                mbx = Mailbox()
                mbx.xml_parse(child)
                self.mbx.append(mbx)
            elif child.tag == "Dc":
                self.dc.xml_parse(child)
            elif child.tag == "Eeprom":
                self.eeprom.xml_parse(child)
            elif child.tag == "Image16x14":
                self.image_16x14 = child.text
            elif child.tag == "Info":
                self.info.xml_parse(child)
            elif child.tag == "ESC":
                self.esc.xml_parse(child)
            else:
                raise ChildTagException(self, xml_node, child)


class GroupName:
    name = ""
    lc_id = ""

    def __init__(self):
        self.name = ""
        self.lc_id = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Name":
            raise TagException(self, xml_node)

        if len(xml_node):
            raise NodeHasChildrenException(self, xml_node)

        if not len(xml_node.attrib):
            raise NodeHasNoAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "LcId":
                self.lc_id = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)


class Group:
    group_type = ""
    name: GroupName = GroupName()
    image_data = ""
    sort_order = ""
    name_lc_id = ""

    def __init__(self):
        self.group_type = ""
        self.name = GroupName()
        self.image_data = ""
        self.sort_order = ""
        self.name_lc_id = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Group":
            raise TagException(self, xml_node)

        if not len(xml_node.attrib):
            raise NodeHasNoAttributesException(self, xml_node)
        for attr in xml_node.attrib:
            if attr == "SortOrder":
                self.sort_order = xml_node.attrib[attr]
            else:
                raise NodeAttrException(self, xml_node, attr)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "Type":
                self.group_type = child.text
            elif child.tag == "Name":
                self.name.xml_parse(child)
            elif child.tag == "ImageData16x14":
                self.image_data = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class Vendor:
    id = ""
    name = ""
    image_data = ""

    def __init__(self):
        self.id = ""
        self.name = ""
        self.image_data = ""

    def xml_parse(self, xml_node):
        if xml_node.tag != "Vendor":
            raise TagException(self, xml_node)

        if len(xml_node.attrib):
            raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "Id":
                self.id = child.text
            elif child.tag == "Name":
                self.name = child.text
            elif child.tag == "ImageData16x14":
                self.image_data = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class Description:
    groups: List[Group] = []
    devices: List[Device] = []
    vendor: Vendor = Vendor()

    def __init__(self, esi_file):
        self.groups = []
        self.devices = []
        self.vendor = Vendor()
        self._root = Xet.parse(esi_file).getroot()
        self.xml_parse(self._root)

    def xml_parse(self, xml_node):
        if xml_node.tag != "EtherCATInfo":
            raise TagException(self, xml_node)

        # We don't care about the attributes that exist here
        # if len(xml_node.attrib):
        #     raise NodeHasAttributesException(self, xml_node)

        if not len(xml_node):
            raise NodeHasNoChildrenException(self, xml_node)
        for child in xml_node:
            if child.tag == "Vendor":
                self.vendor.xml_parse(child)
            elif child.tag == "Descriptions":
                for grand_child in child:
                    if grand_child.tag == "Groups":
                        for grand_grand_child in grand_child:
                            if grand_grand_child.tag == "Group":
                                group = Group()
                                group.xml_parse(grand_grand_child)
                                self.groups.append(group)
                            else:
                                raise ChildTagException(self, grand_child, grand_grand_child)
                    elif grand_child.tag == "Devices":
                        for grand_grand_child in grand_child:
                            if grand_grand_child.tag == "Device":
                                device = Device()
                                device.xml_parse(grand_grand_child)
                                self.devices.append(device)
                            else:
                                raise ChildTagException(self, grand_child, grand_grand_child)
                    else:
                        raise ChildTagException(self, child, grand_child)
            else:
                raise ChildTagException(self, xml_node, child)


def class_to_ros_msg_recursive(cls, store):
    """
    Converts a class to ROS message definition files.
    It will attempt to recurse into class-level members until composed of nested strings.
    It does so based on annotation information, so full annotation is required.
    :param cls: Root class to generate messages for
    :param store: Generated message storage as {file_name: file_content}
    :return: The generated messages as {file_name: file_content}
    """

    """ Things we reject messages for """
    if not callable(cls):
        raise Exception('This is not callable, which a raw class generally is .. ')
    if cls in [str, list, dict, Xet.Element]:
        raise Exception("Don't give me this kind of class: [str, list, dict, Xet.Element]. You gave: " +
                        str(cls.__name__))
    """ Filename """
    filename = 'Ecat' + cls.__name__ + 'Msg.msg'
    """ This is to force initialization of empty annotations """
    str(cls.__annotations__)
    """ Message content """
    msg = ''
    for key in cls.__dict__.keys():
        """ Reject dict keys with leading or trailing _ """
        if '_' in [str(key)[0], str(key)[-1]]:
            continue
        """ Reject functions """
        if cls.__dict__[key].__class__.__name__ == 'function':
            continue
        """ Load typing info if available """
        if key in cls.__annotations__.keys():
            type_args = typing.get_args(cls.__annotations__[key])
        else:
            type_args = ()
        if msg != '':
            msg += '\n'
        if len(type_args):
            if type_args[0] == str:
                msg += 'string[] ' + str(key)
            else:
                msg += type_args[0].__name__ + '[] ' + str(key)
        else:
            """ Cannot use annotations info, has to be dict """
            if cls.__dict__[key].__class__.__name__ == 'str':
                msg += 'string ' + str(key)
            else:
                msg += cls.__dict__[key].__class__.__name__ + ' ' + str(key)
    """ Fill store """
    if filename not in store.keys() or store[filename] in ['']:
        store[filename] = msg
    """ Call on annotated members """
    for key in cls.__annotations__.keys():
        type_args = typing.get_args(cls.__annotations__[key])
        if len(type_args):
            if type_args[0] != str:
                class_to_ros_msg_recursive(type_args[0], store)
        else:
            class_to_ros_msg_recursive(cls.__annotations__[key], store)


def main():
    device_esi_dir = './device-descriptions/'
    ros_msg_dir = './generated-ros/msg/'
    device_description = Description(device_esi_dir + "elmo-ecat-desc-00010420.xml")

    try:
        if os.path.isdir(ros_msg_dir):
            shutil.rmtree(ros_msg_dir)
        os.makedirs(ros_msg_dir)

        store = dict()
        class_to_ros_msg_recursive(Description, store)

        for key in store:
            header = '-- ' + str(key) + ' --'
            sep = ''.zfill(len(header)).replace('0', '-')
            # print(sep + '\n' + header)
            if str(store[key]) in ['', None]:
                raise Exception('No contents ...')
            else:
                # print(str(store[key]))

                with open(ros_msg_dir + str(key), 'w') as f:
                    f.write(str(store[key]))

            # print(sep)

    except Exception as ex:
        shutil.rmtree(ros_msg_dir)
        raise ex

    print('Done!')


if __name__ == "__main__":
    main()
