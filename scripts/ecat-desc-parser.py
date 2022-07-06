import xml.etree.ElementTree as Xet

print_debug = True


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


def print_node(xml_node):
    print(str_node(xml_node))


def str_node(xml_node):
    return f"Node tag: {xml_node.tag} - Node text: {xml_node.text} - Node attrib: {xml_node.attrib}"


def dprint(msg: str):
    if print_debug:
        print(msg)


class SubDataType:
    def __init__(self):
        self.sub_idx = ""
        self.name = ""
        self.type = ""
        self.bit_size = ""
        self.bit_offs = ""
        self.flags = {}

    def xml_parse(self, xml_node):
        pass


class DataType:
    def __init__(self):
        self.name = ""
        self.bit_size = ""
        self.sub_items = []

    def xml_parse(self, xml_node):
        pass


class InfoSubItem:
    def __init__(self):
        self.name = ""
        self.info = []

    def xml_parse(self, xml_node):
        pass


class DictionaryObject:
    def __init__(self):
        self.index = ""
        self.name = ""
        self.type = ""
        self.bit_size = ""
        self.info = []
        self.flags = {}

    def xml_parse(self, xml_node):
        pass


class Profile:
    def __init__(self):
        self.profile_no = ""
        self.data_types = []
        self.dictionary_objects = []

    def xml_parse(self, xml_node):
        pass


class Entry:
    def __init__(self):
        self.index = ""
        self.subindex = ""
        self.bitlen = ""
        self.name = ""
        self.data_type = ""

    def xml_parse(self, xml_node):
        pass


class PDO:
    def __init__(self):
        self.fixed = ""
        self.sm = ""
        self.index = ""
        self.name = ""
        self.entries = []

    def xml_parse(self, xml_node):
        pass


class RxPDO(PDO):
    def __init__(self):
        self.type = "rx"
        super().__init__()


class TxPDO(PDO):
    def __init__(self):
        self.type = "tx"
        super().__init__()


class DeviceSM:
    def __init__(self):
        self.name = ""
        self.min_size = ""
        self.max_size = ""
        self.default_size = ""
        self.start_address = ""
        self.control_byte = ""
        self.enable = ""
        self.watchdog = ""

    def __str__(self):
        return f"Device SM - {self.name}\n" \
               f"  Min size: {self.min_size}\n" \
               f"  Max size: {self.max_size}\n" \
               f"  Default size: {self.default_size}\n" \
               f"  Start address: {self.start_address}\n" \
               f"  Control byte: {self.control_byte}\n" \
               f"  Enable: {self.enable}\n" \
               f"  Watchdog: {self.watchdog}"

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


class CoeMailbox:
    def __init__(self):
        self.sdo_info = ""
        self.pdo_assign = ""
        self.pdo_config = ""
        self.complete_access = ""

    def xml_parse(self, xml_node):
        pass


class Mailbox:
    def __init__(self):
        self.eoe = ""
        self.foe = ""
        self.coe = CoeMailbox()

    def xml_parse(self, xml_node):
        pass


class OpMode:
    def __init__(self):
        self.name = ""
        self.desc = ""
        self.assign_activate = ""
        self.cycle_time_sync0 = ""
        self.cycle_time_sync0_factor = ""
        self.shift_time_sync0 = ""

    def xml_parse(self, xml_node):
        pass


class DC:
    def __init__(self):
        self.op_modes = []

    def xml_parse(self, xml_node):
        pass


class EEPROMCategory:
    def __init__(self):
        self.cat_no = ""
        self.data = ""

    def xml_parse(self, xml_node):
        pass


class EEPROM:
    def __init__(self):
        self.byte_size = ""
        self.config_data = ""
        self.boot_strap = ""
        self.categories = []

    def xml_parse(self, xml_node):
        pass


class DeviceType:
    def __init__(self):
        self.type = ""
        self.product_code = ""
        self.revision_no = ""

    def xml_parse(self, xml_node):
        pass


class DeviceName:
    def __init__(self):
        self.name = ""
        self.lc_id = ""

    def xml_parse(self, xml_node):
        pass


class DeviceInfo:
    def __init__(self):
        pass

    def xml_parse(self, xml_node):
        pass


class DeviceESC:
    def __init__(self):
        pass

    def xml_parse(self, xml_node):
        pass


class Device:
    def __init__(self):
        self.type = DeviceType()
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
        self.esc = DeviceESC()
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
                self.type.xml_parse(child)
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
                rxpdo = RxPDO()
                rxpdo.xml_parse(child)
                self.rxpdo.append(rxpdo)
            elif child.tag == "TxPdo":
                txpdo = TxPDO()
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
    def __init__(self):
        self.type = ""
        self.name = GroupName()
        self.image_data = ""
        self.sort_order = ""
        self.name_lc_id = ""

    def __str__(self):
        img_data = self.image_data
        s = ""
        while img_data:
            s += f"    {img_data[:58]}\n"
            img_data = img_data[58:]
        if s[-1] == "\n":
            s = s[0:-1]
        return f"Group - {self.name}\n" \
               f"  Type: {self.type}\n" \
               f"  Image data 16x14:\n{s}"

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
                self.type = child.text
            elif child.tag == "Name":
                self.name.xml_parse(child)
            elif child.tag == "ImageData16x14":
                self.image_data = child.text
            else:
                raise ChildTagException(self, xml_node, child)


class Description:
    def __init__(self):
        self.groups = []
        self.devices = []
        self.vendor = Vendor()

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


class Vendor:
    def __init__(self):
        self.id = ""
        self.name = ""
        self.image_data = ""

    def __str__(self):
        img_data = self.image_data
        s = ""
        while img_data:
            s += f"    {img_data[:58]}\n"
            img_data = img_data[58:]
        if s[-1] == "\n":
            s = s[0:-1]
        return f"Vendor - {self.name}\n" \
               f"  ID: {self.id}\n" \
               f"  Image data 16x14:\n{s}"

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


def main():
    dprint("Hello")

    tree = Xet.parse("../device-descriptions/elmo-ecat-desc-00010420.xml")
    root = tree.getroot()

    device_description = Description()
    device_description.xml_parse(root)


if __name__ == "__main__":
    main()
