#!/usr/bin/python3

import argparse
import os
import sys
import shutil
import typing
import xml.etree.ElementTree as Xet

from ecat_esi_classes import Description


class Printer:
    """ So I won't have to change each print statement """
    def __init__(self, quiet: bool = False):
        self.quiet = quiet

    def __call__(self, s: str):
        if not self.quiet:
            print(s)


printer: Printer = Printer()


def object_to_yaml_recursive(obj, _content: typing.List[str] = None, _indent: int = 0) -> typing.List[str]:
    """
    Generating yaml formatted device description.
    :param obj: Filled out Description or Description sub-object
    :param _content: Content generated so far
    :param _indent: Whitespace indentation for this level
    :return: Yaml formatted device description as List[str]
    """
    """ Things we reject """
    if callable(obj):
        raise Exception('This object should not have a __call__ function')
    if obj.__class__ in [str, list, dict, Xet.Element, None]:
        raise Exception('Do not give me this type of object: ' + obj.__class__.__name__)
    """ Yaml content """
    if _content is None:
        _content = list()
    for key in obj.__dict__.keys():
        if '_' in str(key):
            continue
        if obj.__dict__[key] is None:
            continue
        if isinstance(obj.__dict__[key], str):
            line = ''.zfill(_indent).replace('0', ' ') + str(key) + ': "' + str(obj.__dict__[key]) + '"'
            _content.append(line)
        elif isinstance(obj.__dict__[key], list):
            _content.append(''.zfill(_indent).replace('0', ' ') + str(key) + ':')
            for item in obj.__dict__[key]:
                if isinstance(item, str):
                    _content.append(''.zfill(_indent + 2).replace('0', ' ') + str(key) +
                                    ': "' + item + '"')
                else:
                    object_to_yaml_recursive(item, _content, _indent + 2)
        else:
            object_to_yaml_recursive(obj.__dict__[key], _content, _indent + 2)
    """ Return something """
    return _content


def class_to_ros_msg_recursive(cls, store: dict) -> dict:
    """
    Converts a class to ROS message definition files.
    It will attempt to recurse into class-level members until composed of basic types.
    It does so based on annotation information, so full annotation is required on non-standard types.
    :param cls: Root class to generate messages for
    :param store: Generated message storage as {file_name: file_content}
    :return: The generated messages as {file_name: file_content}, same object as parameter store
    """
    """ Things we reject """
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
        if not len(type_args):
            class_to_ros_msg_recursive(cls.__annotations__[key], store)
        elif type_args[0] != str:
            class_to_ros_msg_recursive(type_args[0], store)
    """ Return """
    printer('  generated ' + filename)
    return store


def generate_ros_msgs(ros_msg_dir: str) -> None:
    if not os.path.isdir(ros_msg_dir):
        raise Exception('Output msg path provided is not a directory: ' + ros_msg_dir)
    try:
        """ Nuke the output directory """
        if os.path.isdir(ros_msg_dir):
            shutil.rmtree(ros_msg_dir)
        os.makedirs(ros_msg_dir)
        """ Generate a dict of {filename: ros_msg} """
        store = dict()
        class_to_ros_msg_recursive(Description, store)
        """ Save the generated content as files """
        for key in store:
            if str(store[key]) in ['', None]:
                raise Exception('No content in ROS message file!')
            else:
                with open(os.path.join(ros_msg_dir, str(key)), 'w') as f:
                    f.write(str(store[key]))

    except Exception as ex:
        """ Remove generated output on failures """
        if os.path.isdir(ros_msg_dir):
            shutil.rmtree(ros_msg_dir)
        raise ex


def generate_ros_device_description(device_description: Description) -> None:
    raise Exception('Unsupported feature: Generating output from device descriptions is not supported yet')


def get_files_in_path(esi_path: str, recurse=True) -> list:
    if not os.path.isdir(esi_path):
        raise Exception('Path provided is not a directory: ' + esi_path)
    files = []
    for f in os.listdir(esi_path):
        af = os.path.join(esi_path, f)
        if recurse and os.path.isdir(af):
            [files.append(r) for r in get_files_in_path(af)]
        elif not os.path.isdir(af):
            files.append(af)
    return files


def install(src_dir: str) -> None:
    ecat_interfaces_path = os.path.join(os.getcwd(), '..', '..', 'ros2', 'src', 'ecat_interfaces')
    """ Read msg and srv files in output directory """
    new_msgs = []
    new_srvs = []
    if os.path.isdir(os.path.join(src_dir, 'msg')) and len(os.listdir(os.path.join(src_dir, 'msg'))):
        new_msgs = [os.path.basename(msg).split('.') for msg in get_files_in_path(os.path.join(src_dir, 'msg'))]
    if os.path.isdir(os.path.join(src_dir, 'srv')) and len(os.listdir(os.path.join(src_dir, 'srv'))):
        new_srvs = [os.path.basename(srv).split('.') for srv in get_files_in_path(os.path.join(src_dir, 'srv'))]
    """ Copy files """
    files = new_msgs + new_srvs
    for new_file in files:
        source = os.path.join(src_dir, new_file[1], new_file[0] + '.' + new_file[1])
        destination = os.path.join(ecat_interfaces_path, new_file[1], new_file[0] + '.' + new_file[1])
        if os.path.exists(destination):
            printer('  overwriting ' + new_file[0] + '.' + new_file[1])
        else:
            printer('  creating ' + new_file[0] + '.' + new_file[1])
        shutil.copy(source, destination)
    """ Read and sort through contents of ecat_interfaces cmake """
    msg_dir = os.path.join(ecat_interfaces_path, 'msg')
    srv_dir = os.path.join(ecat_interfaces_path, 'srv')
    cmake_txt = os.path.join(ecat_interfaces_path, 'CMakeLists.txt')
    with open(cmake_txt) as f:
        cmake_content = f.read().splitlines()
    idx_start = [cmake_content.index(line) for line in cmake_content
                 if line == 'rosidl_generate_interfaces(${PROJECT_NAME}'][0] + 1
    idx_end = [cmake_content[idx_start:].index(line) + idx_start for line in cmake_content[idx_start:]
               if line == ')'][0]
    lines = [(line.split('/')[0],
              line.split('/')[1].split('.')[0],
              line.split('/')[1].split('.')[1]) for line in cmake_content[idx_start:idx_end]
             if ('msg/' in line and '.msg' in line) or ('srv/' in line and '.srv' in line)]
    """ Remove current entries in cmake content """
    for line in lines:
        li = line[0] + '/' + line[1] + '.' + line[2]
        if cmake_content.count(li):
            cmake_content.remove(li)
    """ Printing and adding new content """
    msgs = [os.path.basename(msg).split('.') for msg in get_files_in_path(msg_dir)]
    srvs = [os.path.basename(srv).split('.') for srv in get_files_in_path(srv_dir)]
    files = msgs + srvs
    for line in lines:
        if line[1] not in [file[0] for file in files]:
            printer('  Removing cmake entry: ' + line[1] + ('.msg' if line[2].find('.msg') else '.srv'))
    for file in files:
        if file[0] not in [line[1] for line in lines]:
            printer('  New cmake entry: ' + file[0] + '.' + file[1])
        str_line = ''.zfill(len(lines[0][0]) - len(lines[0][0].lstrip(' '))).replace('0', ' ') + \
                   '"' + file[1] + '/' + file[0] + '.' + file[1] + '"'
        cmake_content.insert(idx_start, str_line)
    """ Sorting new content """
    idx_end = [cmake_content[idx_start:].index(line)+idx_start for line in cmake_content[idx_start:] if line == ')'][0]
    new_content = [line for line in cmake_content[idx_start:idx_end] if line.strip(' ') != '']
    new_content.sort()
    cmake_content[idx_start:idx_end] = new_content
    """ Overwriting old cmake """
    os.remove(cmake_txt)
    with open(cmake_txt, 'w+') as f:
        f.write('\n'.join(cmake_content))


def main():
    """
    Options:
        -f, --esi-file [file]
            Run on a single ESI file
        -g, --generate <msg, description> ..
            Generate ROS definition files
        -i, --install
            Install generated files in the ecat_interfaces package
        -o, --output <path>
            Override definition file output directory
            Can only be used with -g
        -p, --esi-path <path>
            Override ESI directory path
        -q, --quiet
            Run silently
        -r, --recurse
            Run recursively into the ESI directory when looking for ESI files
            The default behaviour is to look only in the top level ESI directory
        -t, --test
            Run generate steps without generating output
            Same as '-r -g msg description' with no output
    """
    parser = argparse.ArgumentParser(
        description='Generating ROS msg and srv files from EtherCAT ESI files',
        formatter_class=argparse.RawTextHelpFormatter)

    default_output_path = os.path.join(os.getcwd(), 'generated-ros')
    default_esi_path = os.path.join(os.getcwd(), 'device-descriptions')
    default_esi_file = os.path.join(default_esi_path, 'elmo-ecat-desc-00010420.xml')

    parser.add_argument('-f', '--file', const=f'{default_esi_file}', nargs='?', default=None, type=str, dest='esi_file',
                        help=f'ESI file to use'
                             f'\n(default if no arg: {default_esi_file})')

    parser.add_argument('-g', '--generate', type=str, nargs='+', dest='generate', action='append',
                        choices=['msg', 'description'], default=None,
                        help='which output to generate')

    parser.add_argument('-i', '--install', action='store_true', dest='install',
                        help='install generated files\n'
                             'if used with -g new files are installed\n'
                             '(default: install files from default directory)')

    parser.add_argument('-o', '--output', default=default_output_path, type=str, nargs=1, dest='output_path',
                        help='output path')

    parser.add_argument('-p', '--path', default=default_esi_path, type=str, dest='esi_path', nargs=1,
                        help="path to use containing only ESI files")

    parser.add_argument('-q', '--quiet', action='store_true', dest='quiet',
                        help='run silently')

    parser.add_argument('-r', '--recurse', action='store_true', dest='recurse',
                        help='recurse into ESI path (default: top-level only)')

    parser.add_argument('-t', '--test', action='store_true', dest='test',
                        help='test generators with no output')

    args = parser.parse_args()
    printer.quiet = args.quiet
    esi_files = []

    if args.recurse and args.esi_file:
        printer('Cannot recurse over a single file, ignoring -r')
        args.recurse = None

    if args.recurse and not args.esi_file:
        printer('Recursing into ESI path: ' + args.esi_path)
        esi_files = get_files_in_path(args.esi_path)
        for f in esi_files:
            printer('    ' + os.path.basename(f))
    elif not args.recurse and args.esi_file:
        printer('Using provided file: ' + args.esi_file)
        esi_files.append(args.esi_file)
    else:
        printer('Using top level ESI path: ' + args.esi_path)
        esi_files = get_files_in_path(args.esi_path, recurse=False)
        for f in esi_files:
            printer('    ' + os.path.basename(f))

    if args.generate is not None:
        if len(args.generate) > 1:
            raise Exception('I did not know the args.generate list could have more than one entry')

        if 'msg' in args.generate[0]:
            printer('Generating ROS message files')
            if not os.path.isdir(os.path.join(args.output_path, 'msg')):
                os.makedirs(os.path.join(args.output_path, 'msg'))
            generate_ros_msgs(os.path.join(args.output_path, 'msg'))

        if 'description' in args.generate[0]:
            printer('Generating ROS device description files')
            for esi_file in esi_files:
                printer('    ' + os.path.basename(esi_file))
                device_description = Description(esi_file)
                generate_ros_device_description(device_description)

    if args.install:
        printer('Installing files into ecat_interfaces')
        install(args.output_path)

    printer('Done!')


if __name__ == "__main__":
    # main()

    default_esi_file = os.path.join(os.getcwd(), 'device-descriptions', 'elmo', 'elmo-ecat-desc-00010420.xml')
    device = Description(default_esi_file)
    device_yaml = object_to_yaml_recursive(device)
    for yline in device_yaml:
        print(yline)
