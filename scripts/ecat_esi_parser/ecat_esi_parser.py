#!/usr/bin/python3

import argparse
import os
import sys
import shutil
import typing
import xml.etree.ElementTree as Xet

from ecat_esi_classes import Description


def class_to_ros_msg_recursive(cls, store: dict) -> dict:
    """
    Converts a class to ROS message definition files.
    It will attempt to recurse into class-level members until composed of nested strings.
    It does so based on annotation information, so full annotation is required on non-standard types.
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
        if not len(type_args):
            class_to_ros_msg_recursive(cls.__annotations__[key], store)
        elif type_args[0] != str:
            class_to_ros_msg_recursive(type_args[0], store)
    """ Return """
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


def install() -> None:
    ecat_interfaces_path = os.path.join(os.getcwd(), '..', '..', 'ros2', 'src', 'ecat_interfaces')
    src_dir = os.path.join(os.getcwd(), 'generated-ros')
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
            print('  Overwriting file: ' + new_file[0] + '.' + new_file[1])
        else:
            print('  Copying new file: ' + new_file[0] + '.' + new_file[1])
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
    """ Add or remove msg and srv entries in cmake """
    for line in lines:
        li = line[0] + '/' + line[1] + '.' + line[2]
        if cmake_content.count(li):
            cmake_content.remove(li)
    msgs = [os.path.basename(msg).split('.') for msg in get_files_in_path(msg_dir)]
    srvs = [os.path.basename(srv).split('.') for srv in get_files_in_path(srv_dir)]
    files = msgs + srvs
    for line in lines:
        if line[1] not in [file[0] for file in files]:
            print('  Removing cmake entry: ' + line[1] + ('.msg' if line[2].find('.msg') else '.srv'))
    for file in files:
        if file[0] not in [line[1] for line in lines]:
            print('  New cmake entry: ' + file[0] + '.' + file[1])
        str_line = ''.zfill(len(lines[0][0]) - len(lines[0][0].lstrip(' '))).replace('0', ' ') + \
                   '"' + file[1] + '/' + file[0] + '.' + file[1] + '"'
        cmake_content.insert(idx_start, str_line)
    idx_end = [cmake_content[idx_start:].index(line)+idx_start for line in cmake_content[idx_start:] if line == ')'][0]
    new_content = [line for line in cmake_content[idx_start:idx_end] if line.strip(' ') != '']
    new_content.sort()
    cmake_content[idx_start:idx_end] = new_content
    os.remove(cmake_txt)
    with open(cmake_txt, 'w+') as f:
        f.write('\n'.join(cmake_content))


def main():
    """
    Options:
        -g, --generate <msg, description> ..
            Generate ROS definition files
        -p, --esi-path <path>
            Override ESI directory path
        -f, --esi-file [file]
            Run on a single ESI file
        -r, --recurse
            Run recursively into the ESI directory when looking for ESI files
            The default behaviour is to look only in the top level ESI directory
        -o, --output <path>
            Override definition file output directory
            Can only be used with -g
        -i, --install
            Install generated files in the ecat_interfaces package
            If -g is not provided, the default output directory will be installed
            If -g is provided, new output will be installed
    """
    parser = argparse.ArgumentParser(
        description='Generating ROS message files and device descriptions from EtherCAT ESI files',
        formatter_class=argparse.RawTextHelpFormatter)

    default_output_path = os.path.join(os.getcwd(), 'generated-ros')
    default_esi_path = os.path.join(os.getcwd(), 'device-descriptions')

    parser.add_argument('-f', '--file', default=None, nargs=1, type=str, dest='esi_file',
                        help="ESI file to use")

    parser.add_argument('-g', '--generate', type=str, nargs='+', dest='generate', action='append',
                        choices=['msg', 'description'], default=None,
                        help='which output to generate')

    parser.add_argument('-i', '--install', action='store_true', dest='install',
                        help='install generated files \n'
                             'if used with -g new files are installed \n'
                             '(default: install files from default directory)')

    parser.add_argument('-o', '--output', default=default_output_path, type=str, nargs=1, dest='output_path',
                        help='output path')

    parser.add_argument('-p', '--path', default=default_esi_path, type=str, dest='esi_path', nargs=1,
                        help="path to use containing only ESI files")

    parser.add_argument('-r', '--recurse', action='store_true', dest='recurse',
                        help='recurse into ESI path (default: top-level only)')

    args = parser.parse_args()
    esi_files = []

    try:
        if args.recurse and args.esi_file:
            print('Cannot recurse over a single file, ignoring -r')
            args.recurse = None

        if args.recurse and not args.esi_file:
            print('Recursing into ESI path: ' + args.esi_path)
            esi_files = get_files_in_path(args.esi_path)
            for f in esi_files:
                print('    ' + os.path.basename(f))
        elif not args.recurse and args.esi_file:
            print('Using provided file: ' + args.esi_file)
            esi_files.append(args.esi_file)
        else:
            print('Using top level ESI path: ' + args.esi_path)
            esi_files = get_files_in_path(args.esi_path, recurse=False)
            for f in esi_files:
                print('    ' + os.path.basename(f))

        if args.generate is not None:
            if len(args.generate) > 1:
                raise Exception('I did not know the args.generate list could have more than one entry')

            if 'msg' in args.generate[0]:
                print('Generating ROS message files')
                if not os.path.isdir(os.path.join(args.output_path, 'msg')):
                    os.makedirs(os.path.join(args.output_path, 'msg'))
                generate_ros_msgs(os.path.join(args.output_path, 'msg'))
                msgs = os.listdir(os.path.join(args.output_path, 'msg'))
                for msg in msgs:
                    print('  ' + msg)

            if 'description' in args.generate[0]:
                print('Generating ROS device description files')
                for esi_file in esi_files:
                    print('    ' + os.path.basename(esi_file))
                    device_description = Description(esi_file)
                    generate_ros_device_description(device_description)

        if args.install:
            print('Installing files into ecat_interfaces')
            install()

        print('Done!')

    except Exception as ex:
        print('Exception during operation: ' + str(ex))


if __name__ == "__main__":
    main()
