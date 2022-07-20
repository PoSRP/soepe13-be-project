#!/usr/bin/python3

import argparse
import os
import sys
import shutil
import typing
import xml.etree.ElementTree as Xet

from ecat_esi_classes import Description


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
        if not len(type_args):
            class_to_ros_msg_recursive(cls.__annotations__[key], store)
        elif type_args[0] != str:
            class_to_ros_msg_recursive(type_args[0], store)


def generate_ros_msgs(ros_msg_dir):
    try:
        if os.path.isdir(ros_msg_dir):
            shutil.rmtree(ros_msg_dir)
        os.makedirs(ros_msg_dir)

        store = dict()
        class_to_ros_msg_recursive(Description, store)

        for key in store:
            if str(store[key]) in ['', None]:
                raise Exception('No content in ROS message file!')
            else:
                with open(os.path.join(ros_msg_dir, str(key)), 'w') as f:
                    f.write(str(store[key]))

    except Exception as ex:
        shutil.rmtree(ros_msg_dir)
        raise ex


if __name__ == "__main__":
    """
    Options:
        -g, --generate <msg, description> ..
            Generate ROS definition files
        -p, --esi-path <path>
            Override ESI directory path
        -f, --esi-file [file]
            Run on a single ESI file
            If not provided, the largest .xml file will be used
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

    default_output_path = os.getcwd() + '/generated-ros'
    default_esi_path = os.getcwd() + '/device-descriptions'

    parser.add_argument('-f', '--file', default=None, type=str, dest='esi_file',
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

    if args.recurse and args.esi_file:
        print('Cannot recurse over a single file, ignoring -r')
    elif args.recurse and not args.esi_file:
        print('Recurse selected')
    elif not args.recurse and args.esi_file:
        print('Using provided file: ' + args.esi_file)

    if args.generate is not None:
        if 'msg' in args.generate:
            print('Generating ROS message files')
            generate_ros_msgs(args.output_path + '/msg')
        if 'description' in args.generate:
            print('Generating ROS device description files')
            device_description = Description(args.esi_path + "/elmo-ecat-desc-00010420.xml")

    if args.install:
        print('Installing files into ecat_interfaces')

    print('Done!')
