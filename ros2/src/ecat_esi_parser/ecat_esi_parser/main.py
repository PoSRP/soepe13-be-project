import argparse
import os
import sys
from esi_parser import *

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
    main()

    # default_esi_file = os.path.join(os.getcwd(), 'device-descriptions', 'elmo', 'elmo-ecat-desc-00010420.xml')
    # device = Description(default_esi_file)
    # device_yaml = object_to_yaml_recursive(device)
    # for yline in device_yaml:
        # print(yline)
