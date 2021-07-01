#!/usr/bin/env python
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import os
import argparse


def xml_file(string):
    if not os.path.isfile(string):
        err_msg = 'no such file: %s ' % string
        raise argparse.ArgumentTypeError(err_msg)
    if not string.endswith('.xml'):
        raise argparse.ArgumentTypeError('unsupported format')
    return string


PARSER = argparse.ArgumentParser(description='Parse TM Ethernet xml file')
PARSER.add_argument(
    '--xml-dir', help='xml exported from TMFlow', type=xml_file)
PARSER.add_argument('--msg-dir', type=str)
PARSER.add_argument('--print-content', action='store_true',
                    dest='print_content')

TM_TYPE_TO_ROS_MSG_MAP = {"byte": "uint8", "int": "int32",
                          "float": "float32", "double": "float64", "bool": "bool", "string": "string"}


def main():
    args = PARSER.parse_args()

    tree = ET.parse(args.xml_dir)
    with open(args.msg_dir, 'w+') as f:
        for child in tree.getroot().iter('Setting'):
            type_str = TM_TYPE_TO_ROS_MSG_MAP[child.attrib['Type']]
            if child.attrib['Size'] != '1':
                type_str += "[]"

            var_name = child.attrib['Item']
            f.write('{type} {name}\n'.format(type=type_str, name=var_name))


if __name__ == '__main__':
    main()
