#!/usr/bin/env python3
"""Check the metadata and asset files that a ROS 2 build and a release depend on.

A malformed manifest or world file is the kind of mistake that only surfaces at run time,
so it is checked on every push instead: every `package.xml` must parse and carry the fields
the tooling reads, every XML and YAML asset must parse, and every text file must use LF
endings so that a checkout on any platform produces the same bytes.

Run it from anywhere in the repository:

    python3 .github/scripts/check_manifests.py
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]

REQUIRED_MANIFEST_FIELDS = ('name', 'version', 'description', 'maintainer', 'license')

SKIP_DIRS = {'build', 'install', 'log', '.git', '__pycache__', '.pytest_cache'}

TEXT_NAMES = {
    '.gitattributes', '.gitignore', '.editorconfig', '.flake8',
}

TEXT_SUFFIXES = {
    '.sh', '.py', '.xml', '.yaml', '.yml', '.json', '.cfg', '.md', '.cff', '.txt',
    '.sdf', '.world', '.xacro', '.urdf', '.rviz',
}

XML_SUFFIXES = {'.xml', '.world', '.sdf', '.urdf', '.xacro'}

YAML_SUFFIXES = {'.yaml', '.yml', '.cff', '.rviz'}

problems = []


def problem(message):
    problems.append(message)
    print('::error::' + message)


def iter_files():
    found = []
    for path in ROOT.rglob('*'):
        if not path.is_file():
            continue
        if SKIP_DIRS & set(path.relative_to(ROOT).parts):
            continue
        found.append(path)
    return sorted(found)


def check_manifests(files):
    manifests = [p for p in files if p.name == 'package.xml']
    if not manifests:
        problem('no package.xml found; the manifest check would pass vacuously')
        return

    for path in manifests:
        rel = path.relative_to(ROOT)
        try:
            manifest = ET.parse(str(path)).getroot()
        except ET.ParseError as exc:
            problem('{}: not valid XML: {}'.format(rel, exc))
            continue

        if manifest.tag != 'package':
            problem('{}: root element is <{}>, expected <package>'.format(rel, manifest.tag))

        for field in REQUIRED_MANIFEST_FIELDS:
            element = manifest.find(field)
            if element is None or not (element.text or '').strip():
                problem('{}: missing or empty <{}>'.format(rel, field))

        maintainer = manifest.find('maintainer')
        if maintainer is not None and not (maintainer.get('email') or '').strip():
            problem('{}: <maintainer> has no email attribute'.format(rel))

        if manifest.find('url') is None:
            problem('{}: no <url> element, so the package cannot be traced back'.format(rel))

    print('checked {} package.xml files'.format(len(manifests)))


def check_yaml(files):
    documents = [p for p in files if p.suffix in YAML_SUFFIXES]
    for path in documents:
        rel = path.relative_to(ROOT)
        try:
            with path.open('rb') as handle:
                list(yaml.safe_load_all(handle))
        except yaml.YAMLError as exc:
            problem('{}: not valid YAML: {}'.format(rel, exc))
    print('checked {} YAML documents'.format(len(documents)))


def check_xml_assets(files):
    assets = [p for p in files
              if p.suffix in XML_SUFFIXES and p.name != 'package.xml']
    for path in assets:
        rel = path.relative_to(ROOT)
        try:
            ET.parse(str(path))
        except ET.ParseError as exc:
            problem('{}: not valid XML: {}'.format(rel, exc))
    print('checked {} XML assets'.format(len(assets)))


def check_line_endings(files):
    offenders = []
    for path in files:
        if path.suffix not in TEXT_SUFFIXES and path.name not in TEXT_NAMES:
            continue
        if b'\r\n' in path.read_bytes():
            offenders.append(str(path.relative_to(ROOT)))
    if offenders:
        problem('CRLF line endings in: ' + ', '.join(offenders))
    print('checked line endings in {} files'.format(len(files)))


def main():
    files = iter_files()
    check_manifests(files)
    check_yaml(files)
    check_xml_assets(files)
    check_line_endings(files)

    if problems:
        print('\n{} problem(s) found'.format(len(problems)))
        return 1
    print('\nall metadata checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
