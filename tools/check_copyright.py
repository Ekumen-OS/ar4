#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright 2025 Ekumen, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Check that files have a copyright notice.

This tool checks that a copyright notice matching a specific template exists in the
provided files. It can also fix the file by adding a copyright notice or replacing
an existing one.
"""

import argparse
import datetime
import re
import textwrap

from pathlib import Path

DEFAULT_COPYRIGHT_HOLDER = 'Ekumen, Inc.'

# Template for the copyright notice.
# This should not contain any regex patterns since special characters will be escaped.
DEFAULT_COPYRIGHT_TEMPLATE = '''\
BSD 3-Clause License

Copyright {year} {name}
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\
'''


class FileContentChecker:
    """Helper class to check the contents of a file and apply fixes."""

    # Regex pattern that matches a year (YYYY) and a year range (YYYY-ZZZZ).
    # It has capturing groups for both years (the second one is optional).
    _YEAR_PATTERN = r'([0-9]{4})(?:-([0-9]{4}))?'

    # Regex pattern that matches the name of a copyright holder.
    _NAME_PATTERN = r'(.*)'

    # Regex pattern that matches anything that looks like a copyright block.
    # Specifically, a comment beginning with the word `copyright` (case insensitive)
    # followed by single line comments that form a block.
    _COPYRIGHT_PATTERN = r'(?:[;#%|*]|//)\s*[Cc]opyright.*(?:\n(?:[;#%|*]|//).*)*'

    def __init__(self, path: Path):
        """Initialize a file content checker."""
        self._path = path
        with open(path) as file:
            self._content = file.read()

    def _comment_lines(self, text: str) -> str:
        """Comment text lines using the syntax associated with the file extension.

        :param text: Input text.
        :return: The modified text.
        """
        comment_prefix = {
            '.py': '#',
            '.cpp': '//',
            '.hpp': '//',
        }

        base = textwrap.indent(text, ' ', None)
        return textwrap.indent(
            base, comment_prefix.get(self._path.suffix, '#'), lambda line: True
        )

    def search_copyright(self, template: str = DEFAULT_COPYRIGHT_TEMPLATE) -> re.Match:
        """Search for a valid copyright notice using the given the template.

        :param template: Template copyright notice to match with.
        :return: The corresponding match object.
        """
        pattern = (
            # Comment lines and escape special characters in the template...
            re.escape(self._comment_lines(template))
            # Un-escape the format fields...
            .replace(r'\{', '{').replace(r'\}', '}')
            # Then insert year and name patterns.
            .format(year=self._YEAR_PATTERN, name=self._NAME_PATTERN)
        )
        return re.search(pattern, self._content)

    def fix_copyright(
        self,
        template: str = DEFAULT_COPYRIGHT_TEMPLATE,
        name: str = DEFAULT_COPYRIGHT_HOLDER,
        year: int = datetime.date.today().year,
    ) -> None:
        """Fix copyright notice.

        Searches for a copyright notice block and replaces it with the template.
        If no copyright block is found, adds the notice at the top of the
        file preserving the shebang line if it exists.

        The new copyright notice will be preceded by an empty line if not at
        the beginning of the file, and will always be followed by an empty line.

        :param template: Template copyright notice to use when adding/fixing.
        :param name: Name of the copyright holder.
        :param year: Copyright year.
        """

        def fix_content() -> str:
            """Return the fixed file content."""
            copyright_notice = self._comment_lines(
                template.format(year=year, name=name)
            )

            match = re.search(self._COPYRIGHT_PATTERN, self._content)
            if match:
                # Found something that looks like a copyright block...
                # Replace it with the template.
                return self._content.replace(match.group(0), copyright_notice)

            lines = self._content.splitlines()
            if lines and lines[0].startswith('#!'):
                # Keep shebang line at the top.
                lines.insert(1, '\n' + copyright_notice + '\n')
                return '\n'.join(lines)

            return copyright_notice + '\n\n' + self._content

        with open(self._path, 'w') as file:
            self._content = fix_content()
            file.write(self._content)


def main(argv=None) -> int:
    """Run the entry point of the program."""
    parser = argparse.ArgumentParser(description=globals()['__doc__'])
    parser.add_argument(
        'paths',
        type=Path,
        nargs='+',
        help='files to check',
    )
    parser.add_argument(
        '-f',
        '--fix',
        action='store_true',
        help='whether to apply the detected fixes',
    )
    parser.add_argument(
        '-n',
        '--copyright-owner-name',
        type=str,
        default=DEFAULT_COPYRIGHT_HOLDER,
        help='copyright owner name',
        metavar='NAME',
    )

    args = parser.parse_args(argv)

    # Whether any of the files need to be fixed.
    need_fix: bool = False

    excluded_pkgs = ['ar4_description', 'ar4_hardware_interface']

    for path in args.paths:
        if any(excluded_pkg in str(path) for excluded_pkg in excluded_pkgs):
            continue
        checker = FileContentChecker(path)
        match = checker.search_copyright()
        if not match:
            print(f'{path}: does not have a valid copyright notice')
            need_fix = True
            if args.fix:
                checker.fix_copyright(name=args.copyright_owner_name)

    return 0 if not need_fix else 1


if __name__ == '__main__':
    exit(main())
