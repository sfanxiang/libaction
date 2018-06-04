#!/bin/env python

import os

HEADER_EXTENSION = '.hpp'

for root, dirs, files in os.walk('include'):
    relpath = os.path.relpath(root, 'include')
    for file in files:
        path = os.path.join(relpath, file)
        if path.endswith(HEADER_EXTENSION):
            full_path = os.path.join(root, file)
            guard_name = path.replace('/', '__').replace('.', '_').upper() + '_'

            print('%s: %s' % (full_path, guard_name))

            content = []
            with open(full_path, 'r') as f:
                content = [line.strip('\r').strip('\n') for line in f]

            # remove old guards
            if content and content[-1] == '':
                content = content[:-1]
            if len(content) >= 5 and content[0].startswith('#ifndef') and \
                    content[1].startswith('#define') and \
                    content[2] == '' and \
                    content[-2] == '' and \
                    content[-1] == '#endif':
                content = content[3:-2]

            content = ['#ifndef ' + guard_name,
                       '#define ' + guard_name,
                       ''] + \
                      content + \
                      ['',
                       '#endif']

            with open(full_path, 'w') as f:
                for line in content:
                    f.write(line + '\n')
