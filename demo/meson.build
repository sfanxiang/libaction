# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
# This Source Code Form is "Incompatible With Secondary Licenses", as
# defined by the Mozilla Public License, v. 2.0.

executable('action-still',
           sources : ['action_still.cpp'],
           include_directories : include_directories('../include'),
           dependencies : [deptensorflow, dependency('threads')])

executable('action-motion',
           sources : ['action_motion.cpp'],
           include_directories : include_directories('../include'),
           dependencies : [deptensorflow, dependency('threads')])
