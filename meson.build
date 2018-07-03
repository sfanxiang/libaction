# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
# This Source Code Form is "Incompatible With Secondary Licenses", as
# defined by the Mozilla Public License, v. 2.0.

project('libaction', 'cpp',
        default_options : ['cpp_std=c++11'],
        version : '1.0')

cpp = meson.get_compiler('cpp')
requires = []
libraries = [
    cpp.find_library('tensorflow-lite'),
    # libdl is required by TensorFlow Lite on some platforms
    cpp.find_library('dl', required : false),
    dependency('threads')]
dependencies = requires + libraries

subdir('demo')
subdir('include')

pkg = import('pkgconfig')
pkg.generate(version : meson.project_version(),
             name : 'libaction',
             filebase : 'libaction',
             description : 'Libaction')
