project('libaction', 'cpp',
        default_options : ['cpp_std=c++11'])

deptensorflow = dependency('tensorflow-lite')

subdir('demo')
subdir('include')

pkg = import('pkgconfig')
pkg.generate(version : '1.0',
             name : 'libaction',
             filebase : 'libaction',
             description : 'Libaction')
