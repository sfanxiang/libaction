project('libaction', 'cpp',
        default_options : ['cpp_std=c++11'],
        version : '1.0')

deptensorflow = dependency('tensorflow-lite')

subdir('demo')
subdir('include')

pkg = import('pkgconfig')
pkg.generate(version : meson.project_version(),
             name : 'libaction',
             filebase : 'libaction',
             description : 'Libaction')
