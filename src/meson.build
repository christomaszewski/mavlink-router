subdir('common')

exe_mavlink_router = executable('mavlink-routerd',
        [
         'autolog.cpp',
         'binlog.cpp',
         'dedup.cpp',
         'endpoint.cpp',
         'git_version.cpp',
         'logendpoint.cpp',
         'logwriter.cpp',
         'main.cpp',
         'mainloop.cpp',
         'pollable.cpp',
         'timeout.cpp',
         'ulog.cpp',
         'tlog.cpp',
         version_h,
        ],
        include_directories: [src_inc, mavlink_inc],
        dependencies: [
                dep_math,
                dep_rt,
                dep_thread,
        ],
        link_with: libcommon_private,
        install: true,
)

if dep_gtest.found()
  exe_mainloop_test = executable('mainloop_test',
          [
            'autolog.cpp',
            'binlog.cpp',
            'dedup.cpp',
            'endpoint.cpp',
            'logendpoint.cpp',
            'logwriter.cpp',
            'mainloop.cpp',
            'mainloop_test.cpp',
            'pollable.cpp',
            'timeout.cpp',
            'ulog.cpp',
            'tlog.cpp',
            version_h,
          ],
          include_directories: [src_inc, mavlink_inc],
          # gtest may be compiled from distro sources (/usr/src/gtest) as part of this
          # target, so the project's strict warning set reaches gtest's own code and fails
          # -Werror builds (observed: float-equal in gtest-printers.h, suggest-noreturn in
          # gtest-death-test.cc, deprecation warnings). Relax the test targets only; the
          # production binary keeps the full warning set.
          cpp_args: [
            '-Wno-float-equal',
            '-Wno-suggest-attribute=noreturn',
            '-Wno-deprecated-declarations',
            '-Wno-missing-declarations',
          ],
          dependencies : [
            dep_gtest,
            dep_rt,
            dep_thread,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('mainloop', exe_mainloop_test)

  exe_endpoints_test = executable('endpoints_test',
          [
            'autolog.cpp',
            'binlog.cpp',
            'dedup.cpp',
            'endpoint.cpp',
            'endpoints_test.cpp',
            'logendpoint.cpp',
            'logwriter.cpp',
            'mainloop.cpp',
            'pollable.cpp',
            'timeout.cpp',
            'ulog.cpp',
            'tlog.cpp',
            version_h,
          ],
          include_directories: [src_inc, mavlink_inc],
          # gtest may be compiled from distro sources (/usr/src/gtest) as part of this
          # target, so the project's strict warning set reaches gtest's own code and fails
          # -Werror builds (observed: float-equal in gtest-printers.h, suggest-noreturn in
          # gtest-death-test.cc, deprecation warnings). Relax the test targets only; the
          # production binary keeps the full warning set.
          cpp_args: [
            '-Wno-float-equal',
            '-Wno-suggest-attribute=noreturn',
            '-Wno-deprecated-declarations',
            '-Wno-missing-declarations',
          ],
          dependencies : [
            dep_gtest,
            dep_rt,
            dep_thread,
          ],
          link_with: libcommon_private,
          install: false,
  )
  test('endpoints', exe_endpoints_test)
endif
