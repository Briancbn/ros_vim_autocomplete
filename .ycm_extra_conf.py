# Copyright 2021 Chen Bainian
# Copyright 2015 Gaël Ecorchard
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Reference:
#   Gist link: https://gist.github.com/galou/92a2d05dd772778f86f2
#   Author: Gaël Ecorchard (2015)
#   License: CC0

import os
import ycm_core

# These are the compilation flags that will be used in case there's no
# compilation database set (by default, one is not set).
# CHANGE THIS LIST OF FLAGS. YES, THIS IS THE DROID YOU HAVE BEEN LOOKING FOR.
# You can get CMake to generate the compilation_commands.json file for you by
# adding:
#   set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# to your CMakeLists.txt file or by once entering
#   catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# or
#   colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# in your shell.

DEFAULT_FLAGS = [
    '-Wall',
    '-Wextra',
    '-Werror',
    # '-Wc++98-compat',
    '-Wno-long-long',
    '-Wno-variadic-macros',
    '-fexceptions',
    '-DNDEBUG',
    # THIS IS IMPORTANT! Without a "-std=<something>" flag, clang won't know
    # which language to use when compiling headers. So it will guess. Badly. So
    # C++ headers will be compiled as C headers. You don't want that so ALWAYS
    # specify a "-std=<something>".
    # For a C project, you would set this to something like 'c99' instead of
    # 'c++11'.
    '-std=c++14',
    # ...and the same thing goes for the magic -x option which specifies the
    # language that the files to be compiled are written in. This is mostly
    # relevant for c++ headers.
    # For a C project, you would set this to 'c' instead of 'c++'.
    '-x',
    'c++',
    '-I',
    '.',
    # include third party libraries
    # '-isystem',
    # '/some/path/include',
]


def GetWorkspaceDir():
    """
    Get the ROS workspace directory path.

    Return this script directory is ROS_WORKSPACE is not set.
    """
    ws_dir = os.environ.get('ROS_WORKSPACE')
    return os.path.dirname(os.path.abspath(__file__)) if ws_dir is None else ws_dir


def GetDefaultRosIncludePaths():
    """
    Return a list of potential include directories.

    The directories are looked for in ros workspace.
    This doesn't work well with the ROS 1 build configured with install.
    """
    ros_ver = os.environ.get('ROS_VERSION')
    includes = []
    # ROS 1
    if ros_ver == '1':
        try:
            from rospkg import RosPack
        except ImportError:
            return []
        rospack = RosPack()
        devel_includes_path = os.path.join(GetWorkspaceDir(),
                                           'devel', 'include')
        if os.path.exists(devel_includes_path):
            includes.append(devel_includes_path)

        for p in rospack.list():
            if os.path.exists(rospack.get_path(p) + '/include'):
                includes.append(rospack.get_path(p) + '/include')

        includes.append('/opt/ros/' + os.environ.get('ROS_DISTRO') + '/include')

    # ROS 2
    elif ros_ver == '2':
        try:
            from ros2pkg.api import get_package_names
            from ament_index_python import get_package_prefix
        except ImportError:
            return []
        for package_name in get_package_names():
            include_path = os.path.join(get_package_prefix(package_name), 'include')
            if os.path.exists(include_path) and include_path not in includes:
                includes.append(include_path)

    return includes


def GetDefaultFlags():
    includes = GetDefaultRosIncludePaths()
    flags = DEFAULT_FLAGS
    for include in includes:
        flags.append('-isystem')
        flags.append(include)
    return flags


def GetCompileCommandsPath(filename):
    """
    Return the directory potentially containing `compilation_commands.json`.

    Return the absolute path to the folder (NOT the file!) containing the
    compile_commands.json file to use that instead of 'flags'. See here for
    more details: http://clang.llvm.org/docs/JSONCompilationDatabase.html.
    The compilation_commands.json for the given file is returned by getting
    the package the file belongs to.
    """
    # Find the corresponding ROS package name for the file.
    # This function works in both ROS1 and ROS2
    try:
        from rospkg import get_package_name
    except ImportError:
        return ''
    pkg_name = get_package_name(filename)
    if not pkg_name:
        return ''

    return os.path.join(GetWorkspaceDir(), 'build', pkg_name)


def GetDatabase(compilation_database_folder):
    if os.path.exists(compilation_database_folder):
        return ycm_core.CompilationDatabase(compilation_database_folder)
    return None


def MakeRelativePathsInFlagsAbsolute(flags, working_directory):
    if not working_directory:
        return list(flags)
    new_flags = []
    make_next_absolute = False
    path_flags = ['-isystem', '-I', '-iquote', '--sysroot=']
    for flag in flags:
        new_flag = flag

        if make_next_absolute:
            make_next_absolute = False
            if not flag.startswith('/'):
                new_flag = os.path.join(working_directory, flag)

        for path_flag in path_flags:
            if flag == path_flag:
                make_next_absolute = True
                break

            if flag.startswith(path_flag):
                path = flag[len(path_flag):]
                new_flag = path_flag + os.path.join(working_directory, path)
                break

        if new_flag:
            new_flags.append(new_flag)
    return new_flags


def Settings(**kwargs):
    if kwargs['language'] != 'cfamily':
        return {}

    filename = kwargs['filename']
    flags = []
    database_dir = GetCompileCommandsPath(filename)

    if os.path.exists(database_dir):
        # Load the compile_commands.json file
        database = ycm_core.CompilationDatabase(database_dir)
        if database:
            # Bear in mind that compilation_info.compiler_flags_ does NOT return a
            # python list, but a "list-like" StringVec object
            compilation_info = database.GetCompilationInfoForFile(filename)
            if not compilation_info:
                flags = GetDefaultFlags()  # Use default flags if there are any loading error.
            else:
                flags = MakeRelativePathsInFlagsAbsolute(
                    compilation_info.compiler_flags_,
                    compilation_info.compiler_working_dir_)
    if not flags:
        flags = GetDefaultFlags()  # Use default flags if there is no flags defined.

    # Force std 17 if not define
    std_exist = False
    for flag in flags:
        std_exist |= '-std=c++' in flag
    if not std_exist:
        flags += ['-std=c++17']

    return {
        'flags': flags,
        'do_cache': True
    }
