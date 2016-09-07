"""Scons Tool for building external sources against lib-rc.
   The tool assumes the build configuration is available through
   construction environment variable 'CONFIG'. The following
   configuration elements are required:

   - LIB_RC_DIR - top level directory that contains the source
     tree

   Copyright (c) 2016 Braiins Systems s.r.o.
"""
import os


def generate(env):
    """Set build environment so that this project is also available to
    other projects
    """
    config = env['CONFIG']


    env.Append(CPPPATH=[config.LIB_RC_DIR,
                        # Extend search path for generated files under the build directory
                        os.path.join('#$VARIANT_DIR',
                                     os.path.basename(config.LIB_RC_DIR))])

def exists(env):
    return 1
