# Description: provides configurations for building external
# sources with lib-rc library. The current project configuration has to
# define the following elements:
#
# - CONFIG_LIB_RC_DIR - top level directory that contains the glcd
# source tree
#
CFLAGS += $(addprefix -I, $(abspath $(CONFIG_LIB_RC_DIR)))


