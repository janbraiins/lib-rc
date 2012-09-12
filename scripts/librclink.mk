# Description: provides configurations for building external
# sources with rclink library. The current project configuration has to
# define the following elements:
#
# - CONFIG_LIBRCLINK_DIR - top level directory that contains the glcd
# source tree
#
CFLAGS += $(addprefix -I, $(CONFIG_LIBRCLINK_DIR))


