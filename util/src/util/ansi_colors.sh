#!/bin/bash

# Mabel Zhang
# 19 Oct 2018
#
# ANSI colors in bash scripts.
# Difference between bash and Python is that Python uses \033 for escape
#   character, bash uses \e.
# Ref 8/16 colors: https://misc.flogisoft.com/bash/tip_colors_and_formatting
#
# Usage:
#   Must use -e flag with echo:
#   $ echo -e "$MAGENTA"abc"$ENDC"abc""
#


# 8/16 colors
OKCYAN="\e[96m"
OKGREEN="\e[92m"  # green
WARN="\e[93m"  # yellow
FAIL="\e[91m"  # red

ENDC="\e[0m"

BOLD="\e[m"
UNDERLINE="\e[4m"
# Invert colors
HILIGHT="\e[7m"

LOW_RED="\e[31m"
LOW_GREEN="\e[32m"
LOW_BLUE="\e[34m"

BLUE="\e[94m"
MAGENTA="\e[95m"



# 88/256 colors
#MAGENTA="\e[38;5;165m"

