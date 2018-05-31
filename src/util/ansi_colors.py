# Mabel Zhang
# 3 Sep 2015
#
# Refactored from tactile_collect weights_collect.py
#

# From http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
# ANSI colors https://gist.github.com/chrisopedia/8754917
#   These ones here fall into the "High Intensity" category
class ansi_colors:
  OKCYAN = '\033[96m'
  OKGREEN = '\033[92m'  # green
  WARNING = '\033[93m'  # yellow
  FAIL = '\033[91m'  # red

  ENDC = '\033[0m'

  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'

  BLUE = '\033[94m'
  MAGENTA = '\033[95m'  # magenta

  LOW_RED = '\033[31m'
  LOW_GREEN = '\033[32m'
  LOW_BLUE = '\033[34m'

