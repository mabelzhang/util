// Mabel Zhang
// 30 Dec 2016
//
// Copied from triangle_sampling sample_pcl.cpp
//
// For printing colored text in terminal.
//

#ifndef ANSI_COLORS_H
#define ANSI_COLORS_H

// Make some colors to print info
// From http://stackoverflow.com/questions/287871/print-in-terminal-with-colors-using-python
//   ANSI colors https://gist.github.com/chrisopedia/8754917
//   C++ http://stackoverflow.com/questions/7414983/how-to-use-the-ansi-escape-code-for-outputting-colored-text-on-console

static const char * HEADER = "\033[95m";  // magenta
static const char * OKCYAN = "\033[96m";
static const char * OKGREEN = "\033[92m";  // green
static const char * WARN = "\033[93m";  // yellow
static const char * FAIL = "\033[91m";  // red
static const char * ENDC = "\033[0m";
static const char * BOLD = "\033[1m";
static const char * UNDERLINE = "\033[4m";

static const char * BLUE = "\033[94m";

static const char * LOW_RED = "\033[31m";  // can use as OK RED
static const char * LOW_GREEN = "\033[32m";  // can use as OK GREEN
static const char * LOW_BLUE = "\033[34m";  // can use as OK BLUE

#endif

