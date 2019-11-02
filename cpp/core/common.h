#ifndef CORE_COMMON_H
#define CORE_COMMON_H

#include "core/config.h"

const real ToReal(const double v);
const double ToDouble(const real v);

// Colorful print.
const std::string GreenHead();
const std::string RedHead();
const std::string CyanHead();
const std::string GreenTail();
const std::string RedTail();
const std::string CyanTail();
// Use return_code = -1 unless you want to customize it.
void PrintError(const std::string& message, const int return_code = -1);
void PrintWarning(const std::string& message);
void PrintSuccess(const std::string& message);

// Timing.
void Tic();
void Toc(const std::string& message);

// String helper.
const std::vector<std::string> SplitString(const std::string& str, const char separator = ' ');

// Error check.
void CheckError(const bool condition, const std::string& error_message);

#endif
