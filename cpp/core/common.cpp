#include "core/common.h"

const real ToReal(const double v) {
    return static_cast<real>(v);
}

const double ToDouble(const real v) {
    return static_cast<double>(v);
}

const real StrToReal(const std::string& v) {
    return ToReal(std::stod(v));
}

const std::string GreenHead() {
    return "\x1b[6;30;92m";
}

const std::string RedHead() {
    return "\x1b[6;30;91m";
}

const std::string CyanHead() {
    return "\x1b[6;30;96m";
}

const std::string GreenTail() {
    return "\x1b[0m";
}

const std::string RedTail() {
    return "\x1b[0m";
}

const std::string CyanTail() {
    return "\x1b[0m";
}

void PrintError(const std::string& message, const int return_code) {
    std::cerr << RedHead() << message << RedTail() << std::endl;
    throw return_code;
}

void PrintWarning(const std::string& message) {
    std::cout << CyanHead() << message << CyanTail() << std::endl;
}

void PrintSuccess(const std::string& message) {
    std::cout << GreenHead() << message << GreenTail() << std::endl;
}

// Timing.
static struct timeval t_begin, t_end;

void Tic() {
    gettimeofday(&t_begin, nullptr);
}

void Toc(const std::string& message) {
    gettimeofday(&t_end, nullptr);
    const real t_interval = (t_end.tv_sec - t_begin.tv_sec) + (t_end.tv_usec - t_begin.tv_usec) / 1e6;
    std::cout << CyanHead() << "[Timing] " << message << ": " << t_interval << "s"
              << CyanTail() << std::endl;
}

const std::vector<std::string> SplitString(const std::string& str, const char separator) {
    std::vector<std::string> strs;
    std::string word = "";
    for (const char ch : str) {
        if (ch != separator) word += ch;
        else {
            if (!word.empty()) strs.push_back(word);
            word = "";
        }
    }
    if (!word.empty()) strs.push_back(word);
    return strs;
}

const bool StartsWith(const std::string& str, const std::string& substr) {
    if (str.size() < substr.size()) return false;
    const int n = static_cast<int>(substr.size());
    for (int i = 0; i < n; ++i) {
        if (str[i] != substr[i]) return false;
    }
    return true;
}

const bool EndsWith(const std::string& str, const std::string& substr) {
    if (str.size() < substr.size()) return false;
    const int n = static_cast<int>(substr.size());
    const int m = static_cast<int>(str.size());
    for (int i = 0; i < n; ++i) {
        if (str[m - n + i] != substr[i]) return false;
    }
    return true;
}

void CheckError(const bool condition, const std::string& error_message) {
#ifdef SAFETY_CHECK
    if (!condition) {
        PrintError(error_message);
        exit(0);
    }
#endif
}