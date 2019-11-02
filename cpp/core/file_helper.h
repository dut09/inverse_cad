#ifndef CORE_FILE_HELPER_H
#define CORE_FILE_HELPER_H

#include "core/config.h"

void SaveInt(std::ofstream& f, const int val);
const int LoadInt(std::ifstream& f);
void SaveInt(std::ofstream& f, const std::vector<int>& vals);
void LoadInt(std::ifstream& f, std::vector<int>& vals);
void SaveReal(std::ofstream& f, const real val);
const real LoadReal(std::ifstream& f);
void SaveReal(std::ofstream& f, const std::vector<real>& vals);
void LoadReal(std::ifstream& f, std::vector<real>& vals);

// Replace all '\' with '/'.
const std::string RegularizeFilePath(const std::string& path);
// Concatenate folder and file paths.
const std::string AppendFileToPath(const std::string& folder,
                                   const std::string& file_name);
const std::string AppendFolderToPath(const std::string& folder,
                                     const std::string& subfolder);
const std::string GetParentFolder(const std::string path);
void PrepareToCreateFile(const std::string& file_path);
const bool FileExist(const std::string& file_path);

#endif