#include "core/file_helper.h"
#include <sys/stat.h>
#include "core/common.h"

void SaveInt(std::ofstream& f, const int val) {
    f.write(reinterpret_cast<const char*>(&val), sizeof(int));
}

const int LoadInt(std::ifstream& f) {
    int val = 0;
    f.read(reinterpret_cast<char*>(&val), sizeof(int));
    return val;
}

void SaveInt(std::ofstream& f, const std::vector<int>& vals) {
    const int num = static_cast<int>(vals.size());
    SaveInt(f, num);
    f.write(reinterpret_cast<const char*>(&vals[0]), sizeof(int) * num);
}

void LoadInt(std::ifstream& f, std::vector<int>& vals) {
    const int num = LoadInt(f);
#if SAFETY_CHECK
    if (!(num >= 0)) {
        PrintError("LoadInt: invalid num: " + std::to_string(num));
        exit(0);
    }
#endif
    vals.resize(num);
    f.read(reinterpret_cast<char*>(&vals[0]), sizeof(int) * num);
}

void SaveReal(std::ofstream& f, const real val) {
    const double val_db = ToDouble(val);
    f.write(reinterpret_cast<const char*>(&val_db), sizeof(double));
}

const real LoadReal(std::ifstream& f) {
    double val = 0;
    f.read(reinterpret_cast<char*>(&val), sizeof(double));
    return ToReal(val);
}

void SaveReal(std::ofstream& f, const std::vector<real>& vals) {
    const std::vector<double> vals_db(vals.begin(), vals.end());
    const int num = static_cast<int>(vals_db.size());
    SaveInt(f, num);
    f.write(reinterpret_cast<const char*>(&vals_db[0]), sizeof(double) * num);
}

void LoadReal(std::ifstream& f, std::vector<real>& vals) {
    const int num = LoadInt(f);
#if SAFETY_CHECK
    if (!(num >= 0)) {
        PrintError("LoadReal: invalid num: " + std::to_string(num));
        exit(0);
    }
#endif
    std::vector<double> vals_db(num, 0);
    f.read(reinterpret_cast<char*>(&vals_db[0]), sizeof(double) * num);
    vals.resize(num);
    for (int i = 0; i < num; ++i) {
        vals[i] = ToReal(vals_db[i]);
    }
}

const std::string RegularizeFilePath(const std::string& path) {
    std::string new_path = "";
    bool in_slash = false;
    for (const char ch : path) {
        const bool is_slash = (ch == '\\' || ch == '/');
        if (!is_slash) {
            new_path += ch;
            in_slash = false;
        } else if (!in_slash) {
            new_path += '/';
            in_slash = true;
        }
    }
    return new_path;
}

const std::string AppendFileToPath(const std::string& folder, const std::string& file_name) {
    return RegularizeFilePath(folder + "/" + file_name);
}

const std::string AppendFolderToPath(const std::string& folder, const std::string& subfolder) {
    return RegularizeFilePath(folder + "/" + subfolder);
}

const std::string GetParentFolder(const std::string path) {
    std::string reg_path = RegularizeFilePath(path);
    // Stop if it is already the root.
    if (reg_path == "/") return reg_path;
    // Ignore the trailing slash.
    if (reg_path.back() == '/')
        reg_path = reg_path.substr(0, reg_path.size() - 1);
    const auto idx = reg_path.find_last_of('/');
    if (idx == std::string::npos) return "./";
    else return reg_path.substr(0, idx);
}

void PrepareToCreateFile(const std::string& file_path) {
    const std::string reg_file_path = RegularizeFilePath(file_path);
    const std::size_t found = reg_file_path.rfind("/");
    if (found != std::string::npos) {
        const std::string folder_name = reg_file_path.substr(0, found + 1);
        size_t pos = 0;
        do {
            pos = folder_name.find_first_of('/', pos + 1);
            mkdir(folder_name.substr(0, pos).c_str(), S_IRWXU);
        } while (pos != std::string::npos);
    }
    std::ofstream fout(reg_file_path, std::ios::out);
    if (!fout.is_open()) {
        std::cerr << RedHead() << "PrepareToCreateFile: did not create file " << reg_file_path
                  << " successfully." << RedTail() << std::endl;
        exit(-1);
    }
    fout.close();
}

const bool FileExist(const std::string& file_path) {
    std::ifstream fin;
    fin.open(file_path);
    const bool exists = fin.good();
    fin.close();
    return exists;
}

const std::string GetFileExtension(const std::string& file_name) {
    const std::vector<std::string> name_and_ext = SplitString(file_name, '.');
    CheckError(name_and_ext.size() == 2u, "File name must have an extension.");
    return name_and_ext[1];
}