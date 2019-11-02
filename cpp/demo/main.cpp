#include "core/config.h"
#include "core/common.h"
#include "core/scene.h"

int main() {
    Scene scene;

    // Interactive examples.
    while (true) {
        // Get commands.
        std::string command;
        std::getline(std::cin, command);
        std::vector<std::string> words = SplitString(command);
        const int word_num = static_cast<int>(words.size());
        if (word_num == 0) continue;
        const std::string command_type = words[0];
        if (command_type == "exit") {
            break;
        } else if (command_type == "save") {
            CheckError(word_num == 2, "The 'save' command requires 1 inputs.");
            const std::string file_name = words[1];
            scene.SaveScene(file_name);
        } else if (command_type == "load") {
            CheckError(word_num == 2, "The 'load' command requires 1 inputs.");
            const std::string file_name = words[1];
            scene.LoadScene(file_name);
        } else if (command_type == "ls") {
            CheckError(word_num == 1, "The 'ls' command does not accept any input.");
            scene.ShowTopologyInformation();
        } else if (command_type == "extrude") {
            CheckError(word_num >= 14 && (word_num - 2) % 3 == 0,
                "The 'extrude' command requires at least 13 inputs.");
            // TODO.
        } else if (command_type == "bool") {
            CheckError(word_num == 4 && words[1].size() == 1u, "The 'bool' command requires 4 inputs.");
            const char op = words[1][0];
            const std::string& ob0 = words[2];
            const std::string& ob1 = words[3];
            scene.DoBooleanOperation(op, ob0, ob1);
        } else if (command_type == "help") {
            // TODO.
        }
    }
    return 0;
}