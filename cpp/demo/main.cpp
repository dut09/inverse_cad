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
            CheckError(word_num == 2, "The 'save' command requires two inputs.");
            const std::string file_name = words[1];
            scene.SaveScene(file_name);
        }
    }
    return 0;
}