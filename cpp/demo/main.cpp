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
            CheckError(word_num >= 15 && word_num % 3 == 0,
                "The 'extrude' command requires at least 14 inputs.");
            // extrude <face name> <x y z> <x y z> ... <x y z> <dx, dy dz> <+|->.
            const std::string face_name = words[1];
            const int poly_dof = static_cast<int>((word_num - 6) / 3);
            std::vector<Vector3r> polygon(poly_dof);
            for (int i = 0; i < poly_dof; ++i) {
                for (int j = 0; j < 3; ++j) {
                    polygon[i](j) = StrToReal(words[2 + i * 3 + j]);
                }
            }
            Vector3r dir(StrToReal(words[2 + 3 * poly_dof]),
                StrToReal(words[2 + 3 * poly_dof + 1]),
                StrToReal(words[2 + 3 * poly_dof + 2]));
            CheckError(words[word_num - 1].size() == 1u, "The last input has to be a char.");
            const char op = words[word_num - 1][0];
            CheckError(op == '+' || op == '-', "The boolean operator has to be either + or -.");
            scene.Extrude(face_name, polygon, dir, op);
        } else if (command_type == "help") {
            // TODO.
        }
    }
    return 0;
}