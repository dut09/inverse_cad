#include "core/config.h"
#include "core/common.h"
#include "core/scene.h"

int main() {
    Scene scene;

    // Interactive examples.
    // Following the steps below to try reconstructing a model.
    // - load <nef3 file>                   # Load the scene from files.
    // - load_target <nef3 file>            # Load a b-rep file that describes the model you want to reconstruct.
    // - ls                                 # Show the target b-rep information: vertices, edges, and facets.
    // - extrude ...                        # Use this command iteratively to reconstruct the model.
    // - save <off file|nef3 file>          # Save the scene into files.
    // - exit
    // Some other useful functions:
    // - convert <off file> <nef3 file>     # Convert a mesh to b-rep.
    // - convert <nef3 file> <off file>     # Convert b-rep to a mesh.
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
        } else if (command_type == "load") {
            CheckError(word_num == 2, "This command needs 1 input.");
            const std::string file_name = words[1];
            scene.LoadScene(file_name);
        } else if (command_type == "load_target") {
            CheckError(word_num == 2, "This command needs 1 input.");
            const std::string file_name = words[1];
            scene.LoadTarget(file_name);
        } else if (command_type == "ls") {
            CheckError(word_num == 2, "This command needs 1 input.");
            CheckError(words[1].size() >= 1u, "Names need to have at least 1 character.");
            if (words[1] == "v") {
                // List all vertex names and positions.
                scene.ListAllVertices();
            } else if (words[1] == "e") {
                // List all edges and two endpoints.
                scene.ListAllEdges();
            } else if (words[1] == "f") {
                // List all faces and edges, organized in a loop (or loops). Sample output:
                // f1 2
                // e0 e2 e4
                // e1 e3 e6 e5
                // This face f1 has two loops. The first loop has three edges (e0, e2, and e3) and
                // the second loop has four edges (e1, e3, e6, e5).
                scene.ListAllFaces();
            } else {
                CheckError(false, "ls does not recognize this input argument.");
            }
        } else if (command_type == "extrude") {
            CheckError(word_num >= 14 && (word_num - 2) % 3 == 0,
                "The 'extrude' command requires at least 14 inputs.");
            // extrude <x y z> <x y z> ... <x y z> <dx, dy dz> <+|->.
            const int poly_dof = static_cast<int>((word_num - 5) / 3);
            std::vector<Vector3r> polygon(poly_dof);
            for (int i = 0; i < poly_dof; ++i) {
                for (int j = 0; j < 3; ++j) {
                    polygon[i](j) = StrToReal(words[1 + i * 3 + j]);
                }
            }
            Vector3r dir(StrToReal(words[1 + 3 * poly_dof]),
                StrToReal(words[1 + 3 * poly_dof + 1]),
                StrToReal(words[1 + 3 * poly_dof + 2]));
            CheckError(words[word_num - 1].size() == 1u, "The last input has to be a char.");
            const char op = words[word_num - 1][0];
            CheckError(op == '+' || op == '-', "The boolean operator has to be either + or -.");
            scene.Extrude(polygon, dir, op);
        } else if (command_type == "extrude_ref") {
            CheckError(word_num == 6, "The 'extrude_ref' command requires 5 inputs.");
            const std::string f_name = words[1];
            CheckError(StartsWith(f_name, "f"), "Invalid facet name.");
            const int f_idx = std::stoi(f_name.substr(1));
            const int loop_idx = std::stoi(words[2]);
            
            const std::string v_source = words[3];
            CheckError(StartsWith(v_source, "v"), "Invalid vertex name.");
            const int source_idx = std::stoi(v_source.substr(1));
            const std::string v_target = words[4];
            CheckError(StartsWith(v_target, "v"), "Invalid vertex name.");
            const int target_idx = std::stoi(v_target.substr(1));

            CheckError(words[5].size() == 1u, "The last input has to be a char.");
            const char op = words[5][0];
            CheckError(op == '+' || op == '-', "The boolean operator has to be either + or -.");
            scene.ExtrudeFromRef(f_idx, loop_idx, source_idx, target_idx, op);
        } else if (command_type == "save") {
            CheckError(word_num == 2, "This command needs 1 input.");
            const std::string file_name = words[1];
            scene.SaveScene(file_name);
        } else if (command_type == "convert") {
            CheckError(word_num == 3, "This command nneds 2 inputs.");
            const std::string in_file_name = words[1];
            const std::string out_file_name = words[2];
            scene.Convert(in_file_name, out_file_name);
        } else if (command_type == "help") {
            // TODO.
        }
    }
    return 0;
}