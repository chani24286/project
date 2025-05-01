#define _CRT_SECURE_NO_WARNINGS
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <vector>
#include <sstream>

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd, "r"), _pclose);
    if (!pipe) {
        throw std::runtime_error("popen failed");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
int listen() {
    std::string output;
    try {
        std::cout << "runing speech recognition" << std::endl;
	//	while (true) {
			// Run the Python script to get the speech recognition result
		std::string wellcome = "Hi, I am here to help you. If you want to go somewhere, just tell me and say go. for another service say other";
        std::string command = "python \"C:\\Users\\User\\Documents\\projectC\\tts.py\"" + wellcome;
        int result = system(command.c_str());

	
        output = exec("python \"C:\\Users\\User\\Documents\\projectC\\speech-recognition\\speechToText.py\"");
        std::cout << "result: " << output << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;
        return -1;
    }
    // Trim leading and trailing whitespace
    output.erase(0, output.find_first_not_of(" \n\r\t"));
    output.erase(output.find_last_not_of(" \n\r\t") + 1);

    // Check if the text starts with "sens"
    if (output.rfind("go", 0) == 0) { // rfind with 0 checks if it starts with "sens"
        std::cout << "Input starts with 'other'" << std::endl;
        split(output);
    }
}
    int split(std::string output){
        // Split the text into words
        std::istringstream iss(output);
        std::vector<std::string> words;
        std::string word;
        while (iss >> word) {
            words.push_back(word);
        }


        // Check if the second word is "meet" and a name follows
        if (words.size() >= 3 && words[1] == "meet") {
            std::string personName = words[2]; // The third word is the person's name

            // Run the Python script to save face images with the person's name

            std::string str = "python \"C:\\Users\\User\\Documents\\projectC\\ML\\faceDetection\\faceDetection\\newFace.py\" \"" + personName + "\"";
            char command[256];
            strcpy(command, str.c_str());
            std::string saveOutput = exec(command);
            std::cout << "Started face capturing for: " << personName << std::endl;

        }
		if (words.size() >= 3 && words[1] == "go") {
			std::string destination = words[2]; // The third word is the destination
            //להפעיל קבלת מסלול עם היעד.

            
		}
        return 0;
    }
//while going

   


int main() 
{
    if (listen() == -1) {
        std::cerr << "Error in speech recognition" << std::endl;
        return -1;
    }

    return 0;
}
