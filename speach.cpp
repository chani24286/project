#include "speach.h"
#include <cstdio>
#include <iostream>
#include "global.h"
#include <stdexcept>
#include <string>
#include <array>
#include <vector>
#include <sstream>
#include "graph.h"

std::string output;

std::string pathSTT = "python \"C:\\Users\\User\\Documents\\projectC\\speech-recognition\\speechToText.py\"";
std::string  speachAndSplit(NevigationObject &me){
	
	if (findWord("send")) {
		std::string wellcome = "Hi, I am here to help you. If you want to go somewhere, just tell me and say go. for another service say other";
		sayIt(wellcome);
		if(findWord("go")){
			std::string location = getWord();
			std::cout << "location: " << location << std::endl;
			createGraph(location, me);
			return "going";
		}
	}

}
void sayIt(std::string str) {
	std::string scriptPath = "C:\\Users\\User\\Documents\\projectC\\tts.py";
	std::string command = "python \"" + scriptPath + "\" \"" + str + "\"";
	int result = system(command.c_str());
}
bool findWord (std::string str){
	std::string output = exec(pathSTT.c_str());
	std::cout << "result: " << output << std::endl; 
	std::cout << "looking for " << str << std::endl;
		// Trim leading and trailing whitespace
	output.erase(0, output.find_first_not_of(" \n\r\t"));
	output.erase(output.find_last_not_of(" \n\r\t") + 1);
	if (output.rfind(str) != std::string::npos) {
		return true;
	}
	return false;
}
std::string getWord() {
	std::string output = "recorded: ";
	 std::string results  = exec(pathSTT.c_str());
	 output += results;
	printing(output);
	// Trim leading and trailing whitespace
	output.erase(0, output.find_first_not_of(" \n\r\t"));
	output.erase(output.find_last_not_of(" \n\r\t") + 1);
	return output;
}