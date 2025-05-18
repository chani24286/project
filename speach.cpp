#include "speach.h"
#include <cstdio>
#include <iostream>
#include "global.h"
#include <stdexcept>
#include <string>
#include <array>
#include <vector>
#include <sstream>
std::string output;
std::string pathTTS = "python \"C:\\Users\\User\\Documents\\projectC\\tts.py\"";
std::string pathSTT = "python \"C:\\Users\\User\\Documents\\projectC\\speech-recognition\\speechToText.py\"";
void function(){
	while ( findWord("sense")) {
		std::string wellcome = "Hi, I am here to help you. If you want to go somewhere, just tell me and say go. for another service say other";
		std::string command = pathTTS +"/"+ wellcome+"/";
		int result = system(command.c_str());
		if(findWord("go")){
			//threads KF,LIDAR,GPS,pedometer, camera
		}
	}

}
bool findWord (std::string str){
	std::string output = exec(pathSTT.c_str());
	std::cout << "result: " << output << std::endl;
		// Trim leading and trailing whitespace
		output.erase(0, output.find_first_not_of(" \n\r\t"));
	output.erase(output.find_last_not_of(" \n\r\t") + 1);

	// Check if the text starts with "sens"
	if (output.rfind(str, 0) == 0) { // rfind with 0 checks if it starts with "sens"
		return true;
	}
	return false;
}