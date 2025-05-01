#include "Pedometer.h"
#include "GPS.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
void saveStepLength(float stepLength) {
    std::ofstream outFile("step_length.txt");
    if (outFile.is_open()) {
        outFile << stepLength;  // ���� �� ���� ���� �����
        outFile.close();
        std::cout << "���� ���� ���� ������!" << std::endl;
    }
    else {
        std::cerr << "�� ���� ����� �� �����." << std::endl;
    }
}

int readStepLength() {
    std::ifstream inFile("step_length.txt");
    int stepLength = 0.0;
    if (inFile.is_open()) {
        inFile >> stepLength;
        inFile.close();
    }
    else {
        std::cerr << "cannot read from file" << std::endl;
    }
    return stepLength;
}

// ������� ������ ����� �� �� ���� ������
int calculateDistance(int stepCount, float stepLength) {
    return stepCount * stepLength;  // ���� = ���� ������ * ���� ����
}
int pedometer() {
    return 3;
}
int calculateDistance(int stepCount, float stepLength) {
    return stepCount * stepLength;  // ���� = ���� ������ * ���� ����
}
int pedometer() {
    return 3;
}
int calculateDistance() {
    int stepCount = pedometer();
    float stepLength = readStepLength();  // ����� ����� ���� ����� �����

    if (stepLength > 0) {
        // ����� �����
        float distance = stepLength * stepCount;
        std::cout << distance << std::endl;
		return distance;
    }
    else {
        std::cerr << "not found step length" << std::endl;
		return -1;
    }

}
using namespace std::chrono;
int save() {
    float stepLength;
    //gps----

    auto last_time = steady_clock::now(); // ��� �����
    duration<int> one_second(3);
    
    int i = 0;
    while (i < 5) {
        auto current_time = steady_clock::now();
        if (current_time - last_time >= one_second) {
            last_time = current_time;
            //����� ������ ����� ����
            //����� ���� ����� ����� ����
            //����� ���/���� ����� ������ ����� ������ ����� ����
            i = i + 5;
        }

        std::this_thread::sleep_for(milliseconds(10));  // ����� ���� ��� �� ������ �� �����
    }
    //(������� �����) ������ ����� �� ���� ���� �����
}