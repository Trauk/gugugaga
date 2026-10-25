#pragma once
#include <string>
#include <iostream>
using namespace std;

class Student 
{
private:
    string name;
    int score;

public:
    Student(string name, int score);
    void printInfo() const;
    void setScore(int newScore);
    int getScore() const;
};
