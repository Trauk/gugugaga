#pragma once
#include <string>
#include <iostream>

class Student 
{
private:
    std::string name;
    int score;

public:
    Student(string name, int score);
    void printInfo() const;
    void setScore(int newScore);
    int getScore() const;
};
