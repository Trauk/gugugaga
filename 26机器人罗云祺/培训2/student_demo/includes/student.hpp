#pragma once
#include <string>
#include <iostream>

class Student 
{
private:
    std :: string name;
    int score;

public:
    std :: Student(string name, int score);
    void printInfo() const;
    void setScore(int newScore);
    int getScore() const;
};
