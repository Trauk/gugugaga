#include "student.hpp"
using namespace std;

Student::Student(string n, int s) : name(n), score(s) {}

void Student::printInfo() const {
    cout << "姓名: " << name << ", 成绩: " << score << std::endl;
}

void Student::setScore(int newScore) {
    score = newScore;
    cout << "成绩已修改为: " << score << std::endl;
}

int Student::getScore() const {
    return score;
}
