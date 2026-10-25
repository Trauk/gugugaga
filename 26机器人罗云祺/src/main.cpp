#include "student.hpp"

int main() 
{
    Student s("张三", 59);
    s.printInfo();
    s.setScore(60);
    s.printInfo();
    return 0;
}
