#include <iostream>

class A {
public:
  A() {
    _data = new double[3];
    _data[0] = 1.0;
    _data[1] = 2.0;
    _data[2] = 3.0;
    std::cout << "A constructor" << std::endl;
  }
  ~A() {
    delete[] _data;
    std::cout << "A destructor" << std::endl;
  }
  void printInfo() {
    printf("A printInfo: %f\n", _data[2]);
  }
  double* _data = nullptr;
};

class C { 
public:
  C(A* a):_a(a) {
    std::cout << "C constructor" << std::endl;
  }
  ~C() {
    std::cout << "C destructor" << std::endl;
  }
  void printInfo() {
    std::cout << "C printInfo" << std::endl;
    _a->printInfo();
  }
  A* _a;
};

class B {
public:
  B():_c(_a) {
    std::cout << "B constructor" << std::endl;
  }
  ~B() {
    std::cout << "B destructor" << std::endl;
  }
  void printInfo() {
    _a = new A();
    std::cout << "B printInfo" << std::endl;
  }
  A* _a;
  C _c;
};



int main() {
  // A* a = new A();
  A* a = nullptr;
  a->printInfo();
  B b;
  b.printInfo();
  b._c.printInfo();

  std::cout << "Hello, World!" << std::endl;
  return 0;
}