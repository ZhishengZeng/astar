/*
 * @Author: Zhisheng Zeng
 * @Date: 2021-09-28 16:04:12
 * @Description:
 * @LastEditors: Zhisheng Zeng
 * @LastEditTime: 2021-09-28 16:11:53
 * @FilePath: /astar/tests/test.cpp
 */
#include <iostream>

using namespace std;

class A
{
 public:
  virtual void print() { cout << "this is A" << endl; }
};
class B : public A
{
 public:
  void print() { cout << "this is B" << endl; }
};

int main()
{
  A* a1 = new A();
  a1->print();
  A* a2 = new B();
  a2->print();
  B* b2 = new B();
  b2->print();

  return 0;
}
