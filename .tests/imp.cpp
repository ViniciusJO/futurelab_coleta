class Operation {
  public:
    int add(int x, int y) {
      return x + y + 1;
    }
    static int addStatic(int x, int y) {
      return x + y + 2;
    }
};
extern "C" {
  int add(int x, int y) { return x + y; }
  int add1(int x, int y) { Operation o{}; return o.add(x, y); }
  int add2(int x, int y) { return Operation::addStatic(x, y); }
  int add3(int x, int y) { return [](int a, int b) { return a + b + 3; }(x,y); }
}
