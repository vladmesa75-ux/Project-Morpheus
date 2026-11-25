// Host substitute for the Print/UART methods used by the serial console.
// A small TX buffer exercises the same partial-row draining as on the Due.
#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
class Print {
public:
  virtual size_t write(uint8_t byte) = 0;
  size_t write(const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i < n; ++i)
      write(bytes[i]);
    return n;
  }
  void print(const char *s) {
    while (*s)
      write(uint8_t(*s++));
  }
  void print(char *s) {
    print(static_cast<const char *>(s));
  }
  void print(char c) {
    write(uint8_t(c));
  }
  template <class I> void print(I i) {
    print(std::to_string(i).c_str());
  }
  void print(double n, int places = 2) {
    char text[80];
    snprintf(text, sizeof(text), "%.*f", places, n);
    print(text);
  }
  void print(float n, int places = 2) {
    print(double(n), places);
  }
  template <class T> void println(T value) {
    print(value);
    println();
  }
  void println(double value, int digits) {
    print(value, digits);
    println();
  }
  void println() {
    print("\r\n");
  }
};
class UARTClass : public Print {
public:
  std::string rx, tx;
  void begin(unsigned long) {}
  int available() {
    return int(rx.size());
  }
  int read() {
    char c = rx.front();
    rx.erase(0, 1);
    return c;
  }
  int availableForWrite() {
    return 13;
  }
  using Print::write;
  size_t write(uint8_t c) override {
    tx.push_back(char(c));
    return 1;
  }
};
extern UARTClass Serial, Serial2;
