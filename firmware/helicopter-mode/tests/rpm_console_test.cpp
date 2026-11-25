#include "Arduino.h"
#include "FlightData.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
using namespace heli;
UARTClass Serial, Serial2;
namespace heli {
float txRate() {
  return 80;
}
} // namespace heli

static uint32_t now = 0;
static FlightData flight;
static void service(unsigned ms) {
  for (unsigned i = 0; i < ms; i++) {
    console(flight);
    logTelemetry(flight, now++);
  }
}
static std::string send(const char *line) {
  Serial.tx.clear();
  Serial.rx = std::string(line) + "\r\n";
  service(300);
  assert(Serial.rx.empty());
  return Serial.tx;
}
static void contains(const std::string &output, const char *text) {
  if (output.find(text) == std::string::npos) {
    std::fprintf(stderr, "Missing %s in %s\n", text, output.c_str());
    std::abort();
  }
}
int main() {
  flight.profile = Profile::RpmTuning;
  flight.gate.state = State::Off;
  startTelemetry(flight.profile);
  service(800);
  contains(Serial.tx, "Morpheus | RPM PIDF");
  contains(Serial.tx, "ESC L/R");
  contains(send("stream off"), "paused");
  Serial.tx.clear();
  Serial2.tx.clear();
  service(250);
  assert(Serial.tx.empty());
  assert(!Serial2.tx.empty());
  contains(send("help"), "SET CAP");
  contains(send("status"), "Coefficients are zero");
  contains(send("report"), "No run");
  contains(send("ffest 25 50"), "FR = 0.5");
  assert(flight.tuning.rpmF == 0);
  contains(send("SET FR 0.5"), "FR=0.50000");
  contains(send("SET P 0.2"), "P=0.20000");
  contains(send("SET D 1e-9"), "D=");
  contains(send("export"), "constexpr Gains rpmGains");
  auto exported = send("export");
  auto start = exported.find("rpmGains = {");
  assert(start != exported.npos);
  auto d = exported.find(',', exported.find(',', start) + 1) + 1;
  assert(float(std::strtod(exported.c_str() + d, nullptr)) == flight.tuning.rpm.kd);
  contains(send("SET CAP 10"), "CAP = 10");
  contains(send("SET TARGET 40"), "TARGET = 40");
  for (const auto &cmd : {"SET CAP 65", "SET CAP 0", "SET CAP 1.5", "SET TARGET 75", "SET TARGET 20",
                          "SET TARGET NAN", "SET TARGET INF", "SET TARGET 40z"})
    contains(send(cmd), "range");
  contains(send("SET P 4 EXTRA"), "Too many");
  assert(flight.tuning.rpm.kp == 0.2f && flight.tuning.rpmEscCap == 10 &&
         flight.tuning.rpmSetpoint == 40);
  contains(send("RATE 21"), "1..20");
  contains(send("RATE 2"), "Hz = 2");
  flight.gate.state = State::Starting;
  flight.rotor.rpm = 27;
  flight.out.leftEsc = 8;
  flight.out.rightEsc = 10;
  service(20);
  contains(send("set p 7"), "CH6 off");
  contains(send("set target 49"), "CH6 off");
  contains(send("report"), "peak ESC byte 10");
  contains(send("report"), "target RPM 40");
  flight.gate.state = State::Faulted;
  flight.gate.reason = Fault::Overspeed;
  service(10);
  contains(send("report"), "stop overspeed");
  flight.gate.state = State::Off;
  service(50);
  contains(send("report"), "stop overspeed");
  contains(send("view rpm"), "OK");
  service(600);
  contains(Serial.tx, "rpm_error:");
  contains(send("monitor"), "Readable");
  service(600);
  contains(Serial.tx, "P/I/D/F");
  contains(send("gaINS"), "FR 0.500000");
  puts("Passed RPM serial commands through the buffered Monitor, Plotter and OpenLog paths.");
}
