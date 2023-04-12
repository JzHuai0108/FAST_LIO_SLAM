#ifndef TERMINATOR_H
#define TERMINATOR_H

#include <chrono>
#include <mutex>

class Terminator {
public:
  Terminator(int _maxWaitPacketsForNextPacket = 3, double _fps = 1.0);

  void newPacket();

  bool quit();

  double rate() const {
    return fps;
  }

  void setWaitPacketsForNextPacket(int waitPackets) {
    maxWaitPacketsForNextPacket = waitPackets;
  }

private:
  std::mutex mutex;
  std::chrono::time_point<std::chrono::system_clock> lastPacketTime;

  int maxWaitPacketsForNextPacket;
  double fps;
  bool shutdown;
  int numPackets;
};

#endif // TERMINATOR_H
