#include "scancontext/terminator.h"

Terminator::Terminator(int _maxWaitPacketsForNextPacket, double _fps)
    : lastPacketTime(std::chrono::system_clock::now()),
      maxWaitPacketsForNextPacket(_maxWaitPacketsForNextPacket), fps(_fps),
      shutdown(false), numPackets(0) {}

void Terminator::newPacket() {
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - lastPacketTime;
  fps = 0.4 * fps + 0.6 / diff.count();
  lastPacketTime = now;
  ++numPackets;
}

bool Terminator::quit() {
  if (shutdown)
    return true;
  if (maxWaitPacketsForNextPacket < 0) {
    return false;
  }
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - lastPacketTime;
  if (diff.count() > maxWaitPacketsForNextPacket / fps && numPackets) {
    shutdown = true;
    return true;
  } else {
    return false;
  }
}
