#include "scancontext/terminator.h"
#include <iostream>

Terminator::Terminator(int _maxWaitPacketsForNextPacket, double _fps)
    : lastPacketTime(std::chrono::system_clock::now()),
      maxWaitPacketsForNextPacket(_maxWaitPacketsForNextPacket), fps(_fps),
      shutdown(false), numPackets(0) {}

void Terminator::newPacket() {
  std::lock_guard<std::mutex> lock(mutex);
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - lastPacketTime;
  fps = 0.4 * fps + 0.6 / std::max(0.01, diff.count());
  lastPacketTime = now;
  ++numPackets;
}

bool Terminator::quit() {
  std::lock_guard<std::mutex> lock(mutex);
  if (shutdown)
    return true;
  if (maxWaitPacketsForNextPacket < 0) {
    return false;
  }
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - lastPacketTime;
  if (diff.count() > maxWaitPacketsForNextPacket / fps && numPackets) {
    std::cout << "Shutdown a node as packet gap " << diff.count()
        << " greater than #waitPackets(" << maxWaitPacketsForNextPacket << ")/fps(" << fps << ").\n";
    shutdown = true;
    return true;
  } else {
    return false;
  }
}
