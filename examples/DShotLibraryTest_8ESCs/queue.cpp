
#include <cppQueue.h>

#define IMPLEMENTATION FIFO
#define MAX_TELEMETRY_REQUESTS 8

class Queue : public cppQueue {
 public:
  Queue() : cppQueue(sizeof(uint8_t[16]), MAX_TELEMETRY_REQUESTS, IMPLEMENTATION){};
};
