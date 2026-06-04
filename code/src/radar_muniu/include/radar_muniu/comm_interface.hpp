#ifndef RADAR_MUNIU__COMM_INTERFACE_HPP_
#define RADAR_MUNIU__COMM_INTERFACE_HPP_

#include <vector>
#include <string>
#include <functional>
#include <cstdint>
#include <vector> 


namespace radar_muniu
{
// Communication data callback function (receives binary data)
using DataCallback = std::function<void(uint32_t id, const std::vector<uint8_t> &)>;

// Abstract class for communication interface
class CommInterface
{
public:
  virtual ~CommInterface() = default;

  // Initialize communication (e.g., bind port, open CAN device)
  virtual bool init() = 0;

  // Start receiving data (asynchronous/threaded reception, data returned via callback)
  virtual void start_receive(const DataCallback & callback) = 0;

  // Send data (optional, used for radar configuration)
  virtual bool send(uint32_t id, const std::vector<uint8_t> & data) = 0;

  // Close communication
  virtual void close() = 0;
};

}  // namespace radar_muniu

#endif  // RADAR_MUNIU__COMM_INTERFACE_HPP_