#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cerr << "usage: gz_config_publisher TOPIC\n";
    return 2;
  }

  gz::transport::Node node;
  auto publisher =
      node.Advertise<gz::msgs::StringMsg>(std::string(argv[1]));
  if (!publisher)
  {
    std::cerr << "failed to advertise " << argv[1] << "\n";
    return 3;
  }

  // Keep discovery cost out of the first dashboard command.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  std::string payload;
  while (std::getline(std::cin, payload))
  {
    gz::msgs::StringMsg message;
    message.set_data(payload);
    publisher.Publish(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    publisher.Publish(message);
    std::cout << "ok\n" << std::flush;
  }
  return 0;
}
