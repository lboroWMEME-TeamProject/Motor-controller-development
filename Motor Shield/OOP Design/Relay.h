#ifndef myRobot_h
#define myRobot_h

#define RELAY1 7
#define RELAY2 10
#define RELAY3 14


class Relay
{
  public:
  Relay(int,int,int);
  void On();
  void Off();
  
  private:
  int pins[3];
}

#endif
