#ifndef __parameter_persist_h
#define __parameter_persist_h

#include <vector>
#include <Preferences.h>

class ParameterPersist
{
public:
  ParameterPersist();
  static bool setup(bool readonly=false);

  static std::vector<char *> persist_list;
  static bool persist_all; // defaults to true

  static bool readonly; // defaults to false

};

#endif // __parameter_persist_h
