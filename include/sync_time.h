#ifndef __time_sync_h
#define __time_sync_h

class SyncTime
{
public:
  SyncTime();
  static bool setup(const char *service_name = "/sync_time");
};

#endif // __time_sync_h
