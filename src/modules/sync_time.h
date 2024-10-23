#ifndef __time_sync_h
#define __time_sync_h

#define SERVICE_SYNC_TIME "/sync_time"

class SyncTime {
public:
  SyncTime();
  static bool setup();
};

#endif  // __time_sync_h
