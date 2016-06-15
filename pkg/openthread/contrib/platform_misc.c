#include <openthread-config.h>
#include <openthread.h>

#include <platform/misc.h>

otPlatResetReason otPlatGetResetReason(otInstance *aInstance) {
  return kPlatResetReason_Unknown;
}
