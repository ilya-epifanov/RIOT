#include <openthread-config.h>
#include <openthread.h>

#include <platform/settings.h>

void otPlatSettingsInit(otInstance *aInstance) {
}

ThreadError otPlatSettingsBeginChange(otInstance *aInstance) {
  return kThreadError_None;
}

ThreadError otPlatSettingsCommitChange(otInstance *aInstance) {
  return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsAbandonChange(otInstance *aInstance) {
  return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsGet(otInstance *aInstance,
                              uint16_t aKey,
                              int aIndex,
                              uint8_t *aValue,
                              uint16_t *aValueLength) {
  return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsSet(otInstance *aInstance,
                              uint16_t aKey,
                              const uint8_t *aValue,
                              uint16_t aValueLength) {
  return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsAdd(otInstance *aInstance,
                              uint16_t aKey,
                              const uint8_t *aValue,
                              uint16_t aValueLength) {
  return kThreadError_NotImplemented;
}

ThreadError otPlatSettingsDelete(otInstance *aInstance, uint16_t aKey, int aIndex) {
  return kThreadError_NotImplemented;
}

void otPlatSettingsWipe(otInstance *aInstance) {
}
