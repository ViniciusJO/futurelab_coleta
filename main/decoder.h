#ifndef __DECODER_H__
#define __DECODER_H__
#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include <stdint.h>
float rpm_decoder(uint8_t bytes[2]);
float speed_decoder(uint8_t bytes[1]);
float fuel_rate_decoder(uint8_t bytes[2]);
float pedal_pos_decoder(uint8_t bytes[1]);

#ifdef __cplusplus
}
#endif//__cplusplus
#endif//__DECODER_H__

//#define DECODER_IMPLEMENTATIONS
#ifdef DECODER_IMPLEMENTATIONS
#ifndef __DECODER_IMP__
#define __DECODER_IMP__
#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

float rpm_decoder(uint8_t bytes[2]) {
  int A = bytes[0];
  int B = bytes[1];
  return ((256 * A) + B) / 4.f;
}

float speed_decoder(uint8_t bytes[1]) {
  return bytes[0];
}

float fuel_rate_decoder(uint8_t bytes[2]) {
  int A = bytes[0];
  int B = bytes[1];
  return ((256 * A) + B) / 20.f;
}

float pedal_pos_decoder(uint8_t bytes[1]) {
  return bytes[0]*(100.f/255.f);
}

#ifdef __cplusplus
}
#endif//__cplusplus
#endif//__DECODER_IMP__
#undef DECODER_IMPLEMENTATIONS
#endif//DECODER_IMPLEMENTATIONS

