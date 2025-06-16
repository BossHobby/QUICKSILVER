#include "dma.h"

#include <string.h>

#include "core/project.h"
#include "core/target.h"
#include "util/cbor_helper.h"
#include "util/util.h"

dma_device_t dma_stream_map[DMA_STREAM_MAX];

#ifndef STM32F4
bool dma_allocate_stream(dma_device_t device, uint32_t request_or_channel, resource_tag_t tag) {
  // Check if already allocated for this device
  if (target.dma[device].dma != DMA_STREAM_INVALID) {
    return true;
  }
  
  // Find first available stream
  for (uint32_t i = 1; i < DMA_STREAM_MAX; i++) {
    if (dma_stream_map[i] == DMA_DEVICE_INVALID) {
      dma_stream_map[i] = device;

      target.dma[device].dma = (dma_stream_t)i;
#if defined(STM32G4) || defined(STM32H7) || defined(AT32)
      target.dma[device].request = request_or_channel;
#else
      target.dma[device].channel = request_or_channel;
#endif
      target.dma[device].tag = tag;

      return true;
    }
  }

  return false;
}
#endif

cbor_result_t cbor_decode_dma_device_t(cbor_value_t *dec, dma_device_t *d) {
  const uint8_t *name;
  uint32_t name_len;

  cbor_result_t res = CBOR_OK;
  CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &name, &name_len));

#define DMA_DEVICE(member)                       \
  if (buf_equal_string(name, name_len, #member)) \
    *d = DMA_DEVICE_##member;                    \
  else
  DMA_DEVICES
#undef DMA_DEVICE
  *d = DMA_DEVICE_INVALID;

  return res;
}

cbor_result_t cbor_encode_dma_device_t(cbor_value_t *enc, const dma_device_t *d) {
  cbor_result_t res = CBOR_OK;

  switch (*d) {
#define DMA_DEVICE(_name)                                 \
  case DMA_DEVICE_##_name:                                \
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #_name)); \
    break;
    DMA_DEVICES
#undef DMA_DEVICE
  default:
    break;
  }

  return res;
}

cbor_result_t cbor_decode_dma_stream_t(cbor_value_t *dec, dma_stream_t *d) {
  const uint8_t *name;
  uint32_t name_len;
  cbor_result_t res = cbor_decode_tstr(dec, &name, &name_len);

#define DMA_STREAM(_port, _stream)                                       \
  if (buf_equal_string(name, name_len, "DMA" #_port "_STREAM" #_stream)) \
    *d = DMA##_port##_STREAM##_stream;                                   \
  else
  DMA_STREAMS
#undef DMA_STREAM
  *d = DMA_STREAM_INVALID;

  return res;
}

cbor_result_t cbor_encode_dma_stream_t(cbor_value_t *enc, const dma_stream_t *d) {
  cbor_result_t res = CBOR_OK;

  switch (*d) {
#define DMA_STREAM(_port, _stream)                                                 \
  case DMA##_port##_STREAM##_stream:                                               \
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "DMA" #_port "_STREAM" #_stream)); \
    break;
    DMA_STREAMS
#undef DMA_STREAM
  default:
    break;
  }

  return res;
}

cbor_result_t cbor_decode_target_dma_array(cbor_value_t *dec, target_dma_t (*dma)[DMA_DEVICE_MAX]) {
  cbor_result_t res = CBOR_OK;

  memset(dma, 0, DMA_DEVICE_MAX * sizeof(target_dma_t));
  memset(dma_stream_map, 0, DMA_STREAM_MAX * sizeof(dma_device_t));

  cbor_container_t map;
  CBOR_CHECK_ERROR(res = cbor_decode_map(dec, &map));
  for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
    dma_device_t dev = DMA_DEVICE_INVALID;
    CBOR_CHECK_ERROR(res = cbor_decode_dma_device_t(dec, &dev));
    if (dev == DMA_DEVICE_INVALID) {
      cbor_decode_skip(dec);
      continue;
    }

    target_dma_t *d = &(*dma)[dev];
    CBOR_CHECK_ERROR(res = cbor_decode_target_dma_t(dec, d));
    dma_stream_map[d->dma] = dev;
  }

  return res;
}

cbor_result_t cbor_encode_target_dma_array(cbor_value_t *enc, const target_dma_t (*dma)[DMA_DEVICE_MAX]) {
  cbor_result_t res = CBOR_OK;
  CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

  for (uint32_t i = 0; i < DMA_DEVICE_MAX; i++) {
    const target_dma_t *d = &(*dma)[i];
    if (d->dma == DMA_STREAM_INVALID)
      continue;

    CBOR_CHECK_ERROR(res = cbor_encode_dma_device_t(enc, (const dma_device_t *)&i));
    CBOR_CHECK_ERROR(res = cbor_encode_target_dma_t(enc, d));
  }

  return cbor_encode_end_indefinite(enc);
}