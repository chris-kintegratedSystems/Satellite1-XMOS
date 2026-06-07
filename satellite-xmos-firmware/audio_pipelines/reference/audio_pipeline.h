// Copyright 2022-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#ifndef AUDIO_PIPELINE_H_
#define AUDIO_PIPELINE_H_

#include <stdint.h>
#include "app_conf.h"

#define AUDIO_PIPELINE_DONT_FREE_FRAME 0
#define AUDIO_PIPELINE_FREE_FRAME      1

void audio_pipeline_init(
        void *input_app_data,
        void *output_app_data);

void audio_pipeline_input(
        void *input_app_data,
        int32_t **input_audio_frames,
        size_t ch_count,
        size_t frame_count);

int audio_pipeline_output(
        void *output_app_data,
        int32_t **output_audio_frames,
        size_t ch_count,
        size_t frame_count);

/* [LC-TELE dev.203] Loss-control telemetry readout (instrumentation only; does NOT alter the audio
 * path or any lc_* value). Serializes a per-window peak-hold snapshot of the AGC loss-control state
 * into buf (LC_TELE_NUM_BYTES, little-endian) and resets the peak-hold. Implemented ON_TILE(0) in
 * audio_pipeline_t0.c; read off-chip via the DFU servicer GET_LC_STATE command.
 * Layout (dev.204, 32 bytes LE): [0:2] lc_gain_min*1000 u16 | [2:4] lc_gain_now*1000 u16 |
 *   [4:6] lc_corr_now*1000 u16 | [6:8] lc_t_far_max u16 | [8:10] lc_t_near_now u16 |
 *   [10:14] aec_ref_power_max f32 | [14:18] aec_ref_power_now f32 |
 *   [18:20] lc_corr_min*1000 u16 (onset corr-dip depth) | [20:24] lc_near_power_max f32 |
 *   [24:28] lc_near_power_now f32 | [28:32] lc_near_bg_power_now f32 */
#define LC_TELE_NUM_BYTES 32
void audio_pipeline_get_lc_telemetry(uint8_t *buf);

#endif /* AUDIO_PIPELINE_H_ */
