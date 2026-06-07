// Copyright 2022-2024 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* STD headers */
#include <string.h>
#include <stdint.h>
#include <xcore/hwtimer.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "stream_buffer.h"

/* Library headers */
#include "generic_pipeline.h"
#include "aec_api.h"
#include "agc_api.h"
#include "ic_api.h"
#include "ns_api.h"
#include "vnr_features_api.h"
#include "vnr_inference_api.h"

/* App headers */
#include "app_conf.h"
#include "audio_pipeline.h"
#include "audio_pipeline_dsp.h"

#if appconfAUDIO_PIPELINE_FRAME_ADVANCE != 240
#error This pipeline is only configured for 240 frame advance
#endif

#define VNR_AGC_THRESHOLD (0.5)

#if ON_TILE(0)
static ic_stage_ctx_t DWORD_ALIGNED ic_stage_state = {};
static vnr_pred_stage_ctx_t DWORD_ALIGNED vnr_pred_stage_state = {};
static ns_stage_ctx_t DWORD_ALIGNED ns_stage_state = {};
static agc_stage_ctx_t DWORD_ALIGNED agc_stage_state = {};

/* [LC-TELE dev.203] Per-window peak-hold of the AGC loss-control internal state, for off-chip readout
 * via the DFU servicer GET_LC_STATE command. Updated each frame in stage_agc; serialized + cleared in
 * audio_pipeline_get_lc_telemetry(). Instrumentation only - touches no lc_* value and no audio sample.
 * Benign cross-task race (audio task writes, dfu servicer task reads) is acceptable for diagnostics. */
typedef struct {
    float gain_min;     /* min lc_gain over the window  -> ~0.022 means the far-end crush engaged */
    float gain_now;     /* latest lc_gain */
    float corr_now;     /* latest lc_corr_val (vs lc_corr_threshold 0.993) */
    int   t_far_max;    /* max lc_t_far over the window (>0 => far-end detected) */
    int   t_near_now;   /* latest lc_t_near (>0 => near/double-talk detected) */
    float ref_pow_max;  /* max aec_ref_power over the window (far-end energy the AEC saw) */
    float ref_pow_now;  /* latest aec_ref_power */
    /* [LC-TELE dev.204] near-power + corr-dip visibility, to set the dev.205 onset-double-talk lever. */
    float corr_min;     /* MIN lc_corr_val over the window (the onset corr-dip depth) */
    float near_pow_max; /* MAX lc_near_power_est over the window (onset near-power spike) */
    float near_pow_now; /* latest lc_near_power_est */
    float near_bg_now;  /* latest lc_near_bg_power_est (the baseline; bar = lc_near_delta_far_active*this) */
} lc_tele_t;
static lc_tele_t lc_tele = { 1.0f, 1.0f, 0.0f, 0, 0, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };

static void *audio_pipeline_input_i(void *input_app_data)
{
    frame_data_t *frame_data;

    frame_data = pvPortMalloc(sizeof(frame_data_t));
    memset(frame_data, 0x00, sizeof(frame_data_t));

    size_t bytes_received = 0;
    bytes_received = rtos_intertile_rx_len(
            intertile_ctx,
            appconfAUDIOPIPELINE_PORT,
            portMAX_DELAY);

    xassert(bytes_received == sizeof(frame_data_t));

    rtos_intertile_rx_data(
            intertile_ctx,
            frame_data,
            bytes_received);

    return frame_data;
}

static int audio_pipeline_output_i(frame_data_t *frame_data,
                                   void *output_app_data)
{

    return audio_pipeline_output(output_app_data,
                               (int32_t **)frame_data->samples,
                               6,
                               appconfAUDIO_PIPELINE_FRAME_ADVANCE);
}

static void stage_vnr_and_ic(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_IC_AND_VAD
#else
    int32_t DWORD_ALIGNED ic_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    ic_filter(&ic_stage_state.state,
              frame_data->samples[0],
              frame_data->samples[1],
              ic_output);

    vnr_pred_state_t *vnr_pred_state = &vnr_pred_stage_state.vnr_pred_state;
    ic_calc_vnr_pred(&ic_stage_state.state, &vnr_pred_state->input_vnr_pred, &vnr_pred_state->output_vnr_pred);

    float_s32_t agc_vnr_threshold = f32_to_float_s32(VNR_AGC_THRESHOLD);
    frame_data->vnr_pred_flag = float_s32_gt(vnr_pred_stage_state.vnr_pred_state.output_vnr_pred, agc_vnr_threshold);

    ic_adapt(&ic_stage_state.state, vnr_pred_stage_state.vnr_pred_state.input_vnr_pred);

    /* Intentionally ignoring comms ch from here on out */
    memcpy(frame_data->samples[0], ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#if appconfAUDIO_PIPELINE_STORE_IC_AUDIO    
    memcpy(frame_data->aec_reference_audio_samples[0], ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));   // Store the interference cancelled audio in the first reference channel
#endif
#endif
}

static void stage_ns(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_NS
#else
    int32_t DWORD_ALIGNED ns_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    configASSERT(NS_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    ns_process_frame(
                &ns_stage_state.state,
                ns_output,
                frame_data->samples[0]);
    memcpy(frame_data->samples[0], ns_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#if appconfAUDIO_PIPELINE_STORE_NS_AUDIO
    memcpy(frame_data->aec_reference_audio_samples[1], ns_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));   // Store NS audio in the second reference channel
#endif
#endif
}

static void stage_agc(frame_data_t *frame_data)
{
#if appconfAUDIO_PIPELINE_SKIP_AGC
#else
    int32_t DWORD_ALIGNED agc_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    configASSERT(AGC_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    agc_stage_state.md.vnr_flag = frame_data->vnr_pred_flag;
    agc_stage_state.md.aec_ref_power = frame_data->max_ref_energy;
    agc_stage_state.md.aec_corr_factor = frame_data->aec_corr_factor;

    agc_process_frame(
            &agc_stage_state.state,
            agc_output,
            frame_data->samples[0],
            &agc_stage_state.md);
    memcpy(frame_data->samples, agc_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));

    /* [LC-TELE dev.203] snapshot loss-control state into the peak-hold accumulator (diagnostics only;
     * does not touch any sample or lc_* value). */
    {
        float g  = float_s32_to_float(agc_stage_state.state.lc_gain);
        float cr = float_s32_to_float(agc_stage_state.state.lc_corr_val);
        float rp = float_s32_to_float(agc_stage_state.md.aec_ref_power);
        float np = float_s32_to_float(agc_stage_state.state.lc_near_power_est);
        float nb = float_s32_to_float(agc_stage_state.state.lc_near_bg_power_est);
        if (g < lc_tele.gain_min) { lc_tele.gain_min = g; }
        lc_tele.gain_now = g;
        lc_tele.corr_now = cr;
        if (cr < lc_tele.corr_min) { lc_tele.corr_min = cr; }   /* [dev.204] onset corr-dip depth */
        if (agc_stage_state.state.lc_t_far > lc_tele.t_far_max) { lc_tele.t_far_max = agc_stage_state.state.lc_t_far; }
        lc_tele.t_near_now = agc_stage_state.state.lc_t_near;
        if (rp > lc_tele.ref_pow_max) { lc_tele.ref_pow_max = rp; }
        lc_tele.ref_pow_now = rp;
        if (np > lc_tele.near_pow_max) { lc_tele.near_pow_max = np; }  /* [dev.204] */
        lc_tele.near_pow_now = np;
        lc_tele.near_bg_now  = nb;
    }
#endif
}

/* [LC-TELE dev.203] Serialize the loss-control peak-hold snapshot (LC_TELE_NUM_BYTES, little-endian)
 * and reset the per-window peaks. Called from the DFU servicer (tile 0) on a GET_LC_STATE poll. */
void audio_pipeline_get_lc_telemetry(uint8_t *buf)
{
    uint16_t gain_min_milli = (uint16_t)(lc_tele.gain_min * 1000.0f + 0.5f);
    uint16_t gain_now_milli = (uint16_t)(lc_tele.gain_now * 1000.0f + 0.5f);
    uint16_t corr_now_milli = (uint16_t)(lc_tele.corr_now * 1000.0f + 0.5f);
    uint16_t t_far_max      = (uint16_t)(lc_tele.t_far_max);
    uint16_t t_near_now     = (uint16_t)(lc_tele.t_near_now);
    float    ref_pow_max    = lc_tele.ref_pow_max;
    float    ref_pow_now    = lc_tele.ref_pow_now;
    uint16_t corr_min_milli = (uint16_t)(lc_tele.corr_min * 1000.0f + 0.5f);  /* [dev.204] */
    float    near_pow_max   = lc_tele.near_pow_max;
    float    near_pow_now   = lc_tele.near_pow_now;
    float    near_bg_now    = lc_tele.near_bg_now;

    buf[0]  = (uint8_t)(gain_min_milli & 0xFF); buf[1]  = (uint8_t)(gain_min_milli >> 8);
    buf[2]  = (uint8_t)(gain_now_milli & 0xFF); buf[3]  = (uint8_t)(gain_now_milli >> 8);
    buf[4]  = (uint8_t)(corr_now_milli & 0xFF); buf[5]  = (uint8_t)(corr_now_milli >> 8);
    buf[6]  = (uint8_t)(t_far_max & 0xFF);      buf[7]  = (uint8_t)(t_far_max >> 8);
    buf[8]  = (uint8_t)(t_near_now & 0xFF);     buf[9]  = (uint8_t)(t_near_now >> 8);
    memcpy(&buf[10], &ref_pow_max, sizeof(float));
    memcpy(&buf[14], &ref_pow_now, sizeof(float));
    /* [dev.204] near-power + onset corr-dip visibility (bytes 18..31) */
    buf[18] = (uint8_t)(corr_min_milli & 0xFF); buf[19] = (uint8_t)(corr_min_milli >> 8);
    memcpy(&buf[20], &near_pow_max, sizeof(float));
    memcpy(&buf[24], &near_pow_now, sizeof(float));
    memcpy(&buf[28], &near_bg_now,  sizeof(float));

    /* read-and-clear the peak-hold for the next window */
    lc_tele.gain_min     = 1.0f;
    lc_tele.t_far_max    = 0;
    lc_tele.ref_pow_max  = 0.0f;
    lc_tele.corr_min     = 1.0f;   /* [dev.204] */
    lc_tele.near_pow_max = 0.0f;   /* [dev.204] */
}

static void initialize_pipeline_stages(void)
{
    ic_init(&ic_stage_state.state);

    ns_init(&ns_stage_state.state);

    /* [LC-ECHO] Enable AGC Loss Control (LC) as the residual-echo suppressor. The shipping
     * AGC_PROFILE_ASR has lc_enabled=0 (all lc_* zeroed) -> NO residual-echo suppression, so the
     * speech-level echo leaks past the linear AEC. LC classifies each frame via aec_ref_power +
     * aec_corr_factor (already supplied by stage_aec/stage_agc) and scales the mic output:
     * far-end-only (our TTS echo) -> lc_gain_min (~-33dB, crushed below VAD); double-talk (real
     * barge-in) -> lc_gain_double_talk (0.9, preserved). Reference lc_* values are the in-tree
     * COMMS-derived set from modules/voice/test/lib_agc/test_process_frame/src/test_process_frame.h. */
    agc_config_t agc_conf = AGC_PROFILE_ASR;
    agc_conf.lc_enabled = 1;
    agc_conf.lc_n_frame_far = 17;
    agc_conf.lc_n_frame_near = 34;
    agc_conf.lc_corr_threshold = f32_to_float_s32(0.993);
    agc_conf.lc_bg_power_gamma = f32_to_float_s32(1.002);
    agc_conf.lc_gamma_inc = f32_to_float_s32(1.005);
    agc_conf.lc_gamma_dec = f32_to_float_s32(0.995);
    agc_conf.lc_far_delta = f32_to_float_s32(300);
    agc_conf.lc_near_delta = f32_to_float_s32(50);
    agc_conf.lc_near_delta_far_active = f32_to_float_s32(1500);   /* [dev.205 P4A] 100->1500: raise the far-active near-power bar above the echo onset peak (~370x bg) but well below real barge-in (5000-17000x) so the far-only crush engages THROUGH reply onset without losing barge-in protection */
    agc_conf.lc_gain_max = f32_to_float_s32(1);
    agc_conf.lc_gain_double_talk = f32_to_float_s32(0.9);
    agc_conf.lc_gain_silence = f32_to_float_s32(0.1);
    agc_conf.lc_gain_min = f32_to_float_s32(0.022387);
    agc_init(&agc_stage_state.state, &agc_conf);
    agc_stage_state.md.aec_ref_power = AGC_META_DATA_NO_AEC;
    agc_stage_state.md.aec_corr_factor = AGC_META_DATA_NO_AEC;
}

void audio_pipeline_init(
    void *input_app_data,
    void *output_app_data)
{
    const int stage_count = 3;
    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_vnr_and_ic,
        (pipeline_stage_t)stage_ns,
        (pipeline_stage_t)stage_agc,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_vnr_and_ic) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_ns),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_agc) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

    initialize_pipeline_stages();


    generic_pipeline_init((pipeline_input_t)audio_pipeline_input_i,
                        (pipeline_output_t)audio_pipeline_output_i,
                        input_app_data,
                        output_app_data,
                        stages,
                        (const size_t*) stage_stack_sizes,
                        appconfAUDIO_PIPELINE_TASK_PRIORITY,
                        stage_count);

}

#endif /* ON_TILE(0)*/
