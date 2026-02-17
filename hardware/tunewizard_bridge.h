/**
 * 调参侠 — MCU 端串口协议桥接 v2（三环调参）
 *
 * 支持角度环/速度环/转向环的独立 PID 设置和统一遥测上报。
 *
 * 协议格式:
 *   PC → MCU:  APID,seq=N,kp=X,ki=X,kd=X*CS\n   (角度环)
 *   PC → MCU:  SPID,seq=N,kp=X,ki=X,kd=X*CS\n   (速度环)
 *   PC → MCU:  TPID,seq=N,kp=X,ki=X,kd=X*CS\n   (转向环)
 *   MCU → PC:  D,seq=N,asp=F,apv=F,au=F,sspv=F,su=F,tpv=F,tu=F,t=N*CS\n
 *   MCU → PC:  ACK,seq=N,ok*CS\n
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "blue_serial.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Hot-updatable PID gains — written from UART ISR ===== */

/* Angle PID */
extern volatile float g_tw_kp;
extern volatile float g_tw_ki;
extern volatile float g_tw_kd;
extern volatile uint8_t g_tw_params_updated;

/* Speed PID */
extern volatile float g_tw_skp;
extern volatile float g_tw_ski;
extern volatile float g_tw_skd;
extern volatile uint8_t g_tw_speed_updated;

/* Turn PID */
extern volatile float g_tw_tkp;
extern volatile float g_tw_tki;
extern volatile float g_tw_tkd;
extern volatile uint8_t g_tw_turn_updated;

/* ===== Internal state ===== */
#define TW_BUF_SIZE 128

static char tw_rx_buf[TW_BUF_SIZE];
static uint8_t tw_rx_idx = 0;
static uint32_t tw_data_seq = 0;

static inline uint8_t TW_XorChecksum(const char *data, int len)
{
  uint8_t cs = 0;
  for (int i = 0; i < len; i++)
  {
    cs ^= (uint8_t)data[i];
  }
  return cs;
}

/**
 * Send ACK frame back.
 */
static inline void TW_SendAck(int seq)
{
  char ack[64];
  int len = snprintf(ack, sizeof(ack), "ACK,seq=%d,ok", seq);
  uint8_t cs = TW_XorChecksum(ack, len);
  char full[80];
  snprintf(full, sizeof(full), "%s*%02X\n", ack, cs);
  BlueSerial_SendString(full);
}

/**
 * Parse and apply a PID command for angle/speed/turn loop.
 * Input examples:
 *   "APID,seq=1,kp=2.70,ki=0.10,kd=2.50*XX"
 *   "SPID,seq=2,kp=2.00,ki=0.05,kd=0.00*XX"
 *   "TPID,seq=3,kp=4.00,ki=3.00,kd=0.00*XX"
 *   "PID,seq=N,kp=X,ki=X,kd=X*XX"  (legacy = angle)
 */
static inline void TW_ApplyCommand(const char *cmd)
{
  int seq = 0;
  float kp_val = 0.0f, ki_val = 0.0f, kd_val = 0.0f;
  const char *p;

  p = strstr(cmd, "seq=");
  if (p) seq = atoi(p + 4);

  p = strstr(cmd, "kp=");
  if (p) kp_val = (float)atof(p + 3);

  p = strstr(cmd, "ki=");
  if (p) ki_val = (float)atof(p + 3);

  p = strstr(cmd, "kd=");
  if (p) kd_val = (float)atof(p + 3);

  if (strncmp(cmd, "SPID,", 5) == 0)
  {
    g_tw_skp = kp_val;
    g_tw_ski = ki_val;
    g_tw_skd = kd_val;
    g_tw_speed_updated = 1;
  }
  else if (strncmp(cmd, "TPID,", 5) == 0)
  {
    g_tw_tkp = kp_val;
    g_tw_tki = ki_val;
    g_tw_tkd = kd_val;
    g_tw_turn_updated = 1;
  }
  else /* APID or PID (legacy) → angle */
  {
    g_tw_kp = kp_val;
    g_tw_ki = ki_val;
    g_tw_kd = kd_val;
    g_tw_params_updated = 1;
  }

  TW_SendAck(seq);
}

/**
 * Feed one received byte from UART ISR.
 */
static inline void TW_RxByte(uint8_t byte)
{
  if (byte == '\n' || byte == '\r')
  {
    if (tw_rx_idx > 0)
    {
      tw_rx_buf[tw_rx_idx] = '\0';

      /* Accept: PID, APID, SPID, TPID */
      if (strncmp(tw_rx_buf, "PID,", 4) == 0 ||
          strncmp(tw_rx_buf, "APID,", 5) == 0 ||
          strncmp(tw_rx_buf, "SPID,", 5) == 0 ||
          strncmp(tw_rx_buf, "TPID,", 5) == 0)
      {
        TW_ApplyCommand(tw_rx_buf);
      }

      tw_rx_idx = 0;
    }
  }
  else
  {
    if (tw_rx_idx < TW_BUF_SIZE - 1)
    {
      tw_rx_buf[tw_rx_idx++] = (char)byte;
    }
    else
    {
      tw_rx_idx = 0;
    }
  }
}

/**
 * Send combined telemetry frame with all three loops.
 *
 * @param a_sp   Angle setpoint (degrees)
 * @param a_pv   Angle actual (degrees)
 * @param a_u    Angle PID output
 * @param s_pv   Average speed
 * @param s_u    Speed PID output
 * @param t_pv   Differential speed
 * @param t_u    Turn PID output
 */
static inline void TW_SendData3(float a_sp, float a_pv, float a_u,
                                 float s_pv, float s_u,
                                 float t_pv, float t_u)
{
  tw_data_seq++;
  uint32_t t = HAL_GetTick();

  char body[200];
  int len = snprintf(body, sizeof(body),
      "D,seq=%lu,asp=%.2f,apv=%.2f,au=%.2f,spv=%.2f,su=%.2f,tpv=%.2f,tu=%.2f,t=%lu",
      (unsigned long)tw_data_seq, a_sp, a_pv, a_u, s_pv, s_u, t_pv, t_u,
      (unsigned long)t);

  uint8_t cs = TW_XorChecksum(body, len);
  char frame[230];
  snprintf(frame, sizeof(frame), "%s*%02X\n", body, cs);
  BlueSerial_SendString(frame);
}

/* Keep old single-loop API for backward compat */
static inline void TW_SendData(float setpoint, float measured, float output)
{
  TW_SendData3(setpoint, measured, output, 0.0f, 0.0f, 0.0f, 0.0f);
}

#ifdef __cplusplus
}
#endif
