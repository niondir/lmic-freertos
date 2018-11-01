#ifndef DRV_LORAWAN_LMIC_DRV_LMIC_H_
#define DRV_LORAWAN_LMIC_DRV_LMIC_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "limits.h"
#include "semphr.h"

// TODO: Do not use HAL but api struct filled by board!
#include "github.com/Lobaro/hal-stm32l151CB-A/hal.h"
#include "lmic/hal.h"

typedef struct {
	bool otaa;
	uint8_t spreadingFactor;
	uint8_t txPower;
	bool adr;
	uint32_t devAddr; // ABP
	uint8_t appEUI[8]; // OTAA
	uint8_t devEUI[8]; // OTAA
	uint8_t devKey[16]; // OTAA - aka appKey
	uint8_t netSessionKey[16]; // ABP
	uint8_t appSessionKey[16]; // ABP
	bool useLowPowerAntennaOutput;
} lmicCfg_t;

void drv_lmic_init(lmicApi_t lmicApi, lmicCfg_t lmicCfg);
void drv_lmic_sx_irq_handler(uint8_t dio);
void drv_lmic_systick_irq_handler();

void drv_lmic_start();

// Setters for LoRaWAN Parameters
void drv_lmic_setOTAA(bool otaa);
void drv_lmic_setSpreadingFactor(uint8_t sf);
void drv_lmic_setAdr(bool adr);
void drv_lmic_setTxPower(uint8_t txPower);
void drv_lmic_setDevAddr(uint8_t* devAddr);
void drv_lmic_setAppEUI(uint8_t* appEUI);
void drv_lmic_setDevEUI(uint8_t* devEUI);
void drv_lmic_setAppKey(uint8_t* appKey);
void drv_lmic_setNetSessionKey(uint8_t* netSessionKey);
void drv_lmic_setAppSessionKey(uint8_t* appSessionKey);


bool drv_lmic_IsSending();
bool drv_lmic_IsBusy();
int drv_lmic_TimeToNextJobMs();

BaseType_t drv_lmic_send(uint8_t port, uint8_t* data, size_t len, TickType_t ticksToWait);
BaseType_t drv_lmic_sendConfirmed(uint8_t port, uint8_t* data, size_t len, TickType_t ticksToWait);
void drv_lmic_sleep();
void drv_lmic_wakeup();

bool lmic_hal_asserCalled();
void lmic_hal_increase_systicks(uint32_t ticks);

#endif /* DRV_LORAWAN_LMIC_DRV_LMIC_H_ */
