/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_H
#define __USER_H

#ifdef __cplusplus
extern "C" {
#endif

void USART1_TX_Init(void );
void USART2_RX_Init(void);
void SPI1_Init(void);
void DMA_RX_Init(void);
void TimerSendDMA(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_H */
