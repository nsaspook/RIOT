/* 
 * File:   spi.h
 * Author: root
 *
 * Created on October 25, 2016, 1:53 PM
 */

#ifndef SPI_H
#define	SPI_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>

    void SPI_Init(void);
    void SPI_Speed(const uint8_t);
    void SPI_ClearBufs(void);
    void SPI_TxStart(void);
    bool SPI_IsNewRxData(void);
    uint8_t SPI_ReadRxBuffer(void);
    bool SPI_IsTxData(void);
    void SPI_WriteTxBuffer(const uint8_t);
    uint16_t SPI_GetTXBufferFreeSpace(void);
    uint8_t SPI_PeekRxBuffer(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

