
#include "stm32f1xx.h"
#include <string.h>
#include "user.h"

#include "system_stm32f1xx.h"
#include "stm32f1xx_hal_tim.h"




#define DATASIZE	9
#define MAX_WORD    4


// массив передоваемых двухбайтных целочисленных положительных чисел, состоящий из 4 элементов
uint16_t buf_out[] = {0x1f7,0x3f6,0x2f5,0x1f4};
                // младшее слово ... старшее слово

// массив принятых двухбайтных целочисленных положительных чисел, состоящий из 4 элементов
uint16_t buf_in[4] = {0,0,};



typedef union {
//=======================================


    struct array_struct {
        uint32_t BlockCRC;
        union {
            uint8_t  buf[5];  // 40 бит - четыре слова по 10 бит.
            uint64_t word;    // 64 бита - четыре слова по 16 бит.
            // отсюда пишем в CRC->DR, а из него потом в BlockCRC
            uint32_t InCRC[2];
        } un;

    } uart_struct;
//=======================================


    uint8_t  uart_array[9];
//=======================================
} data;



data UART_TX_BUFFER;
data UART_RX_BUFFER;

/*
void Delay(uint16_t Val)
{
for( ; Val != 0; Val--)
        {
        __NOP();
        }
}
*/

void crc_calc_tx(void) {

    CRC->CR |= CRC_CR_RESET; // Сбросить данные в блоке
    // Записать данные из буфера в регистр данных

    CRC->DR = UART_TX_BUFFER.uart_struct.un.InCRC[0];
    CRC->DR = UART_TX_BUFFER.uart_struct.un.InCRC[1];
    UART_TX_BUFFER.uart_struct.BlockCRC    =   CRC->DR;
}

uint32_t crc_calc_rx(void) {

    CRC->CR |= CRC_CR_RESET; // Сбросить данные в блоке
    // Записать данные из буфера в регистр данных

    CRC->DR = UART_TX_BUFFER.uart_struct.un.InCRC[0];
    CRC->DR = UART_TX_BUFFER.uart_struct.un.InCRC[1];

    return CRC->DR;
}


// Передача DMA по таймеру
void TimerSendDMA(void)
{
    UART_TX_BUFFER.uart_struct.un.word = 0;
/******************************************************************************
            un_out.word |=  buf_out[3]; // старший байт
            un_out.word <<= 10; un_out.word |= buf_out[2];
            un_out.word <<= 10; un_out.word |= buf_out[1];
            un_out.word <<= 10; un_out.word |= buf_out[0]; // младший байт
*******************************************************************************/

// Вариант предыдущего закоментированного фрагмента, оформленный в виде цикла

    int8_t i=MAX_WORD-1;
    do {
        UART_TX_BUFFER.uart_struct.un.word |=  buf_out[i--];
        UART_TX_BUFFER.uart_struct.un.word <<= 10;
    } while (i>0);
    UART_TX_BUFFER.uart_struct.un.word |=  buf_out[i];

    crc_calc_tx(); // Заполняем блок CRC


    DMA1_Channel4->CCR &= ~DMA_CCR_EN; // выключаем передачу по каналу
    USART1->SR &= ~USART_SR_TC;  // сбрасываем 6-й бит - "передача завершена" (Transsmition complete)
    USART1->CR3 |= USART_CR3_DMAT; // устанавливаем 7-й бит "DMA enable transmitter"


    DMA1_Channel4->CCR = DMA_CCR_MSIZE | DMA_CCR_PSIZE; // размер данных 8 бит
    DMA1_Channel4->CCR |= DMA_CCR_DIR;  // направление из памяти в порт
    DMA1_Channel4->CCR |= DMA_CCR_MINC; // автоувеличение адреса памяти

    // (uint32_t) &UART_RX_BUFFER.uart_array[0], (uint32_t) &USART2->DR,  DATASIZE
    // uint32_t destAdd, uint32_t srcAdd, uint16_t datasize
    DMA1_Channel4->CMAR = (uint32_t)&UART_RX_BUFFER.uart_array[0];
    DMA1_Channel4->CPAR = (uint32_t)&USART2->DR;
    DMA1_Channel4->CNDTR = DATASIZE;

    DMA1->IFCR |= DMA_IFCR_CGIF4; // Сброс всех флагов
    DMA1_Channel4->CCR |= DMA_CCR_EN; // включаем передачу по каналу
}



void start_clk(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // включаем тактирование UART1
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // включаем тактирование UART2
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // включаем тактирование порта A
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // включаем тактирование порта B
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;    // включаем тактирование DMA1
    RCC->AHBENR  |= RCC_AHBENR_CRCEN;     // включаем тактирование блока CRC
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   // Enable SPI1 CLock
}



void GPIOConfig (void)
{
    /*Confgiure PA9 as output maximum speed to 50MHz
     * and alternate output push-pull mode*/
    GPIOA->CRH &=  ~ (GPIO_CRH_CNF9 | GPIO_CRH_MODE9); // сбрасываем биты полей CNF и MODE
    GPIOA->CRH |= (0b10 << GPIO_CRH_CNF9_Pos) | (0b11 << GPIO_CRH_MODE9_Pos)  ;  // устанавливаем биты


}

void SPI1_Init(void)
{

  /**********************************************************/
  /*** Настройка выводов GPIOA на работу совместно с SPI1 ***/
  /**********************************************************/
  //PA7 - MOSI
  //PA6 - MISO
  //PA5 - SCK

  //Для начала сбрасываем все конфигурационные биты в нули
  GPIOA->CRL &= ~(GPIO_CRL_CNF5_Msk | GPIO_CRL_MODE5_Msk
                | GPIO_CRL_CNF6_Msk | GPIO_CRL_MODE6_Msk
                | GPIO_CRL_CNF7_Msk | GPIO_CRL_MODE7_Msk);

  //Настраиваем
  GPIOA->CRL |= (0b10<<GPIO_CRL_CNF5_Pos) | (0b11<<GPIO_CRL_MODE5_Pos);
  GPIOA->CRL |= (0b01<<GPIO_CRL_CNF6_Pos) | (0b00<<GPIO_CRL_MODE6_Pos);
  GPIOA->CRL |= (0b10<<GPIO_CRL_CNF7_Pos) | (0b11<<GPIO_CRL_MODE7_Pos);


  /**********************/
  /*** Настройка SPI1 ***/
  /**********************/

  SPI1->CR1 = 0<<SPI_CR1_DFF_Pos  //Размер кадра 8 бит
    | 0<<SPI_CR1_LSBFIRST_Pos     //MSB transmitted first
    | 1<<SPI_CR1_SSM_Pos          //Программное управление SS
    | 1<<SPI_CR1_SSI_Pos          //SS в высоком состоянии
    | 0x04<<SPI_CR1_BR_Pos        //Скорость передачи: F_PCLK/32
    | 1<<SPI_CR1_MSTR_Pos         //Режим Master (ведущий)
    | 0<<SPI_CR1_CPOL_Pos | 0<<SPI_CR1_CPHA_Pos; //Режим работы SPI: 0

  SPI1->CR1 |= 1<<SPI_CR1_SPE_Pos; //Включаем SPI
}

void SPI_Transmit (uint8_t *data, int size)
{

    /************** STEPS TO FOLLOW *****************
    1. Wait for the TXE bit to set in the Status Register
    2. Write the data to the Data Register
    3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
    4. Clear the Overrun flag by reading DR and SR
    ************************************************/

    int i=0;
    while (i<size)
    {
       while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
       SPI1->DR = data[i];  // load the data into the Data Register
       i++;
    }

/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
    while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
    while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

    //  Clear the Overrun flag by reading DR and SR
    uint8_t temp = SPI1->DR;
    temp = SPI1->SR;
}



void Blink(uint32_t delay)
{
    GPIOC->ODR &= ~ GPIO_ODR_ODR13; // зажечь светодиод
    //GPIOB->BSRR = GPIO_BSRR_BR13; // сброс бита
    //Delay(delay);
    HAL_Delay(delay);
    GPIOC->ODR |=   GPIO_ODR_ODR13; // погасить светодиод
    //GPIOB->BSRR = GPIO_BSRR_BS13; // установка бита
    //Delay(delay);
    HAL_Delay(delay);
}


void USART1_TX_Init(void) // Инициализация UART1
{
    /*Confgiure PA9 as output maximum speed to 50MHz
     * and alternate output push-pull mode*/
    GPIOA->CRH &=  ~ (GPIO_CRH_CNF9 | GPIO_CRH_MODE9); // сбрасываем биты полей CNF и MODE
    GPIOA->CRH |= (0b10 << GPIO_CRH_CNF9_Pos) | (0b11 << GPIO_CRH_MODE9_Pos)  ;  // устанавливаем биты


    // разблокировать UART1 и передачу по нему
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_UE); // 3-й и 13-й разряды

    /*Enable DMA for Transmit*/
    USART1->CR3|=USART_CR3_DMAT;
}

void USART2_RX_Init(void) // Инициализация UART2
{
    /*Configure PA3 as Input floating*/
    /*Set mode to be input*/
    GPIOA->CRL &=  ~ (GPIO_CRL_CNF3 | GPIO_CRL_MODE3); // сбрасываем биты полей CNF и MODE
    GPIOA->CRL |= (0b01 << GPIO_CRL_CNF3_Pos) | (0b00 << GPIO_CRL_MODE3_Pos)  ;  //

    GPIOA->ODR |= GPIO_ODR_ODR3; // Pull Up for PA3


    USART2->BRR = 7500; // 72000000 Гц / 9600 бод = 7500
    // разблокировать UART2 и приём по нему
    USART2->CR1 |= (USART_CR1_RE | USART_CR1_UE);

    /*Enable DMA for receiver*/
    USART2->CR3|=USART_CR3_DMAR;
}

void DMA_RX_Config (uint32_t destAdd, uint32_t srcAdd, uint16_t datasize)
{
    /*Set the peripheral address to be USART2->DR*/
    DMA1_Channel6->CPAR=srcAdd;

    //Set the memory addrerr in MAR Register
    DMA1_Channel6->CMAR=destAdd;

    //Set the data size in CNDTR Register
    DMA1_Channel6->CNDTR=datasize;

    // Enable the DMA1
    DMA1_Channel6->CCR|=DMA_CCR_EN;
}


void DMA_RX_Init(void)
{
    //The channel shall be configured as following:

    // MINC: Memory increment mode
    // CIRC: Circular mode
    // TCIE: Transfer complete interrupt enable
    // HTIE: Half transfer interrupt enable
    // TEIE: transfer error interrupt enable
    DMA1_Channel6->CCR|=DMA_CCR_MINC|DMA_CCR_CIRC|DMA_CCR_TCIE /*| DMA_CCR_HTIE | DMA_CCR_TEIE*/;

    // DIR: Data transfer direction
    DMA1_Channel6->CCR &= ~DMA_CCR_DIR; // 0: Read from peripheral

    // 6. Set the Peripheral data size (PSIZE) and the memory data size (MSIZE)
    DMA1_Channel6->CCR &= ~(0b00 << DMA_CCR_PSIZE_Pos); // 00 : 8 Bit data
    DMA1_Channel6->CCR &= ~(0b00 << DMA_CCR_MSIZE_Pos); // 00 : 8 Bit data

    // 00: Low
    // 01: Medium
    // 10: High
    // 11: Very high
    DMA1_Channel6->CCR &= ~(0b01 << DMA_CCR_PL_Pos); // Channel priority level

    DMA_RX_Config ((uint32_t) &UART_RX_BUFFER.uart_array[0], (uint32_t) &USART2->DR,  DATASIZE);
}



// Copy from USART2 в UART_RX_BUFFER
void DMA1_Channel6_IRQHandler(void)
{
    if ((DMA1->ISR) &  DMA_ISR_TCIF6 ) // If the TRANSFER COMPLETE interrupt is set
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF6;  // Clear the bit interrupt DMA TC


        if(UART_RX_BUFFER.uart_struct.BlockCRC != crc_calc_rx())
        {
            //Blink(55560);
            Blink(500);
        }
        else
            //Blink(5560);
            Blink(10);

/*** Заносим 10 младших разрядов в каждое 16-ти разрядное слово массива
        buf_in[0] = un_in.word & 0x3ff;
        std::cout << buf_in[0] << std::endl;

        un_in.word >>= 10;
        buf_in[1] = un_in.word & 0x3ff;
        std::cout << buf_in[1] << std::endl;

        un_in.word >>= 10;
        buf_in[2] = un_in.word & 0x3ff;
        std::cout << buf_in[2] << std::endl;

        un_in.word >>= 10;
        buf_in[3] = un_in.word & 0x3ff;
        std::cout << buf_in[3] << std::endl;
**********************************************************************/

        // Вариант предыдущего закоментированного фрагмента, оформленный в виде цикла

        int i=0;
        do {
            buf_in[i++] = UART_RX_BUFFER.uart_struct.un.word & 0x3ff;
            UART_RX_BUFFER.uart_struct.un.word >>= 10;
        } while (i<MAX_WORD);

    }
    SPI_Transmit(&UART_RX_BUFFER.uart_array[0], 9);
}


