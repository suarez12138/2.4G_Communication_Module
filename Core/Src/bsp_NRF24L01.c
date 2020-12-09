/**
  ******************************************************************************
  * 文件名程: bsp_NRF24L01.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: NRF24L01无线模块底层驱动实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  *
  * 淘宝：
  * 论坛：[url=http://www.ing10bbs.com]http://www.ing10bbs.com[/url]
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_NRF24L01.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_NRF24L01;
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xb0,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xb0,0x43,0x10,0x10,0x01};

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 串行FLASH初始化
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
void NRF24L01_SPI_Init(void)
{
    hspi_NRF24L01.Instance = NRF24L01_SPIx;
    hspi_NRF24L01.Init.Mode = SPI_MODE_MASTER;
    hspi_NRF24L01.Init.Direction = SPI_DIRECTION_2LINES;
    hspi_NRF24L01.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi_NRF24L01.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_NRF24L01.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi_NRF24L01.Init.NSS = SPI_NSS_SOFT;
    hspi_NRF24L01.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi_NRF24L01.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_NRF24L01.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi_NRF24L01.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi_NRF24L01.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi_NRF24L01);
    __HAL_SPI_ENABLE(&hspi_NRF24L01);
}

/**
  * 函数功能: SPI外设系统级初始化
  * 输入参数: hspi：SPI句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
//void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
//{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    if(hspi->Instance==NRF24L01_SPIx)
//    {
//        /* SPI外设时钟使能 */
//        NRF24L01_SPIx_RCC_CLK_ENABLE();
//        /* GPIO外设时钟使能 */
//        NRF24L01_SPI_GPIO_ClK_ENABLE();
//        NRF24L01_SPI_CS_CLK_ENABLE();
//        NRF24L01_CE_CLK_ENABLE();
//        NRF24L01_IRQ_CLK_ENABLE();
//
//        /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
//        __HAL_AFIO_REMAP_SWJ_DISABLE();
//
//        /**SPI3 GPIO Configuration
//        PF11     ------> SPI3_NSS
//        PB3      ------> SPI3_SCK
//        PB4      ------> SPI3_MISO
//        PB5      ------> SPI3_MOSI
//        */
//        GPIO_InitStruct.Pin = NRF24L01_SPI_SCK_PIN|NRF24L01_SPI_MOSI_PIN;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//        HAL_GPIO_Init(NRF24L01_SPI_GPIO_PORT, &GPIO_InitStruct);
//
//        GPIO_InitStruct.Pin = NRF24L01_SPI_MISO_PIN;
//        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        HAL_GPIO_Init(NRF24L01_SPI_GPIO_PORT, &GPIO_InitStruct);
//
//        HAL_GPIO_WritePin(NRF24L01_SPI_CS_PORT, NRF24L01_SPI_CS_PIN, GPIO_PIN_SET);
//        GPIO_InitStruct.Pin = NRF24L01_SPI_CS_PIN;
//        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//        HAL_GPIO_Init(NRF24L01_SPI_CS_PORT, &GPIO_InitStruct);
//
//        HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET);
//        GPIO_InitStruct.Pin = NRF24L01_CE_PIN;
//        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//        HAL_GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);
//
//        GPIO_InitStruct.Pin = NRF24L01_IRQ_PIN;
//        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//        GPIO_InitStruct.Pull = GPIO_PULLUP;
//        HAL_GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStruct);
//    }
//}

/**
  * 函数功能: SPI外设系统级反初始化
  * 输入参数: hspi：SPI句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
//void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
//{
//
//    if(hspi->Instance==NRF24L01_SPIx)
//    {
//        /* SPI外设时钟禁用 */
//        NRF24L01_SPIx_RCC_CLK_DISABLE();
//
//        /**SPI3 GPIO Configuration
//        PF11     ------> SPI3_NSS
//        PB3      ------> SPI3_SCK
//        PB4      ------> SPI3_MISO
//        PB5      ------> SPI3_MOSI
//        */
//        HAL_GPIO_DeInit(NRF24L01_SPI_GPIO_PORT, NRF24L01_SPI_SCK_PIN|NRF24L01_SPI_MISO_PIN|NRF24L01_SPI_MOSI_PIN);
//        HAL_GPIO_DeInit(NRF24L01_SPI_CS_PORT, NRF24L01_SPI_CS_PIN);
//        HAL_GPIO_DeInit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
//        HAL_GPIO_DeInit(NRF24L01_IRQ_PORT, NRF24L01_IRQ_PIN);
//    }
//}

/**
  * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t byte)
{
    uint8_t d_read,d_send=byte;
    if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFF)!=HAL_OK)
    {
        d_read=0xFF;
    }
    return d_read;
}

/**
  * 函数功能: 检测24L01是否存在
  * 输入参数: 无
  * 返 回 值: 0，成功;1，失败
  * 说    明：无
  */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
    uint8_t i;
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.
    NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址
    for(i=0;i<5;i++)if(buf[i]!=0XA5)break;
    if(i!=5)return 1;//检测24L01错误
    return 0;                 //检测到24L01
}

/**
  * 函数功能: SPI写寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:指定寄存器地址
  *
  */
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;
    NRF24L01_SPI_CS_ENABLE();                 //使能SPI传输
    status =SPIx_ReadWriteByte(&hspi_NRF24L01,reg);//发送寄存器号
    SPIx_ReadWriteByte(&hspi_NRF24L01,value);      //写入寄存器的值
    NRF24L01_SPI_CS_DISABLE();                 //禁止SPI传输
    return(status);                               //返回状态值
}

/**
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:要读的寄存器
  *
  */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    NRF24L01_SPI_CS_ENABLE();          //使能SPI传输
    SPIx_ReadWriteByte(&hspi_NRF24L01,reg);   //发送寄存器号
    reg_val=SPIx_ReadWriteByte(&hspi_NRF24L01,0XFF);//读取寄存器内容
    NRF24L01_SPI_CS_DISABLE();          //禁止SPI传输
    return(reg_val);           //返回状态值
}

/**
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: 无
  * 返 回 值: 此次读到的状态寄存器值
  * 说    明：无
  *
  */
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
    uint8_t status,uint8_t_ctr;

    NRF24L01_SPI_CS_ENABLE();           //使能SPI传输
    status=SPIx_ReadWriteByte(&hspi_NRF24L01,reg);//发送寄存器值(位置),并读取状态值
    for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
    {
        pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspi_NRF24L01,0XFF);//读出数据
    }
    NRF24L01_SPI_CS_DISABLE();       //关闭SPI传输
    return status;        //返回读到的状态值
}

/**
  * 函数功能: 在指定位置写指定长度的数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
  *
  */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,uint8_t_ctr;
    NRF24L01_SPI_CS_ENABLE();          //使能SPI传输
    status = SPIx_ReadWriteByte(&hspi_NRF24L01,reg);//发送寄存器值(位置),并读取状态值
    for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
    {
        SPIx_ReadWriteByte(&hspi_NRF24L01,*pBuf++); //写入数据
    }
    NRF24L01_SPI_CS_DISABLE();       //关闭SPI传输
    return status;          //返回读到的状态值
}

/**
  * 函数功能: 启动NRF24L01发送一次数据
  * 输入参数: 无
  * 返 回 值: 发送完成状况
  * 说    明：txbuf:待发送数据首地址
  *
  */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t sta;
    NRF24L01_CE_LOW();
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
    NRF24L01_CE_HIGH();//启动发送

    while(NRF24L01_IRQ_PIN_READ()!=0);//等待发送完成

    sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
    if(sta&MAX_TX)//达到最大重发次数
    {
        NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器
        return MAX_TX;
    }
    if(sta&TX_OK)//发送完成
    {
        return TX_OK;
    }
    return 0xff;//其他原因发送失败
}

/**
  * 函数功能:启动NRF24L01接收一次数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *
  */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;
    sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
    if(sta&RX_OK)//接收到数据
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
        NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
        return 0;
    }
    return 1;//没收到任何数据
}

/**
  * 函数功能: 该函数初始化NRF24L01到RX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *
  */
void NRF24L01_RX_Mode(void)
{
    NRF24L01_CE_LOW();
    NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0F);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);             //设置RF通信频率
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启

    NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度

    NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址


    NRF24L01_CE_HIGH(); //CE为高,进入接收模式
    HAL_Delay(1);
}

/**
  * 函数功能: 该函数初始化NRF24L01到TX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *
  */
void NRF24L01_TX_Mode(void)
{
    NRF24L01_CE_LOW();
    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址
    NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK

    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0xff);//设置自动重发间隔时间:4000us + 86us;最大自动重发次数:15次
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
    NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    NRF24L01_CE_HIGH();//CE为高,10us后启动发送
    HAL_Delay(1);
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
