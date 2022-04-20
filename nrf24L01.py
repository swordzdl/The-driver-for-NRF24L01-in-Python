# Coding environment #
import RPi.GPIO as GPIO
import time
#import numpy as np
#import wiringpi
#from spidev import *

# Address define
TX_ADDRESS = [0x34, 0x43, 0x10, 0x10, 0x01]
RX_ADDRESS = [0x34, 0x43, 0x10, 0x10, 0x01]
CHANNAL    = 40

TX_PLOAD_WIDTH = 32
RX_PLOAD_WIDTH = 32
TX_ADR_WIDTH = 5
RX_ADR_WIDTH = 5

OneMbitps = 0x07
TwoMbitps = 0x0f
#nrf24l01 registers
CONFIG      = 0X00  #config register address
EN_AA       = 0X01  #enable auto ACK register address
EN_RXADDR   = 0X02  #enable RX addresses register address
SETUP_AW    = 0X03  #setup address width register address
SETUP_RETR  = 0X04  #setup Auto.Retrans register address
RF_CH       = 0X05  #RF channel register address
RF_SETUP    = 0X06  #RF setup
STATUS      = 0X07  #status register
OBSERVE_TX  = 0X08  
CD          = 0X09  #carrier detect
RX_ADDR_Pn = [0X0A, 0X0B, 0X0C, 0X0D, 0X0E, 0X0F]
'''RX_ADDR_P0  = 0X0A
RX_ADDR_P1  = 0X0B
RX_ADDR_P2  = 0X0C
RX_ADDR_P3  = 0X0D
RX_ADDR_P4  = 0X0E
RX_ADDR_P5  = 0X0F'''
TX_ADDR     = 0X10
RX_PW_Pn    = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16]
'''RX_PW_P0    = 0X11  #RX payload width, pipe0
RX_PW_P1    = 0X12
RX_PW_P2    = 0X13
RX_PW_P3    = 0X14
RX_PW_P4    = 0X15
RX_PW_P5    = 0X16'''
FIFO_STATUS = 0X17  #FIFO status register

EN_RXADDR_DIC = {0:0x01, 1:0x02, 2:0x04, 3:0x08, 4:0x10, 5: 0x20}
#some important status's signal
TX_FULL     = 0x01
MAX_RT      = 0x10
TX_DS       = 0x20
RX_DR       = 0x40

#SPI commands
NRF_READ_REG = 0X00
NRF_WRITE_REG = 0X20
RD_RX_PLOAD  = 0X61
WR_TX_PLOAD  = 0XA0
FLUSH_TX     = 0XE1
FLUSH_RX     = 0XE2
REUSE_TX_PL  = 0XE3
NOP          = 0XFF

#SPI channel init
SPIbus = 0
SPIchannel = 0
SPIspeed = 9000000  #Hz

#pin mode
CSN = 32
SCK = 29
IRQ = 12
CE  = 22
MOSI = 31
MISO = 33
#init spi and send&recv
#wiringPi.wiringPiSPIsetup(SPIchannel, SPIspeed)
#recvdata = wiringPi.wiringPiSPIDataRW(SPIchannel,sendData)

'''def spi_init(bus,channel,speed):
    rspi = SpiDev(bus, channel)
    #a = rspi.open(bus,channel)
    rspi.max_speed_hz = speed
    #rspi.cshigh = False
    return rspi

#spi1 = spi_init(SPIbus, SPIchannel, SPIspeed)'''

def gpio_init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)   
    GPIO.setup(CE,GPIO.OUT)
    GPIO.setup(CSN, GPIO.OUT)
    GPIO.setup(SCK, GPIO.OUT)
    GPIO.setup(MOSI, GPIO.OUT)
    GPIO.setup(MISO, GPIO.IN)
    GPIO.setup(IRQ, GPIO.IN)

def SPI_RW(bytees):

    for index in range(0,8):
        GPIO.output(MOSI, bytees & 0x80)
        bytees = (bytees << 1)
        GPIO.output(SCK, 1)
        bytees = bytees | GPIO.input(MISO)
        GPIO.output(SCK, 0)
    if bytees > 0xff:
        bytees = bytees & 0xff
    return bytees

def nrf_Read_reg(reg, dat = 0xff):
    '''GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)   
    GPIO.setup(CSN,GPIO.OUT)
    reg = [reg]'''
    GPIO.output(CSN, 0)

    SPI_RW(reg)
    #dat = [dat]
    status = SPI_RW(dat)
    GPIO.output(CSN,1)
    #GPIO.cleanup()
    return status

def nrf_Write_reg(reg, dat):
    GPIO.output(CSN, 0)
    sta = SPI_RW(reg)
    SPI_RW(dat)
    GPIO.output(CSN, 1)
    return sta

##def nrf_read_reg(reg):
##    wiringPi.wiringPiSPIDataRW(SPIchannel,reg)
##    val = wiringPi.wiringPiSPIDataRW(SPIchannel,0)
##    GPIO.output(CSN,1)
##    return val

def nrf_readBuf(reg, pBuf, byte):
    '''GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)   
    GPIO.setup(CSN,GPIO.OUT)'''
    GPIO.output(CSN,0)
    #reg = [reg]
    status = SPI_RW(reg)
    for i in range(0,byte):
        pBuf[i] = SPI_RW(0)
    GPIO.output(CSN,1)
    #GPIO.cleanup()
    return status

def nrf_writeBuf(reg, pBuf, byte):
    '''GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)   
    GPIO.setup(CSN,GPIO.OUT)'''
    GPIO.output(CSN,0)

    status = SPI_RW(reg)
    for i in range(0,byte):
        SPI_RW(pBuf[i])
    GPIO.output(CSN,1)
    #GPIO.cleanup()
    return status

def nrf_RXmode(Pn:int =0, rx_addr: int = RX_ADDRESS,rx_pload_width: int = RX_PLOAD_WIDTH, rx_adr_width: int = RX_ADR_WIDTH):
    """Pn：选定第n号接收（数据宽度）寄存器地址，rx_addr：接收地址（需设定与发送地址相同）,rx_pload_width：接收数据宽度，默认32，rx_adr_width：接收地址宽度，默认为5"""
    GPIO.output(CE, 0)
    if (Pn == 0) | (Pn ==1) :
        nrf_Write_reg(NRF_WRITE_REG + RX_PW_Pn[Pn], rx_pload_width)
        nrf_writeBuf(NRF_WRITE_REG + RX_ADDR_Pn[Pn], rx_addr, rx_adr_width)
        #nrf_Write_reg(NRF_WRITE_REG + EN_RXADDR, EN_RXADDR_DIC[Pn])
    else:
        nrf_Write_reg(NRF_WRITE_REG + RX_PW_Pn[Pn], rx_pload_width)
        nrf_writeBuf(NRF_WRITE_REG + RX_ADDR_Pn[Pn], [rx_addr], 1)
        #nrf_Write_reg(NRF_WRITE_REG + EN_RXADDR, EN_RXADDR_DIC[Pn])
    GPIO.output(CE, 1)  ##set CE pin to enable RX device
    time.sleep(0.00013)
    return

def nrf_Rxpacket(rx_Buf)-> int:
    #nrf_RXmode(Pn, rx_addr,rx_pload_width, rx_adr_width)
    nrf_Write_reg(NRF_WRITE_REG+CONFIG, 0x0f)## set PWR_UP bit, enable CRC(2 byte),PRIM_RX RX_DR enable
    reval = 0

    status = nrf_Read_reg(STATUS)
    if status & RX_DR:## if really receive data and trigger RX interrupt
        GPIO.output(CE,0)
        nrf_readBuf(RD_RX_PLOAD, rx_Buf, TX_PLOAD_WIDTH)
        nrf_Write_reg(FLUSH_RX,0xff)
        reval = 1
    nrf_Write_reg(NRF_WRITE_REG+STATUS, status)## clear RX_DR or ohter interrupt flag
    #GPIO.cleanup()
    return reval

def nrf_TXmode(tx_addr : int, tx_pload_width : int):
    GPIO.output(CE, 0)
    nrf_writeBuf(NRF_WRITE_REG + TX_ADDR, tx_addr, tx_pload_width)
    nrf_writeBuf(NRF_WRITE_REG + RX_ADDR_Pn[0], tx_addr, tx_pload_width)  ## to auto ack
    return

def nrf_Txpacket(tx_buf, tx_addr : int = TX_ADDRESS, tx_pload_width : int= TX_ADR_WIDTH):

    nrf_TXmode(tx_addr, tx_pload_width)
    nrf_Write_reg(NRF_WRITE_REG + CONFIG, 0x0e)##set PWR_UP bit, enable CRC(2 bytes)&Prim:TX.MAX_RT & TX_DS enable
    '''nrf_Write_reg(NRF_WRITE_REG + EN_AA, 0X00)
    nrf_Write_reg(NRF_WRITE_REG + EN_RXADDR, 0X00)
    nrf_Write_reg(NRF_WRITE_REG + SETUP_RETR, 0X00)'''
    '''sta = nrf_Read_reg(STATUS)
    FIFO = nrf_Read_reg(FIFO_STATUS)'''
    nrf_writeBuf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH)
    '''sta = nrf_Read_reg(STATUS)
    FIFO = nrf_Read_reg(FIFO_STATUS)'''
    GPIO.output(CE,1)
    #sta = nrf_Read_reg(STATUS)
    time.sleep(0.0001)

    GPIO.output(CE,0)

    #sta = nrf_RW_reg(STATUS)
    GPIO.output(CE, 1)

    GPIO.output(CE, 0)
    while GPIO.input(IRQ) !=0:
        pass
    #nrf_Write_reg(FLUSH_TX, 0xff)
    sta = nrf_Read_reg(NRF_READ_REG + STATUS)
    nrf_Write_reg(NRF_WRITE_REG + STATUS, sta)
    #FIFO = nrf_Read_reg(NRF_READ_REG + FIFO_STATUS)

    if sta & MAX_RT :
        nrf_Write_reg(FLUSH_TX, 0xff)  #clean TX FIFO register
        #print('MAX_RT')
        return MAX_RT
    if sta & TX_DS :
        return TX_DS
    #GPIO.cleanup()
    return 0xff

def nrf_24L01_Config(ch : int = CHANNAL, tspeed : int = TwoMbitps):
    GPIO.output(CE, 0)
    GPIO.output(CSN, 1)
    GPIO.output(SCK, 0)

    nrf_Write_reg(NRF_WRITE_REG + EN_AA, 0x3f)
    nrf_Write_reg(NRF_WRITE_REG + EN_RXADDR, 0x3f)
    nrf_Write_reg(NRF_WRITE_REG + SETUP_RETR, 0x1a)
    nrf_Write_reg(NRF_WRITE_REG + RF_CH, ch)
    nrf_Write_reg(NRF_WRITE_REG + RF_SETUP, tspeed)  # 2Mbps 0X0F    1Mbps  0X07
    #nrf_Write_reg(NRF_WRITE_REG + CONFIG, 0x0e)

    #GPIO.output(CE,1)
    #GPIO.cleanup()
    return


def nrf_link_check():
    ref = 1
    check_dat = 0xd3
    buffx = [check_dat, check_dat, check_dat, check_dat, check_dat]
    nrf_writeBuf(NRF_WRITE_REG + TX_ADDR, buffx, 5)
    nrf_readBuf(TX_ADDR, buffx, 5)
    for i in range(0,5):
        if buffx[i] !=   check_dat:
            ref = 0
            break
    if i!= 4:
        ref =0
    return ref


if __name__=='__main__':
    gpio_init()

    stat = nrf_link_check()
    if stat == 1:
        print('successful load')
        nrf_24L01_Config()
        temp_buf = [ord('a')]*33
        rec = [0]*33
        #mode = ord('a')
        while True:
            """if nrf_Txpacket(temp_buf) == TX_DS :
                print('link success')
                '''key = mode
                for t in range(0,32):
                    key += 1
                    if key > ord('~') :
                        key = ord(' ')
                        temp_buf[t] = key
                mode += 1
                if mode > ord('~') :
                    temp_buf[32] = 0'''"""
            if nrf_Rxpacket(rec) == 1 :
                print('receive success')
                for i in range(0,32):
                    rec[i] = chr(rec[i])
                print(rec)
            time.sleep(0.1)
    else:
        print('load failure')
        