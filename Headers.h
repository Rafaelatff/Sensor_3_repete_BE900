// Header.h : cabeçalhos de classes

// Mais informações em www.radiuino.cc
// Copyright (c) 2011
// Author: Pedro Henrique Gomes e Omar C. Branquinho
// Versão 1.0: 12/09/2011

// Este arquivo é parte da plataforma Radiuino
// Este programa é um software livre; você pode redistribui-lo e/ou modifica-lo dentro dos termos da Licença Pública Geral Menor GNU 
// como publicada pela Fundação do Software Livre (FSF); na versão 2 da Licença, ou (na sua opnião) qualquer futura versão.
// Este programa é distribuido na esperança que possa ser  util, mas SEM NENHUMA GARANTIA; sem uma garantia implicita 
// de ADEQUAÇÂO a qualquer MERCADO ou APLICAÇÃO EM PARTICULAR. Veja a Licença Pública Geral Menor GNU para maiores detalhes.
// Você deve ter recebido uma cópia da Licença Pública Geral Menor GNU junto com este programa, se não, escreva para a Fundação 
// do Software Livre(FSF) Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
    
// This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version. This library 
// is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
// or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details. You should have received a copy 
// of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, 
// Fifth Floor, Boston, MA  02110-1301  USA

#ifndef HEADERS_H
#define HEADERS_H 1

/**
 * Estrutura do pacote a ser transmitido e recebido. O pacote possui 52 bytes, sendo 4 bytes de cabeçalho de camada física (Phy), 4 bytes de cabeçalho
 * de camada de Controle de Acesso ao Meio (MAC), 4 bytes de cabeçalho de camada de Rede (Net), 4 bytes de cabeçalho de camada de Transporte (Transp) e
 * e 4 bytes de cabeçalho de camada de Aplicação (App). Os 36 bytes restantes são reservados para payload de AD e IO.
 */
typedef struct 
{  
  /* Cabeçalho da camada Física (Phy) */
  byte PhyHdr[4];
  
  /* Cabeçalho da camada de Controle de Acesso ao Meio (MAC) */
  byte MACHdr[4];
  
  /* Cabeçalho da camada de Rede (Net) */
  byte NetHdr[4];
  
  /* Cabeçalho da camada de Transporte (Transp) */
  byte TranspHdr[4];

  /* Bytes o payload da corrente */
  byte Data[36];
 
  
} packet;

packet g_pkt;

/**
 * Classe de camada de Aplicação
 */
 
#define AD0_PIN  0
#define AD1_PIN  1
#define AD2_PIN  2
#define AD3_PIN  3
#define AD4_PIN  4
#define AD5_PIN  5

#define IO0_PIN  4
#define IO1_PIN  5
#define IO2_PIN  6
#define IO3_PIN  7
#define IO4_PIN  8
#define IO5_PIN  9

class APP
{
  public:
    APP(void);
    inline void initialize(void);
    inline void send(packet * pkt);
    inline void receive(packet * pkt);

  private:
    int IO_0, IO_1, IO_2, IO_3, IO_4, IO_5;

};

/* Objeto de acesso à classe da camada de Aplicação */
extern APP App;

/**
 * Classe de camada de Transporte
 */
class TRANSP
{
  public:
    TRANSP(void);
    inline void initialize(void);
    inline void send(packet * pkt);
    inline void receive(packet * pkt);

    byte my_addr;
    byte my_dest;
    byte my_neighbor;
  private:
  
};

/* Objeto de acesso à classe da camada de Transporte */
extern TRANSP Transp;

/**
 * Classe de camada de Rede
 */
class NET
{
  public:
    NET(void);
    inline void initialize(void);
    inline void send(packet * pkt);
    inline void receive(packet * pkt);
    void swapAddresses(packet * pkt);
    
    byte my_addr;
    byte my_dest;
    byte my_neighbor;    

  private:
    
};

/* Objeto de acesso à classe da camada de Rede */
extern NET Net;

/**
 * Classe de camada de Controle de Acesso ao Meio
 */
#define  SLEEP_MSG  1
#define  SLEEP_ACK  2
class MAC
{
  public:
    MAC(void);
    inline void initialize(void);
    inline void send(packet * pkt);
    inline void receive(packet * pkt);
    
    volatile int time_to_sleep;
  private:
    
};

/* Objeto de acesso à classe da camada de Controle de Acesso ao Meio */
extern MAC Mac;

/**
 * Classe de camada Física
 */
#define BUFFLEN  CC1101_PACKT_LEN
byte serialData[BUFFLEN + 1];

class PHY
{
  public:
    PHY(void);
    inline void initialize();
    inline void send(packet * pkt);
    inline int receive(packet * pkt);
    
    void sendSerial(packet * pkt);
    void receiveSerial(void);
    
    byte txFifoFree(void);
    void setChannel(byte channel);
    void lowPowerOn(void);    
    void setPower(byte power);
    void setFreqOffset(byte freq_offset);
    
    boolean carrierSense(void);
    
    byte power;                 /* Potência */
    byte channel;               /* Canal */
    byte freq_offset;           /* Offset de frequencia */
    int serial_baudrate;        /* Serial baudrate */
  
  private:
    int initCC1101Config(void);
    void configWatchdog(int time);
    void strobe_idle_wait(void);
};

/* Objeto de acesso à classe da camada Física */
extern PHY Phy;

/* Configuração de registradores do CC1101. Obtidos através do SmartRF Studio 7 */
// Deviation = 20.629883 
// Base frequency = 914.999969 
// Carrier frequency = 914.999969 
// Channel number = 0 
// Carrier frequency = 914.999969 
// Modulated = true 
// Modulation format = 2-FSK 
// Manchester enable = true 
// Sync word qualifier mode = 30/32 sync word bits detected 
// Preamble count = 4 
// Channel spacing = 199.951172 
// Carrier frequency = 914.999969 
// Data rate = 38.3835 
// RX filter BW = 101.562500 
// Data format = Normal mode 
// CRC enable = true 
// Device address = 0 
// Address config = No address check 
// CRC autoflush = false 
// PA ramping = false 
// TX power = 7 
const byte CC1101_registerSettings[CC1101_NR_OF_CONFIGS][CC1101_NR_OF_REGISTERS] PROGMEM = {
{ 
    0x04,  // IOCFG2        GDO2 Output Pin Configuration
    0x07,  // IOCFG0        GDO0 Output Pin Configuration
    0x07,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    0x34,  // PKTLEN        Packet Length
    0x04,  // PKTCTRL1      Packet Automation Control
    0x04,  // PKTCTRL0      Packet Automation Control
    0x00,  // ADDR          Device Address
    0x00,  // CHANNR        Channel Number
    0x06,  // FSCTRL1       Frequency Synthesizer Control
    0x00,  // FSCTRL0       Frequency Synthesizer Control
    0x23,  // FREQ2         Frequency Control Word, High Byte
    0x31,  // FREQ1         Frequency Control Word, Middle Byte
    0x3B,  // FREQ0         Frequency Control Word, Low Byte
    0xCA,  // MDMCFG4       Modem Configuration
    0x83,  // MDMCFG3       Modem Configuration
    0x0B,  // MDMCFG2       Modem Configuration
    0x22,  // MDMCFG1       Modem Configuration
    0xF8,  // MDMCFG0       Modem Configuration
    0x35,  // DEVIATN       Modem Deviation Setting
    0x18,  // MCSM0         Main Radio Control State Machine Configuration
    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
    0x6C,  // BSCFG         Bit Synchronization Configuration
    0x43,  // AGCCTRL2      AGC Control
    0x40,  // AGCCTRL1      AGC Control
    0x91,  // AGCCTRL0      AGC Control
    0x56,  // FREND1        Front End RX Configuration
    0x10,  // FREND0        Front End TX Configuration
    0xE9,  // FSCAL3        Frequency Synthesizer Calibration
    0x2A,  // FSCAL2        Frequency Synthesizer Calibration
    0x00,  // FSCAL1        Frequency Synthesizer Calibration
    0x1F,  // FSCAL0        Frequency Synthesizer Calibration
    0x59,  // FSTEST        Frequency Synthesizer Calibration Control
    0x81,  // TEST2         Various Test Settings
    0x35,  // TEST1         Various Test Settings
    0x09,  // TEST0         Various Test Settings
}
};

const byte CC1101_paTable[CC1101_NR_OF_CONFIGS][CC1101_PA_TABLESIZE] PROGMEM ={
// -30  -20   -15  -10    0   5    7   10
  {0x03,0x0E,0x1E,0x27,0x8E,0x84,0xCC,0xC3},    // Configuração 0
};

#endif
