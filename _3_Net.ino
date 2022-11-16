// NET : classe da camada de Rede

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

#include "Headers.h"

/**
 * Construtor da camada de Rede.
 */
NET::NET()
{
  
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
//                   ENDEREÇO DO SENSOR
//+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  my_addr = 3;   /* Endereço */ 
  my_dest = 2;   // destino
  my_neighbor = 4;   //vizinho
}

/**
 * Inicializa a camada de Controle de Acesso ao Meio.
 */
void NET::initialize(void) 
{
}

/**
 * Realiza a troca de endereços Origem e Destino
 */
void NET::swapAddresses(packet * pkt) 
{
  /* Troca os endereços de destino e origem para a retranmissão dos pacotes */
  //pkt->NetHdr[0] = pkt->NetHdr[2];
  //pkt->NetHdr[2] = Net.my_addr;
}

/**
 * Envia o pacote para a camada inferior
 */
inline void NET::send(packet * pkt) 
{
  /* Envia para a camada inferior */
  Mac.send(pkt);
  
  return;  
}

/**
 * Recebe o pacote da camada inferior
 */
inline void NET::receive(packet * pkt) 
{    
   // Foi criada uma variável indez para ser um índice de qual byte são armazenados os valores de RSSI
  int index;  //posicao para gravar os RSSI's
  int index1;  //posicao para gravar os LQI's 
 
// PRIMEIRO IF – PACOTE DECENDO
// Verifica se o pacote é para o sensor, se é um pacote de descida e se ainda não chegou no último sensor, verificando se o número de saltos é >3

if((pkt->NetHdr[0]==my_addr) && (pkt->NetHdr[1] == 1) && (pkt->NetHdr[3]>1)) //Se quantidade de saltos na rede > que 1 e se pkt esta marcado como pacote de roteamento RSSI
              {  
                // Coloca no pacote o endereço do nó vizinho para onde vai o pacote e o seu endereço para dizer quem enviou
                pkt->NetHdr[0] = Net.my_neighbor; //coloco o endereço como destino (descida) do nó e no byte 8 que é o NetHdr[0]
                pkt->NetHdr[2] = Net.my_addr; // coloca o próprio endereço como origem no byte 10 que é NetHdr[2]
                
                // Decrementa o número de saltos pois passou por este sensor
                pkt->NetHdr[3]=pkt->NetHdr[3]-1; //decremento o numero de saltos
                
               // Código para armazenamento da corrente
                index=pkt->Data[35];  // recebe o que está na posição 35. Como é o primeiro vai estar zero o byte incrementa o índice para armazenamento no byte correto a RSSI no vetor Data[]. Ou seja, o próximo sensor já sabe onde gravar, em função da linha acima
                pkt->Data[35]=pkt->Data[35]+1;   //  armazeno o RSSI de down na posicao correta e incremento posicao para o proximo sensor
                pkt->Data[index]=pkt->PhyHdr[0]; // como o índice já está certo é possível gravar a RSSI no local correto
                
                // Código para armazenamento de LQI
                index1=pkt->Data[35];  // recebe o que está na posição 35. Como é o primeiro vai estar zero o byte incrementa o índice para armazenamento no byte correto a RSSI no vetor Data[]. Ou seja, o próximo sensor já sabe onde gravar, em função da linha acima
                pkt->Data[35]=pkt->Data[35]+1;   //  armazeno o LQI de down na posicao correta e incremento posicao para o proximo sensor
                pkt->Data[index1]=pkt->PhyHdr[1]; // como o índice já está certo é possível gravar a LQI no local correto
                  
                // Envia pacote para a camada MAC
                Mac.send(pkt); //despacho o pacote pro meu vizinho
                return;
              }

// SEGUNDO IF – AÇÃO DO ÚLTIMO SENSOR DA REDE
 // if se for o último sensor
if((pkt->NetHdr[0]==my_addr) && (pkt->NetHdr[1]==1) && (pkt->NetHdr[3]==1)) //se quant. de saltos for = 1, entao sou o ultimo sensor da contagem
              {
               // observar que a partir de agora o pacote vai subir                
               pkt->NetHdr[0] = Net.my_dest; // esse é o endereço do destino mesmo, pois vai ter que subir e não o endereço de vizinho
               pkt->NetHdr[2] = Net.my_addr;  // coloca o endereço
               
               // INDICAÇÃO QUE O PACOTE AGORA TEM QUE SUBIR
               pkt->NetHdr[1]=2; //importante: marco o pacote para retornar obtendo os RSSI's de UP
                 
               // armazenamento da RSSI
               index=pkt->Data[35];
               pkt->Data[35]=pkt->Data[35]+1;  // incrementa index
               pkt->Data[index]=pkt->PhyHdr[0]; //  armazeno o RSSI de down na posicao correta este é o byte 0 do pacote
               
                // armazenamento da LQI
               index1=pkt->Data[35];
               pkt->Data[35]=pkt->Data[35]+1;  // incrementa index1
               pkt->Data[index1]=pkt->PhyHdr[1]; //  armazeno o LQI de down na posicao correta este é o byte 0 do pacote
  
               // Envia pacote para a camada MAC
               Mac.send(pkt); //despacho o pacote pro meu destino
               return;
              }

// TERCEIRO IF
// TEM QUE TESTAR O PACOTE QUE CHEGA DO VIZINHO. PODE SER UM //PACOTE QUE ESTÁ VOLTANDO NA CORRENTE
// LEMBRAR QUE TEM QUE CONTINUAR GRAVANDO AS RSSIs DE SUBIDA AGORA
if((pkt->NetHdr[0]==my_addr) && (pkt->NetHdr[1]==2) && (pkt->NetHdr[2] == Net.my_neighbor) ) //se quem enviou o pacote for meu vizinho,entao irei passar para o meu destino
                 {
                 // Envia pacote para o nó destino, que 
                 pkt->NetHdr[0] = Net.my_dest; //coloco meu destino
                 pkt->NetHdr[2] = Net.my_addr; //me coloco como origem
                   
                 index=pkt->Data[35]; //recebe a posicao onde gravará o RSSI
                 pkt->Data[35]=pkt->Data[35]+1; //incrementa a posição para o proximo sensor
                 pkt->Data[index]=pkt->PhyHdr[0]; //armazena a RSSI de UP
                 
                 index1=pkt->Data[35]; //recebe a posicao onde gravará o LQI
                 pkt->Data[35]=pkt->Data[35]+1; //incrementa a posição para o proximo sensor
                 pkt->Data[index1]=pkt->PhyHdr[1]; //armazena a LQI de UP
   
                 Mac.send(pkt); //despacho o pacote
                 return; 
                 }

}

/* Instanciação do objeto de acesso à classe da camada de Rede */
NET Net = NET();






