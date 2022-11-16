// APP : classe da camada de Aplicação

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
 * Construtor da camada de Aplicação.
 */
APP::APP()
{
}

/**
 * Inicializa a camada de Aplicação.
 */
void APP::initialize(void) 
{  
  // Faz com que todos os pinos de IO sejam saída
  pinMode (IO0_PIN, OUTPUT); 
  pinMode (IO1_PIN, OUTPUT); 
  pinMode (IO2_PIN, OUTPUT); 
  pinMode (IO3_PIN, OUTPUT); 
  pinMode (IO4_PIN, OUTPUT); 
  pinMode (IO5_PIN, OUTPUT);
}

/**
 * Envia o pacote para a camada inferior
 */
inline void APP::send(packet * pkt) 
{    
  return;  
}

/**
 * Recebe o pacote da camada inferior
 */
inline void APP::receive(packet * pkt) 
{
  int AD0, AD1, AD2, AD3, AD4, AD5;
  if(pkt->MACHdr[3]==2)
  {
    digitalWrite (IO4_PIN, HIGH);
    delay(50000);
    digitalWrite (IO4_PIN, LOW);
    delay(50000);
    digitalWrite (IO4_PIN, HIGH);
    delay(50000);
    digitalWrite (IO4_PIN, LOW);
    pkt->MACHdr[3]= 222;
  }
    
   /* Envia para a camada inferior */
  Transp.send(pkt);
  //Phy.send(pkt);
  
  return;  
}
 
/* Instanciação do objeto de acesso à classe da camada de Aplicação */
APP App = APP();

