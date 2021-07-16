/**
 * \ingroup MODULMACROS
 *
 * \file DummyHal430.c
 *
 * \brief Set certain parameters to spcified values
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "edt.h"
#include "hal.h"

#ifdef MSP430_UIF
    HAL_FUNCTION(_hal_BlowFuse)                          {return  -1;}
    HAL_FUNCTION(_hal_StartJtagActivationCode)           {return  -1;}
    HAL_FUNCTION(_hal_WaitForEem)                        {return  -1;}
    HAL_FUNCTION(_hal_GetJtagId)                         {return  -1;}
    HAL_FUNCTION(_hal_SetDeviceChainInfo)                {return  -1;}
    HAL_FUNCTION(_hal_SetChainConfiguration)             {return  -1;}
    HAL_FUNCTION(_hal_GetNumOfDevices)                   {return  -1;}
    HAL_FUNCTION(_hal_GetDeviceIdPtr)                    {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContext)    {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContext)  {return  -1;}
    HAL_FUNCTION(_hal_RestoreContext_ReleaseJtag)        {return  -1;}
    HAL_FUNCTION(_hal_ReadMemBytes)                      {return  -1;}
    HAL_FUNCTION(_hal_ReadMemWords)                      {return  -1;}
    HAL_FUNCTION(_hal_ReadMemQuick)                      {return  -1;}
    HAL_FUNCTION(_hal_WriteMemBytes)                     {return  -1;}
    HAL_FUNCTION(_hal_WriteMemWords)                     {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchange)                   {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchangeAFE2xx)             {return  -1;}
    HAL_FUNCTION(_hal_SingleStep)                        {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegs)                    {return  -1;}
    HAL_FUNCTION(_hal_WriteAllCpuRegs)                   {return  -1;}
    HAL_FUNCTION(_hal_Psa)                               {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFunclet)                    {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFuncletJtag)                {return  -1;}
    HAL_FUNCTION(_hal_GetDcoFrequency)                   {return  -1;}
    HAL_FUNCTION(_hal_GetDcoFrequencyJtag)               {return  -1;}
    HAL_FUNCTION(_hal_GetFllFrequency)                   {return  -1;}
    HAL_FUNCTION(_hal_GetFllFrequencyJtag)               {return  -1;}
    HAL_FUNCTION(_hal_WaitForStorage)                    {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextX)   {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextX) {return  -1;}
    HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagX)       {return  -1;}
    HAL_FUNCTION(_hal_ReadMemBytesX)                     {return  -1;}
    HAL_FUNCTION(_hal_ReadMemWordsX)                     {return  -1;}
    HAL_FUNCTION(_hal_ReadMemQuickX)                     {return  -1;}
    HAL_FUNCTION(_hal_WriteMemBytesX)                    {return  -1;}
    HAL_FUNCTION(_hal_WriteMemWordsX)                    {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchangeX)                  {return  -1;}
    HAL_FUNCTION(_hal_SingleStepX)                       {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegsX)                   {return  -1;}
    HAL_FUNCTION(_hal_WriteAllCpuRegsX)                  {return  -1;}
    HAL_FUNCTION(_hal_PsaX)                              {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFuncletX)                   {return  -1;}
    HAL_FUNCTION(_hal_GetDcoFrequencyX)                  {return  -1;}
    HAL_FUNCTION(_hal_GetFllFrequencyX)                  {return  -1;}
    HAL_FUNCTION(_hal_WaitForStorageX)                   {return  -1;}
    HAL_FUNCTION(_hal_BlowFuseXv2)                       {return  -1;}
    HAL_FUNCTION(_hal_BlowFuseFram)                      {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextXv2) {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextXv2){return  -1;}
    HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagXv2)      {return  -1;}
    HAL_FUNCTION(_hal_ReadMemWordsXv2)                    {return  -1;}
    HAL_FUNCTION(_hal_ReadMemQuickXv2)                    {return  -1;}
    HAL_FUNCTION(_hal_WriteMemWordsXv2)                   {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchangeXv2)                 {return  -1;}
    HAL_FUNCTION(_hal_SingleStepXv2)                      {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegsXv2)                  {return  -1;}
    HAL_FUNCTION(_hal_WriteAllCpuRegsXv2)                 {return  -1;}
    HAL_FUNCTION(_hal_PsaXv2)                             {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFuncletXv2)                  {return  -1;}
    HAL_FUNCTION(_hal_UnlockDeviceXv2)                    {return  -1;}
    HAL_FUNCTION(_hal_MagicPattern)                       {return  -1;}
    HAL_FUNCTION(_hal_UnlockC092)                         {return  -1;}
    HAL_FUNCTION(_hal_PollJStateReg)                      {return  -1;}
    HAL_FUNCTION(_hal_PollJStateRegFR57xx)                {return  -1;}
    HAL_FUNCTION(_hal_IsJtagFuseBlown)                    {return  -1;}
    HAL_FUNCTION(_hal_ResetXv2)                           {return  -1;}
    HAL_FUNCTION(_hal_WriteFramQuickXv2)                  {return  -1;}
    HAL_FUNCTION(_hal_SendJtagMailboxXv2)                 {return  -1;}
    HAL_FUNCTION(_hal_SingleStepJStateXv2)                {return  -1;}
    HAL_FUNCTION(_hal_PollJStateRegEt8)                   {return  -1;}
    HAL_FUNCTION(_hal_Reset430I)                          {return  -1;}
    HAL_FUNCTION(_hal_PollJStateReg430I)                  {return  -1;}
    HAL_FUNCTION(_hal_PollJStateReg20)                    {return  -1;}
    HAL_FUNCTION(_hal_ResetL092)                          {return  -1;}
    HAL_FUNCTION(_hal_Reset5438Xv2)                       {return  -1;}
    HAL_FUNCTION(_hal_LeaSyncConditional)                 {return  -1;}
    HAL_FUNCTION(_hal_PollDStatePCRegEt)                  {return  -1;}
    HAL_FUNCTION(_hal_UssSyncConditional)                 {return  -1;}
#endif

