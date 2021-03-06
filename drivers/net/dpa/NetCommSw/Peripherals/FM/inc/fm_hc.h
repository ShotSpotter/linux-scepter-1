/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FM_HC_H
#define __FM_HC_H

#include "std_ext.h"
#include "error_ext.h"


#define __ERR_MODULE__  MODULE_FM_PCD


typedef struct t_FmHcParams {
    t_Handle        h_Fm;
    t_Handle        h_FmPcd;
    t_FmPcdHcParams params;
} t_FmHcParams;


t_Handle    FmHcConfigAndInit(t_FmHcParams *p_FmHcParams);
void        FmHcFree(t_Handle h_FmHc);
t_Error     FmHcDumpRegs(t_Handle h_FmHc);

void        FmHcTxConf(t_Handle h_FmHc, t_DpaaFD *p_Fd);

t_Handle    FmHcPcdKgSetScheme(t_Handle h_FmHc, t_FmPcdKgSchemeParams *p_Scheme);
t_Error     FmHcPcdKgDeleteScheme(t_Handle h_FmHc, t_Handle h_Scheme);
t_Error     FmHcPcdCcCapwapTimeoutReassm(t_Handle h_FmHc, t_FmPcdCcCapwapReassmTimeoutParams *p_CcCapwapReassmTimeoutParams );
#ifdef UNDER_CONSTRUCTION_FRAG_REASSEMBLY
t_Error     FmHcPcdCcIpTimeoutReassm(t_Handle h_FmHc, t_FmPcdCcIpReassmTimeoutParams *p_CcIpReassmTimeoutParams );
t_Error     FmHcPcdCcIpFrag(t_Handle h_FmHc, t_FmPcdCcIpFragInitParams *p_CcIpFragInitialization);
#endif /*UNDER_CONSTRUCTION_FRAG_REASSEMBLY*/
t_Error     FmHcPcdKgSetClsPlan(t_Handle h_FmHc, t_FmPcdKgInterModuleClsPlanSet *p_Set);
//t_Handle    FmHcPcdKgSetClsPlanGrp(t_Handle h_FmHc, t_FmPcdKgInterModuleClsPlanGrpParams *p_Grp);
t_Error     FmHcPcdKgDeleteClsPlan(t_Handle h_FmHc, uint8_t clsPlanGrpId);

t_Error     FmHcPcdKgSetSchemeCounter(t_Handle h_FmHc, t_Handle h_Scheme, uint32_t value);
uint32_t    FmHcPcdKgGetSchemeCounter(t_Handle h_FmHc, t_Handle h_Scheme);

t_Error     FmHcPcdCcModifyTreeNextEngine(t_Handle h_FmHc, t_Handle h_CcTree, uint8_t grpId, uint8_t index, t_FmPcdCcNextEngineParams *p_FmPcdCcNextEngineParams);
t_Error     FmHcPcdCcModifyNodeNextEngine(t_Handle h_FmHc, t_Handle h_CcNode, uint8_t keyIndex, t_FmPcdCcNextEngineParams *p_FmPcdCcNextEngineParams);
t_Error     FmHcPcdCcModifyNodeMissNextEngine(t_Handle h_FmHc, t_Handle h_CcNode, t_FmPcdCcNextEngineParams *p_FmPcdCcNextEngineParams);
t_Error     FmHcPcdCcRemoveKey(t_Handle h_FmHc, t_Handle h_CcNode, uint8_t keyIndex);
t_Error     FmHcPcdCcAddKey(t_Handle h_FmHc, t_Handle h_CcNode, uint8_t keyIndex, uint8_t keySize, t_FmPcdCcKeyParams  *p_KeyParams);
t_Error     FmHcPcdCcModifyKeyAndNextEngine(t_Handle h_FmHc, t_Handle h_CcNode, uint8_t keyIndex, uint8_t keySize, t_FmPcdCcKeyParams  *p_KeyParams);
t_Error     FmHcPcdCcModifyKey(t_Handle h_FmHc, t_Handle h_CcNode, uint8_t keyIndex, uint8_t keySize, uint8_t  *p_Key, uint8_t *p_Mask);

t_Handle    FmHcPcdPlcrSetProfile(t_Handle h_FmHc,t_FmPcdPlcrProfileParams *p_Profile);
t_Error     FmHcPcdPlcrDeleteProfile(t_Handle h_FmHc, t_Handle h_Profile);

t_Error     FmHcPcdPlcrSetProfileCounter(t_Handle h_FmHc, t_Handle h_Profile, e_FmPcdPlcrProfileCounters counter, uint32_t value);
uint32_t    FmHcPcdPlcrGetProfileCounter(t_Handle h_FmHc, t_Handle h_Profile, e_FmPcdPlcrProfileCounters counter);

t_Error     FmHcKgWriteSp(t_Handle h_FmHc, uint8_t hardwarePortId, uint32_t spReg, bool add);
t_Error     FmHcKgWriteCpp(t_Handle h_FmHc, uint8_t hardwarePortId, uint32_t cppReg);

t_Error     FmHcPcdKgCcGetSetParams(t_Handle h_FmHc, t_Handle  h_Scheme, uint32_t requiredAction);
t_Error     FmHcPcdPlcrCcGetSetParams(t_Handle h_FmHc,uint16_t absoluteProfileId, uint32_t requiredAction);

#ifdef FM_RMU_ASSIST_SUPPORT
t_Error     FmHcRmuDoorbellInitCmd(t_Handle h_FmHc, t_FmRmuDrblInitHcParams *p_DrblInitHcParams);
t_Error     FmHcRmuDoorbellSetCntsCmd(t_Handle h_FmHc, t_FmRmuDrblSetCntsHcParams *p_DrblSetCntsHcParams);
t_Error     FmHcRmuDoorbellFreeCmd(t_Handle h_FmHc);

t_Error     FmHcRmuMsgInitCmd(t_Handle h_FmHc, t_FmRmuMsgInitHcParams *p_MsgInitHcParams);
t_Error     FmHcRmuBufferProfileChange(t_Handle h_FmHc, t_FmRmuBufferProfileChangeHcParams *p_HcParams);
t_Error     FmHcRmuMsgClassChange(t_Handle h_FmHc, t_FmRmuMsgClassChangeHcParams *p_HcParams);
t_Error     FmHcRmuMsgFreeCmd(t_Handle h_FmHc, uint8_t  mailboxId);
#endif /* FM_RMU_ASSIST_SUPPORT */

#endif /* __FM_HC_H */
