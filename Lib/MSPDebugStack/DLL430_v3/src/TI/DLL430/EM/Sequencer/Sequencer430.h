/*
 * Sequencer430.h
 *
 * Sequencer implementation for 430
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#pragma once

#include "ISequencer.h"
#include "../TriggerManager/TriggerManager430.h"


namespace TI { namespace DLL430 {


class Sequencer430 : public ISequencer
{
public:
	Sequencer430(TriggerManager430Ptr triggerManager, bool sixTriggerEem);

	virtual void writeConfiguration();

	virtual uint32_t readCurrentState() const;

	virtual void reset();

	virtual void setResetTrigger(TriggerConditionPtr triggerCondition);

	virtual void clearResetTrigger();

	virtual void setTransition(uint32_t state, uint32_t transition, uint32_t nextState,
 						 	TriggerConditionPtr triggerCondition);

	virtual void clearTransition(uint32_t state, uint32_t transition);

	virtual void clearAllTransitions();


	virtual void addReaction(TriggerReaction reaction);
	virtual void removeReaction(TriggerReaction reaction);
	virtual void clearReactions();

	virtual void combine(TriggerConditionPtr condition) {/*Not supported*/}
	virtual uint32_t getId() const {/*Not supported*/ return 7; }

	virtual void enable();
	virtual void disable();
	virtual bool isEnabled() const;

private:
	static const uint32_t NUM_STATES = 4;
	static const uint32_t NUM_TRANSITIONS = 2;

	struct State
	{
		State() { std::fill_n(nextState, NUM_TRANSITIONS, 0); }
		uint8_t nextState[NUM_TRANSITIONS];
		TriggerConditionPtr trigger[NUM_TRANSITIONS];
	};

	TriggerConditionPtr resetTrigger_;

	std::vector<State> states_;

	TriggerManager430Ptr triggerManager_;

	uint16_t seqCntrl_;
	uint16_t nxtState0_;
	uint16_t nxtState1_;

	bool sixTriggerEem_;
};

}}
