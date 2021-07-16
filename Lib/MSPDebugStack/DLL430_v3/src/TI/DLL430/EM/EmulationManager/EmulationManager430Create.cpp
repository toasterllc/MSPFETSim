/*
 * EmulationManager430Create.cpp
 *
 * Creators for different EEM modules for MSP430
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


#include <pch.h>

#include "EmulationManager430.h"

#include "../TriggerManager/TriggerManager430.h"
#include "../TriggerCondition/TriggerConditionManager430.h"
#include "../BreakpointManager/BreakpointManager430.h"
#include "../CycleCounter/CycleCounter430.h"
#include "../StateStorage430/StateStorage430.h"
#include "../Sequencer/Sequencer430.h"
#include "../SoftwareBreakpoints/SoftwareBreakpoints430.h"


using namespace TI::DLL430;

namespace {

	class EmNone : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmNone> create()
		{
			std::shared_ptr<EmNone> em = std::make_shared<EmNone>();
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmSmall : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmSmall> create()
		{
			std::shared_ptr<EmSmall> em = std::make_shared<EmSmall>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(2, 0, 2, 0);
				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmMedium : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmMedium> create()
		{
			std::shared_ptr<EmMedium> em = std::make_shared<EmMedium>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(3, 0, 3, 0);
				em->mTriggerManager->setExtendedComparisons();

				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmLarge : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmLarge> create()
		{
			std::shared_ptr<EmLarge> em = std::make_shared<EmLarge>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(8, 2, 8, 7);
				em->mTriggerManager->setExtendedComparisons();
				em->mTriggerManager->setExtendedAccessTypes();
				em->mTriggerManager->setBitwiseMasking();

				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
				em->mSequencer = std::make_shared<Sequencer430>(em->mTriggerManager, false);
				em->mTrace = std::make_shared<StateStorage430>();
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmExtraSmall_5xx : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmExtraSmall_5xx> create()
		{
			std::shared_ptr<EmExtraSmall_5xx> em = std::make_shared<EmExtraSmall_5xx>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(2, 0, 2, 0);
				em->mTriggerManager->setExtendedAccessTypes();

				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
				em->mCycleCounter = std::make_shared<CycleCounter430>(1);
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmSmall_5xx : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmSmall_5xx> create()
		{
			std::shared_ptr<EmSmall_5xx> em = std::make_shared<EmSmall_5xx>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(3, 1, 4, 0);
				em->mTriggerManager->setExtendedComparisons();
				em->mTriggerManager->setExtendedAccessTypes();

				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
				em->mCycleCounter = std::make_shared<CycleCounter430>(1);
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmMedium_5xx : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmMedium_5xx> create()
		{
			std::shared_ptr<EmMedium_5xx> em = std::make_shared<EmMedium_5xx>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(5, 1, 6, 5);
				em->mTriggerManager->setExtendedComparisons();
				em->mTriggerManager->setExtendedAccessTypes();

				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
				em->mCycleCounter = std::make_shared<CycleCounter430>(1);
				em->mSequencer = std::make_shared<Sequencer430>(em->mTriggerManager, true);
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};


	class EmLarge_5xx : public EmulationManager430
	{
	public:
		static std::shared_ptr<EmLarge_5xx> create()
		{
			std::shared_ptr<EmLarge_5xx> em = std::make_shared<EmLarge_5xx>();
			if (em.get())
			{
				em->mTriggerManager = std::make_shared<TriggerManager430>(8, 2, 10, 7);
				em->mTriggerManager->setExtendedComparisons();
				em->mTriggerManager->setExtendedAccessTypes();
				em->mTriggerManager->setBitwiseMasking();

				em->mSoftwareBreakpoints = std::make_shared<SoftwareBreakpoints430>(em->mTriggerManager);
				em->mTriggerConditionManager = std::make_shared<TriggerConditionManager430>(em->mTriggerManager, em->mSoftwareBreakpoints);
				em->mBreakpointManager = std::make_shared<BreakpointManager430>();
				em->mCycleCounter = std::make_shared<CycleCounter430>(2);
				em->mSequencer = std::make_shared<Sequencer430>(em->mTriggerManager, false);

				std::shared_ptr<StateStorage430> stateStorage = std::make_shared<StateStorage430>();
				em->mTrace = stateStorage;
				em->mVariableWatch = stateStorage;
			}
			return em;
		}

		virtual void reset() { *this = *create(); }
	};
}


EmulationManagerPtr EmulationManager430::create(uint8_t emulationLevel)
{
	switch (emulationLevel)
	{
	case 0: return EmNone::create();
	case 1: return EmSmall::create();
	case 2: return EmMedium::create();
	case 3: return EmLarge::create();
	case 4: return EmExtraSmall_5xx::create();
	case 5: return EmSmall_5xx::create();
	case 6: return EmMedium_5xx::create();
	case 7: return EmLarge_5xx::create();
	default: break;
	}

	return EmNone::create();
}
