/*
* Copyright (c) 2015 SODAQ. All rights reserved.
*
* This file is part of MicrochipLoRaWAN.
*
* MicrochipLoRaWAN is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or(at your option) any later version.
*
* MicrochipLoRaWAN is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with MicrochipLoRaWAN.  If not, see
* <http://www.gnu.org/licenses/>.
*/

#include "Switchable_Device.h"

SwitchableDevice::SwitchableDevice()
{
	clearSwitchMethods();
}

void SwitchableDevice::setOnMethod(voidFuncPtr onMethod)
{
	_onMethod = onMethod;
}

void SwitchableDevice::setOffMethod(voidFuncPtr offMethod)
{
	_offMethod = offMethod;
}

void SwitchableDevice::setSwitchMethods(voidFuncPtr onMethod, voidFuncPtr offMethod)
{
	_onMethod = onMethod;
	_offMethod = offMethod;
}

void SwitchableDevice::clearSwitchMethods()
{
	_onMethod = _offMethod = 0;
}

void SwitchableDevice::on()
{
	if (_onMethod != 0)
	{
		_onMethod();
	}
}

void SwitchableDevice::off()
{
	if (_offMethod != 0)
	{
		_offMethod();
	}
}
