// Copyright (c) 2014 Cyberith GmbH
#pragma once

/**
*
*/
class CVirtDevice
{
public:
	CVirtDevice();
	~CVirtDevice();

	virtual bool Open() = 0;
	virtual bool IsOpen() = 0;
	virtual bool Close() = 0;

	virtual float GetPlayerHeight() = 0;
	virtual void ResetPlayerHeight() = 0;
	virtual float GetPlayerOrientation() = 0;
	virtual float GetMovementSpeed() = 0;
	virtual float GetMovementDirection() = 0;
	virtual void ResetPlayerOrientation() = 0;
};
