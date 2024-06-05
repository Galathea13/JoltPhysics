// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

// This is a very basic test that just drops a few objects on the floor
class SimpleTest : public Test
{
public:
	JPH_DECLARE_RTTI_VIRTUAL(JPH_NO_EXPORT, SimpleTest)

	// Destructor
	virtual				~SimpleTest() override;

	// See: Test
	virtual void		Initialize() override;

	//custom
	bool bMakeStatic = false;	//make static bodies or dynamic ?
	int bodyCount = 100;
	int kinematicCount = 6;
	RVec3 targetPos = RVec3(0.f, 10.f, 0.f);
	float targetCooldown = 1.f;
	float currentTargetCooldown = 0.f;
	float timerCounter = 0.f;	//general counter for debug
	std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen; // Standard mersenne_twister_engine seeded with rd()
	//std::vector<JPH::BodyID> bodies;
	std::vector<JPH::Body*> bodies;
	std::vector<JPH::Body*> kinematics;
	Vec3 camBodyScale {10.7f, 1.7f, 1.7f};
	JPH::Body* camBody;
	void SpawnBodies(int count);
	virtual void		PrePhysicsUpdate(const PreUpdateParams &inParams) override;
	static RVec3 RandomVec(std::mt19937& gen);

	void SetScale(const BodyID &bodyID, const Vec3& InScale);
	//end custom
private:
	// A demo of the activation listener
	class Listener : public BodyActivationListener
	{
	public:
		virtual void	OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
		{
			Trace("Body %d activated", inBodyID.GetIndex());
		}

		virtual void	OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
		{
			Trace("Body %d deactivated", inBodyID.GetIndex());
		}
	};

	Listener			mBodyActivationListener;
};
