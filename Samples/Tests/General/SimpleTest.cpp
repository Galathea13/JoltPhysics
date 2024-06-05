// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/General/SimpleTest.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Layers.h>
#include <random>
#include "Jolt/Physics/Collision/Shape/ScaledShape.h"
#include <iostream>

JPH_IMPLEMENT_RTTI_VIRTUAL(SimpleTest)
{
	JPH_ADD_BASE_CLASS(SimpleTest, Test)
}

SimpleTest::~SimpleTest()
{
	// Unregister activation listener
	mPhysicsSystem->SetBodyActivationListener(nullptr);
}

void SimpleTest::Initialize()
{
	// Register activation listener
	mPhysicsSystem->SetBodyActivationListener(&mBodyActivationListener);

	// Floor
	CreateFloor(5000.f);

	RefConst<Shape> box_shape = new BoxShape(Vec3(0.5f, 1.0f, 2.0f));

	// Dynamic body 1
	Body &body1 = *mBodyInterface->CreateBody(BodyCreationSettings(box_shape, RVec3(0, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING));
	mBodyInterface->AddBody(body1.GetID(), EActivation::Activate);
	
	// Dynamic body 2
	Body &body2 = *mBodyInterface->CreateBody(BodyCreationSettings(box_shape, RVec3(5, 10, 0), Quat::sRotation(Vec3::sAxisX(), 0.25f * JPH_PI), EMotionType::Dynamic, Layers::MOVING));
	mBodyInterface->AddBody(body2.GetID(), EActivation::Activate);

	// Dynamic body 3
	Body &body3 = *mBodyInterface->CreateBody(BodyCreationSettings(new SphereShape(2.0f), RVec3(10, 10, 0), Quat::sRotation(Vec3::sAxisX(), 0.25f * JPH_PI), EMotionType::Dynamic, Layers::MOVING));
	mBodyInterface->AddBody(body3.GetID(), EActivation::Activate);

	bMakeStatic = false;

	SpawnBodies(100);
}

//-------------------------------------------------------------------------------------------------
void SimpleTest::SpawnBodies(int count)
{
	//todo : create shape for target
	gen = std::mt19937(rd());
	//float size = 2.f;
	//RefConst<Shape> box_shape = new BoxShape(Vec3(0.5f, 0.5f,0.5f));
	//RefConst<Shape> box_shape = new SphereShape(0.6f);
	RefConst<Shape> box_shape;
	box_shape = new BoxShape(Vec3(0.5f, 0.5f,0.5f));
	//box_shape = new SphereShape(0.6f);;
	RefConst<Shape> cam_shape = new BoxShape(Vec3(10.7f, 1.7f, 1.7f));;
	float spawnRange = 1000.f;
	
	camBody = mBodyInterface->CreateBody(BodyCreationSettings(cam_shape, RVec3(0, 10, 0), Quat::sIdentity(), EMotionType::Kinematic, Layers::MOVING));
	mBodyInterface->AddBody(camBody->GetID(), EActivation::Activate);

	
	const EMotionType motionType = bMakeStatic ? EMotionType::Static : EMotionType::Dynamic;

	const bool bSpawn = true;
	for (int i = 0; i < count && bSpawn; ++i)
	{
		////float randy = spawnRange * rand() / RAND_MAX;
		//float randSpeedx = 100.f * ((rand() / RAND_MAX)-0.5f) *2.f;
		//float randSpeedz = 100.f * ((rand() / RAND_MAX)-0.5f) *2.f;
		//float randx = spawnRange * ((rand() / RAND_MAX)-0.5f) *2.f;
		//float randz = spawnRange * ((rand() / RAND_MAX)-0.5f) *2.f;


		//Vec3 randPos = Vec3::sRandom(gen) * spawnRange;
		std::uniform_real_distribution<double> zero_to_onehundred(-100., 100.);
		std::uniform_real_distribution<double> zero_to_ten(10., 100.);

		RVec3 randPos = RVec3(zero_to_onehundred(gen), zero_to_ten(gen), zero_to_onehundred(gen));
		Vec3 randSpeed =  Vec3::sRandom(gen) * spawnRange;
		//BodyCreationSettings settings(box_shape, RVec3(randx, 10, randz), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
		
		BodyCreationSettings settings(box_shape, randPos, Quat::sIdentity(), motionType, Layers::MOVING);
		settings.mRestitution = 0.f;
		settings.mGravityFactor = 0.f;
		Body* body = mBodyInterface->CreateBody(settings);


		//if(!bMakeStatic)
		//	body->AddForce(randSpeed * spawnRange);
		mBodyInterface->AddBody(body->GetID(), EActivation::Activate);

		bodies.emplace_back(body);
	}
}

//-------------------------------------------------------------------------------------------------
#pragma optimize("", off)
void SimpleTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{

	//Trace("testing simpleTest");
	float speed = 400.f;
	float noiseAmount = 0.f;

	//move cam	
	RVec3 camFwd((double)inParams.mCameraState.mForward.GetX(),
				 (double)inParams.mCameraState.mForward.GetY(),
				 (double)inParams.mCameraState.mForward.GetZ());
	RVec3 camPos = inParams.mCameraState.mPos + camFwd * 40.;


	Quat rot = mBodyInterface->GetRotation(camBody->GetID());
	//rot = rot * Quat::sEulerAngles(Vec3(0.f, 180.f, 0.f));
	rot = Quat::sIdentity();
	//Vec3 NewScale = camBodyScale;
	Vec3 NewScale = std::clamp((std::sin(timerCounter*5.f)+2.f), 0.5f, 1.5f) * Vec3::sReplicate(1.f);
	//Vec3 NewScale = Vec3(1.1f,1.1f ,1.1f ) ;
	NewScale.sFixW(NewScale.mValue);

	//std::cout <<NewScale.GetX() << "," <<NewScale.GetY() << "," <<NewScale.GetZ() << "\n" ;

	mBodyInterface->SetShape(camBody->GetID(), new ScaledShape(camBody->GetShape(), {1.f,1.f,1.f}) , false, EActivation::Activate);
	if(timerCounter >1.f)
		SetScale(camBody->GetID(), Vec3::sReplicate(1.1f));
	else
		SetScale(camBody->GetID(), Vec3::sReplicate(1.f)/Vec3::sReplicate(1.1f));
	/*Vec3 scaleOffset {1.f,1.f,1.f};
	const ScaledShape* scaleShape = static_cast<const ScaledShape*>(camBody->GetShape());
	if(scaleShape)
		scaleOffset = scaleShape->GetScale()/camBodyScale;

	mBodyInterface->SetShape(camBody->GetID(), new ScaledShape(camBody->GetShape(), NewScale/scaleOffset) , false, EActivation::Activate);
	scaleShape = (ScaledShape*)camBody->GetShape();
	Vec3 postScale = scaleShape->GetScale();*/


	mBodyInterface->MoveKinematic(camBody->GetID(), camPos, rot, inParams.mDeltaTime *10.f );
	for (size_t i = 0; i < bodies.size() && !bMakeStatic; ++i)
	{
		currentTargetCooldown += inParams.mDeltaTime;
		if(currentTargetCooldown > targetCooldown)
		{
			currentTargetCooldown = 0.f;
			std::uniform_real_distribution<float> zero_to_one(-100.0f, 100.0f);
			targetPos.SetX(zero_to_one(gen));
			targetPos.SetY(zero_to_one(gen));
		}

		//float targetSpeed = speed * /*targetDir **/ inParams.mDeltaTime *0.f;
		Vec3 noiseVec = Vec3::sRandom(gen) * noiseAmount * inParams.mDeltaTime;
		noiseVec.SetY(0.f);
		//targetPos += RVec3(targetSpeed, 0.f, targetSpeed) + noiseVec;
		RVec3 pos = bodies[i]->GetPosition();
		RVec3 dir = (targetPos - pos).Normalized() * speed;
		bodies[i]->AddImpulse(Vec3(static_cast<float>(dir.GetX()),
								   static_cast<float>(dir.GetY()),
								   static_cast<float>(dir.GetZ())
		));

		//std::uniform_real_distribution<float> zero_to_one(0.0f, 1000.0f);
		//float test = zero_to_one(gen);
		//if(test < 0.001f)
		//	bodies[i]->AddImpulse(dir * 10000.f);
	}


	//update debug counter
	timerCounter += inParams.mDeltaTime;
	if(timerCounter > 2.f)
		timerCounter = 0.f;
}
#pragma optimize("", on)


JPH::RVec3 SimpleTest::RandomVec(std::mt19937& gen)
{
	std::uniform_real_distribution<float> zero_to_one(0.0f, 1.0f);
	float theta = JPH_PI * zero_to_one(gen);
	float phi = 2.0f * JPH_PI * zero_to_one(gen);
	Vec4 s, c;
	Vec4(theta, phi, 0, 0).SinCos(s, c);
	return RVec3(s.GetX() * c.GetY(), s.GetX() * s.GetY(), c.GetX());
}


//-------------------------------------------------------------------------------------------------
void SimpleTest::SetScale(const BodyID &bodyID, const Vec3& InScale)
{

	
	JPH_ASSERT(!bodyID.IsInvalid());
	const Shape* shape = mBodyInterface->GetShape(bodyID);
	const ScaledShape* scaledShape = static_cast<const ScaledShape*>(shape);

	Vec3 previousScale (1.f, 1.f, 1.f);
	if(!scaledShape)	return;

	previousScale = scaledShape->GetScale();

	Shape::ShapeResult res;

	{
		BodyLockWrite lock(mPhysicsSystem->GetBodyLockInterface(), bodyID);
		if (lock.Succeeded())
		{
			const Shape* innerShape = scaledShape->GetInnerShape();
			if(innerShape == nullptr)	return;
			res = shape->ScaleShape(InScale);
			JPH_ASSERT(res.IsValid());
			mPhysicsSystem->GetBodyInterfaceNoLock().SetShape(bodyID, res.Get(), true, EActivation::Activate);
			scaledShape = static_cast<const ScaledShape*>(mPhysicsSystem->GetBodyInterfaceNoLock().GetShape(bodyID).GetPtr());
			Vec3 newScale = scaledShape->GetScale();
			std::cout << "Scaling to " <<InScale.GetX() << "," <<InScale.GetY() << "," <<InScale.GetZ() << "\n" ;
			std::cout << "Result : " << newScale.GetX() << "," <<newScale.GetY() << "," <<newScale.GetZ() << "\n" ;
			std::cout << "previous Scale : " << previousScale.GetX() << "," <<previousScale.GetY() << "," <<previousScale.GetZ() << "\n-----\n" ;


		}
	}
	//mBodyInterface->SetShape(bodyID, res.Get(), true, EActivation::Activate);
	/*Vec3 deltaScale = InScale - previousScale + Vec3(1.f, 1.f, 1.f);
	Vec3 deltaScale = Vec3( InScale.GetX() > previousScale.GetX() ? 1.f
							);*/
	//Vec3 deltaScale = InScale  / previousScale;

	//mBodyInterface->SetShape(bodyID, new ScaledShape(shape, Vec3{1.f,1.f,1.f}/Vec3{1.1f,1.1f,1.1f}) , false, EActivation::Activate);
	//mBodyInterface->SetShape(camBody->GetID(), new ScaledShape(shape, camBodyScale / previousScale) , false, EActivation::Activate);
	//Vec3 deltaScale = InScale * camBodyScale / previousScale;
	//if((deltaScale - previousScale).IsNearZero())
	//	return;
	//mBodyInterface->SetShape(bodyID, new ScaledShape(shape, deltaScale) , false, EActivation::Activate);
	//scaledShape = static_cast<const ScaledShape*>(mBodyInterface->GetShape(bodyID).GetPtr());
	//Vec3 postScale = scaledShape->GetScale();
	
}
