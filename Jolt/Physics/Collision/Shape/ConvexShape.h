// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Core/StaticArray.h>
#include <Physics/Collision/Shape/Shape.h>
#include <Physics/Collision/Shape/SubShapeID.h>
#include <Physics/Collision/PhysicsMaterial.h>
#include <unordered_map>
#ifdef JPH_DEBUG_RENDERER
	#include <Renderer/DebugRenderer.h>
#endif // JPH_DEBUG_RENDERER

namespace JPH {

class CollideShapeSettings;

/// Class that constructs a ConvexShape (abstract)
class ConvexShapeSettings : public ShapeSettings
{
public:
	JPH_DECLARE_SERIALIZABLE_ABSTRACT(ConvexShapeSettings)

	/// Constructor
									ConvexShapeSettings() = default;
									ConvexShapeSettings(const PhysicsMaterial *inMaterial)		: mMaterial(inMaterial) { }

	/// Set the density of the object in kg / m^3
	void							SetDensity(float inDensity)									{ mDensity = inDensity; }

	// Properties
	RefConst<PhysicsMaterial>		mMaterial;													///< Material assigned to this shape
	float							mDensity = 1000.0f;											///< Uniform density of the interior of the convex object (kg / m^3)
};

/// Base class for all convex shapes. Defines a virtual interface.
class ConvexShape : public Shape
{
public:
	JPH_DECLARE_RTTI_ABSTRACT(ConvexShape)

	/// Constructor
									ConvexShape() = default;
									ConvexShape(const ConvexShapeSettings &inSettings, ShapeResult &outResult) : Shape(inSettings, outResult), mMaterial(inSettings.mMaterial), mDensity(inSettings.mDensity) { }
									ConvexShape(const PhysicsMaterial *inMaterial) : mMaterial(inMaterial) { }

	// Get type
	virtual EShapeType				GetType() const override									{ return EShapeType::Convex; }

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override					{ return 0; } // Convex shapes don't have sub shapes

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override	{ JPH_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); return GetMaterial(); }

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector) const override;

	// See Shape::CastShape
	virtual void					CastShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy) const override;

	/// Function that provides an interface for GJK
	class Support
	{
	public:
		/// Warning: Virtual destructor will not be called on this object!
		virtual						~Support() { }

		/// Calculate the support vector for this convex shape (includes / excludes the convex radius depending on how this was obtained). 
		/// Support vector is relative to the center of mass of the shape.
		virtual Vec3				GetSupport(Vec3Arg inDirection) const = 0;

		/// Convex radius of shape. Collision detection on penetrating shapes is much more expensive, 
		/// so you can add a radius around objects to increase the shape. This makes it far less likely that they will actually penetrate.
		virtual float				GetConvexRadius() const = 0;
	};

	/// Buffer to hold a Support object, used to avoid dynamic memory allocations
	class alignas(16) SupportBuffer
	{
	public:
		uint8						mData[4160];
	};

	/// How the GetSupport function should behave
	enum class ESupportMode	
	{
		ExcludeConvexRadius,		///< Return the shape excluding the convex radius
		IncludeConvexRadius,		///< Return the shape including the convex radius
	};

	/// Returns an object that provides the GetSupport function for this shape.
	/// inMode determines if this support function includes or excludes the convex radius.
	/// of the values returned by the GetSupport function. This improves numerical accuracy of the results.
	/// inScale scales this shape in local space.
	virtual const Support *			GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const = 0;

	/// Type definition for a supporting face
	using SupportingFace = StaticArray<Vec3, 32>;

	/// Get the vertices of the face that faces inDirection the most (includes convex radius).
	/// Face is relative to the center of mass of the shape.
	virtual void					GetSupportingFace(Vec3Arg inDirection, Vec3Arg inScale, SupportingFace &outVertices) const = 0;

	/// Material of the shape
	void							SetMaterial(const PhysicsMaterial *inMaterial)				{ mMaterial = inMaterial; }
	const PhysicsMaterial *			GetMaterial() const											{ return mMaterial != nullptr? mMaterial : PhysicsMaterial::sDefault; }

	/// Set density of the shape (kg / m^3)
	void							SetDensity(float inDensity)									{ mDensity = inDensity; }

	/// Get density of the shape (kg / m^3)
	float							GetDensity() const											{ return mDensity; }

	/// Collide 2 shapes and report any hits to ioCollector.
	static void						sCollideConvexVsConvex(const ConvexShape *inShape1, const ConvexShape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector);

#ifdef JPH_DEBUG_RENDERER
	// See Shape::DrawGetSupportFunction
	virtual void					DrawGetSupportFunction(DebugRenderer *inRenderer, Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const override;

	// See Shape::DrawGetSupportingFace
	virtual void					DrawGetSupportingFace(DebugRenderer *inRenderer, Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
#endif // JPH_DEBUG_RENDERER

#ifdef JPH_STAT_COLLECTOR
	/// Reset stats collected during the previous time step
	static void						sResetStats();

	/// Collect stats of the previous time step
	static void						sCollectStats();
#endif // JPH_STAT_COLLECTOR

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;
	virtual void					SaveMaterialState(PhysicsMaterialList &outMaterials) const override;
	virtual void					RestoreMaterialState(const PhysicsMaterialList &inMaterials) override;

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

	/// Vertex list that forms a unit sphere
	static const vector<Vec3>		sUnitSphereTriangles;

private:
	// Class for GetTrianglesStart/Next
	class							CSGetTrianglesContext;

#ifdef JPH_STAT_COLLECTOR
	// Statistics
	alignas(JPH_CACHE_LINE_SIZE) static atomic<int>	sNumCollideChecks;
	alignas(JPH_CACHE_LINE_SIZE) static atomic<int>	sNumGJKChecks;
	alignas(JPH_CACHE_LINE_SIZE) static atomic<int>	sNumEPAChecks;
	alignas(JPH_CACHE_LINE_SIZE) static atomic<int>	sNumCollisions;
#endif // JPH_STAT_COLLECTOR

	// Properties
	RefConst<PhysicsMaterial>		mMaterial;													///< Material assigned to this shape
	float							mDensity = 1000.0f;											///< Uniform density of the interior of the convex object (kg / m^3)

#ifdef JPH_DEBUG_RENDERER
	mutable unordered_map<Vec3, DebugRenderer::GeometryRef> mGetSupportFunctionGeometry;
#endif // JPH_DEBUG_RENDERER
};

} // JPH