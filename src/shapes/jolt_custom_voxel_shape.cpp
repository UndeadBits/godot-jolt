#include "jolt_custom_voxel_shape.hpp"

#include <Jolt/Geometry/OrientedBox.h>

namespace {

JPH::Shape* construct_empty() {
	return new JoltCustomVoxelShape();
}

void collide_noop(
	[[maybe_unused]] const JPH::Shape* p_shape1,
	[[maybe_unused]] const JPH::Shape* p_shape2,
	[[maybe_unused]] JPH::Vec3Arg p_scale1,
	[[maybe_unused]] JPH::Vec3Arg p_scale2,
	[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform1,
	[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform2,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	[[maybe_unused]] const JPH::CollideShapeSettings& p_collide_shape_settings,
	[[maybe_unused]] JPH::CollideShapeCollector& p_collector,
	[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter
) { }

void cast_noop(
	[[maybe_unused]] const JPH::ShapeCast& p_shape_cast,
	[[maybe_unused]] const JPH::ShapeCastSettings& p_shape_cast_settings,
	[[maybe_unused]] const JPH::Shape* p_shape,
	[[maybe_unused]] JPH::Vec3Arg p_scale,
	[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter,
	[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform2,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	[[maybe_unused]] JPH::CastShapeCollector& p_collector
) { }

} // namespace

JPH::ShapeSettings::ShapeResult JoltCustomVoxelShapeSettings::Create() const {
	if (mCachedResult.IsEmpty()) {
		new JoltCustomVoxelShape(*this, mCachedResult);
	}

	return mCachedResult;
}

JPH::AABox JoltCustomVoxelShape::GetLocalBounds() const {
	return {JPH::Vec3(), this->size};
}

float JoltCustomVoxelShape::GetInnerRadius() const {
	return this->size.ReduceMin();
};

JPH::MassProperties JoltCustomVoxelShape::GetMassProperties() const {
	JPH::MassProperties p;
	// TODO: Calculate correct mass and intertia
	p.SetMassAndInertiaOfSolidBox(this->size, 1000.0f);
	return p;
}

void JoltCustomVoxelShape::SaveBinaryState(JPH::StreamOut& inStream) const {
	Shape::SaveBinaryState(inStream);

	inStream.Write(this->size.GetX());
	inStream.Write(this->size.GetY());
	inStream.Write(this->size.GetZ());
	inStream.Write(this->voxels.size());
	inStream.WriteBytes(this->voxels.ptr(), static_cast<size_t>(this->voxels.size()));
}

void JoltCustomVoxelShape::RestoreBinaryState(JPH::StreamIn& inStream) {
	Shape::RestoreBinaryState(inStream);

	float x, y, z;
	int64_t numVoxels;

	inStream.Read(x);
	inStream.Read(y);
	inStream.Read(z);
	inStream.Read(size);

	this->size = JPH::Vec3(x, y, z);
	this->voxels.resize(numVoxels);

	inStream.ReadBytes(this->voxels.ptrw(), static_cast<size_t>(numVoxels));
}

void JoltCustomVoxelShape::CollideSphereVsVoxelBox(
	[[maybe_unused]] const JPH::Shape* inShape1,
	[[maybe_unused]] const JPH::Shape* inShape2,
	[[maybe_unused]] JPH::Vec3Arg inScale1,
	[[maybe_unused]] JPH::Vec3Arg inScale2,
	[[maybe_unused]] JPH::Mat44Arg inCenterOfMassTransform1,
	[[maybe_unused]] JPH::Mat44Arg inCenterOfMassTransform2,
	[[maybe_unused]] const JPH::SubShapeIDCreator& inSubShapeIDCreator1,
	[[maybe_unused]] const JPH::SubShapeIDCreator& inSubShapeIDCreator2,
	[[maybe_unused]] const JPH::CollideShapeSettings& inCollideShapeSettings,
	[[maybe_unused]] JPH::CollideShapeCollector& ioCollector,
	[[maybe_unused]] const JPH::ShapeFilter& inShapeFilter
) {
	JPH_PROFILE_FUNCTION();

	// Get the shapes
	JPH_ASSERT(inShape1->GetSubType() == EShapeSubType::Sphere);
	JPH_ASSERT(inShape2->GetSubType() == EShapeSubType::VOXEL);

	const JPH::SphereShape* sphereShape = static_cast<const JPH::SphereShape*>(inShape1);
	const JoltCustomVoxelShape* voxelShape = static_cast<const JoltCustomVoxelShape*>(inShape2);

	// Get transforms
	JPH::Mat44 inverse_transform1 = inCenterOfMassTransform1.InversedRotationTranslation();
	JPH::Mat44 transform_2_to_1 = inverse_transform1 * inCenterOfMassTransform2;

	// Get bounding boxes
	JPH::AABox shape1_bbox = inShape1->GetLocalBounds().Scaled(inScale1);
	shape1_bbox.ExpandBy(JPH::Vec3::sReplicate(inCollideShapeSettings.mMaxSeparationDistance));
	JPH::AABox shape2_bbox = inShape2->GetLocalBounds().Scaled(inScale2);

	// Check if they overlap
	if (!JPH::OrientedBox(transform_2_to_1, shape2_bbox).Overlaps(shape1_bbox)) {
		return;
	}

	// Calculate the center of the sphere in the space of 2
	const auto mSphereCenterIn2 = inCenterOfMassTransform2.Multiply3x3Transposed(
		inCenterOfMassTransform1.GetTranslation() - inCenterOfMassTransform2.GetTranslation()
	);

	// Determine if shape 2 is inside out or not
	[[maybe_unused]] const auto mScaleSign2 = JPH::ScaleHelpers::IsInsideOut(inScale2)
		? -1.0f
		: 1.0f;
	JPH_ASSERT(mScaleSign2 > 0);

	// Check that the sphere is uniformly scaled
	JPH_ASSERT(ScaleHelpers::IsUniformScale(inScale1.Abs()));
	const auto mRadius = abs(inScale1.GetX()) * sphereShape->GetRadius();

	const auto sizeX = static_cast<int>(voxelShape->size.GetX());
	const auto sizeY = static_cast<int>(voxelShape->size.GetY());
	const auto sizeXY = sizeX * sizeY;

	for (auto i = 0; i < voxelShape->voxels.size(); i++) {
		const auto& data = voxelShape->voxels[i];
		if (data == 0) {
			continue;
		}

		const auto localX = i % sizeX;
		const auto localY = (i / sizeX) % sizeY;
		const auto localZ = i / sizeXY;
		const auto localPos = JPH::Vec3(
			static_cast<float>(localX),
			static_cast<float>(localY),
			static_cast<float>(localZ)
		);

		JPH::Vec3 point2 = inScale2 * localPos - mSphereCenterIn2;

		// Calculate penetration depth
		float penetration_depth = mRadius - point2.Length();
		if (-penetration_depth >= ioCollector.GetEarlyOutFraction()) {
			continue;
		}

		// Calculate penetration axis, direction along which to push 2 to move it out of collision
		// (this is always away from the sphere center)
		JPH::Vec3 penetration_axis = point2.NormalizedOr(JPH::Vec3::sAxisY());

		// Calculate the point on the sphere
		JPH::Vec3 point1 = mRadius * penetration_axis;

		// Convert to world space
		point1 = inCenterOfMassTransform2 * (mSphereCenterIn2 + point1);
		point2 = inCenterOfMassTransform2 * (mSphereCenterIn2 + point2);
		JPH::Vec3 penetration_axis_world = inCenterOfMassTransform2.Multiply3x3(penetration_axis);

		/*
		CollideShapeResult(
			Vec3Arg inContactPointOn1,
			Vec3Arg inContactPointOn2,
			Vec3Arg inPenetrationAxis,
			float inPenetrationDepth,
			const SubShapeID& inSubShapeID1,
			const SubShapeID& inSubShapeID2,
			const BodyID& inBodyID2
		);
		*/

		// Create collision result
		JPH::CollideShapeResult result(
			point1,
			point2,
			penetration_axis_world,
			penetration_depth,
			inSubShapeIDCreator1.GetID(),
			inSubShapeIDCreator2.GetID(),
			JPH::TransformedShape::sGetBodyID(ioCollector.GetContext())
		);

		// Notify the collector
		JPH_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
		ioCollector.AddHit(result);
	}

	/*
		const auto inverse = inCenterOfMassTransform1.Inversed();
		const auto localSpherePos = inverse * sphereShape->GetCenterOfMass();
		*/
}

void JoltCustomVoxelShape::register_type() {
	JPH::ShapeFunctions& shape_functions = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL);

	shape_functions.mConstruct = construct_empty;
	shape_functions.mColor = JPH::Color::sCyan;

	JPH::CollisionDispatch::sRegisterCollideShape(
		JPH::EShapeSubType::Sphere,
		JoltCustomShapeSubType::VOXEL,
		JoltCustomVoxelShape::CollideSphereVsVoxelBox
	);

	JPH::CollisionDispatch::sRegisterCollideShape(
		JoltCustomShapeSubType::VOXEL,
		JPH::EShapeSubType::Sphere,
		JPH::CollisionDispatch::sReversedCollideShape
	);

	/*
	for (const JPH::EShapeSubType sub_type : JPH::sAllSubShapeTypes) {
		JPH::CollisionDispatch::sRegisterCollideShape(
			JoltCustomShapeSubType::VOXEL,
			sub_type,
			collide_noop
		);

		JPH::CollisionDispatch::sRegisterCollideShape(
			sub_type,
			JoltCustomShapeSubType::VOXEL,
			collide_noop
		);

		JPH::CollisionDispatch::sRegisterCastShape(
			JoltCustomShapeSubType::VOXEL,
			sub_type,
			cast_noop
		);

		JPH::CollisionDispatch::sRegisterCastShape(
			sub_type,
			JoltCustomShapeSubType::VOXEL,
			cast_noop
		);
	}
	*/
}
