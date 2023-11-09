#pragma once

#include "jolt_custom_shape_type.hpp"

class JoltCustomVoxelShapeSettings final : public JPH::ShapeSettings {
public:
	JoltCustomVoxelShapeSettings() = default;

	JoltCustomVoxelShapeSettings(
		int sizeX,
		int sizeY,
		int sizeZ,
		PackedByteArray voxels,
		const JPH::PhysicsMaterial* p_material = nullptr
	)
		: material(p_material)
		, sizeX(sizeX)
		, sizeY(sizeY)
		, sizeZ(sizeZ)
		, voxels(voxels) { }

	JPH::ShapeSettings::ShapeResult Create() const override;

	JPH::RefConst<JPH::PhysicsMaterial> material;

	int sizeX = 0;
	int sizeY = 0;
	int sizeZ = 0;
	PackedByteArray voxels;
};

class JoltCustomVoxelShape final : public JPH::Shape {
private:
	JPH::Vec3 size;
	PackedByteArray voxels;

public:
	static void register_type();

	JoltCustomVoxelShape()
		: Shape(JoltCustomShapeType::CUSTOM, JoltCustomShapeSubType::VOXEL) { }

	JoltCustomVoxelShape(const JoltCustomVoxelShapeSettings& p_settings, ShapeResult& p_result)
		: Shape(JoltCustomShapeType::CUSTOM, JoltCustomShapeSubType::VOXEL, p_settings, p_result)
		, size(
			  static_cast<float>(p_settings.sizeX),
			  static_cast<float>(p_settings.sizeY),
			  static_cast<float>(p_settings.sizeZ)
		  )
		, voxels(p_settings.voxels) {
		if (!p_result.HasError()) {
			p_result.Set(this);
		}
	}

	/// Get local bounding box including convex radius, this box is centered around the center of
	/// mass rather than the world transform
	JPH::AABox GetLocalBounds() const override;

	/// Get the max number of sub shape ID bits that are needed to be able to address any leaf shape
	/// in this shape. Used mainly for checking that it is smaller or equal than
	/// SubShapeID::MaxBits.
	JPH::uint GetSubShapeIDBitsRecursive() const override { return 0; }

	/// Returns the radius of the biggest sphere that fits entirely in the shape. In case this shape
	/// consists of multiple sub shapes, it returns the smallest sphere of the parts. This can be
	/// used as a measure of how far the shape can be moved without risking going through geometry.
	float GetInnerRadius() const override;

	/// Calculate the mass and inertia of this shape
	JPH::MassProperties GetMassProperties() const override;

	/// Get the material assigned to a particular sub shape ID
	const JPH::PhysicsMaterial* GetMaterial([[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id
	) const override {
		return JPH::PhysicsMaterial::sDefault;
	}

	/// Get the surface normal of a particular sub shape ID and point on surface (all vectors are
	/// relative to center of mass for this shape). Note: When you have a CollideShapeResult or
	/// ShapeCastResult you should use -mPenetrationAxis.Normalized() as contact normal as
	/// GetSurfaceNormal will only return face normals (and not vertex or edge normals).
	JPH::Vec3 GetSurfaceNormal(
		[[maybe_unused]] const JPH::SubShapeID& p_sub_shape_id,
		[[maybe_unused]] JPH::Vec3Arg p_local_surface_position
	) const override {
		return JPH::Vec3::sZero();
	}

	// clang-format off

	void GetSubmergedVolume(
		[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform,
		[[maybe_unused]] JPH::Vec3Arg p_scale,
		[[maybe_unused]] const JPH::Plane& p_surface,
		float& p_total_volume,
		float& p_submerged_volume,
		JPH::Vec3& p_center_of_buoyancy
#ifdef JPH_DEBUG_RENDERER
		, [[maybe_unused]] JPH::RVec3Arg p_base_offset
#endif // JPH_DEBUG_RENDERER
	) const override {
		p_total_volume = 0.0f;
		p_submerged_volume = 0.0f;
		p_center_of_buoyancy = JPH::Vec3::sZero();
	}

	// clang-format on

#ifdef JPH_DEBUG_RENDERER
	void Draw(
		[[maybe_unused]] JPH::DebugRenderer* p_renderer,
		[[maybe_unused]] JPH::RMat44Arg p_center_of_mass_transform,
		[[maybe_unused]] JPH::Vec3Arg p_scale,
		[[maybe_unused]] JPH::ColorArg p_color,
		[[maybe_unused]] bool p_use_material_colors,
		[[maybe_unused]] bool p_draw_wireframe
	) const override { }
#endif // JPH_DEBUG_RENDERER
	/// Cast a ray against this shape, returns true if it finds a hit closer than ioHit.mFraction
	/// and updates that fraction. Otherwise ioHit is left untouched and the function returns false.
	/// Note that the ray should be relative to the center of mass of this shape (i.e. subtract
	/// Shape::GetCenterOfMass() from RayCast::mOrigin if you want to cast against the shape in the
	/// space it was created). Convex objects will be treated as solid (meaning if the ray starts
	/// inside, you'll get a hit fraction of 0) and back face hits against triangles are returned.
	/// If you want the surface normal of the hit use GetSurfaceNormal(ioHit.mSubShapeID2,
	/// inRay.GetPointOnRay(ioHit.mFraction)).
	bool CastRay(
		[[maybe_unused]] const JPH::RayCast& p_ray,
		[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator,
		[[maybe_unused]] JPH::RayCastResult& p_hit
	) const override {
		return false;
	}

	/// Cast a ray against this shape. Allows returning multiple hits through ioCollector. Note that
	/// this version is more flexible but also slightly slower than the CastRay function that
	/// returns only a single hit. If you want the surface normal of the hit use
	/// GetSurfaceNormal(collected sub shape ID, inRay.GetPointOnRay(collected faction)).
	void CastRay(
		[[maybe_unused]] const JPH::RayCast& p_ray,
		[[maybe_unused]] const JPH::RayCastSettings& p_ray_cast_settings,
		[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator,
		[[maybe_unused]] JPH::CastRayCollector& p_collector,
		[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter = {}
	) const override { }

	/// Check if inPoint is inside this shape. For this tests all shapes are treated as if they were
	/// solid. Note that inPoint should be relative to the center of mass of this shape (i.e.
	/// subtract Shape::GetCenterOfMass() from inPoint if you want to test against the shape in the
	/// space it was created). For a mesh shape, this test will only provide sensible information if
	/// the mesh is a closed manifold. For each shape that collides, ioCollector will receive a hit.
	void CollidePoint(
		[[maybe_unused]] JPH::Vec3Arg p_point,
		[[maybe_unused]] const JPH::SubShapeIDCreator& p_sub_shape_id_creator,
		[[maybe_unused]] JPH::CollidePointCollector& p_collector,
		[[maybe_unused]] const JPH::ShapeFilter& p_shape_filter = {}
	) const override { }

	/// Collides all vertices of a soft body with this shape and updates
	/// SoftBodyVertex::mCollisionPlane, SoftBodyVertex::mCollidingShapeIndex and
	/// SoftBodyVertex::mLargestPenetration if a collision with more penetration was found.
	/// @param inCenterOfMassTransform Center of mass transform for this shape relative to the
	/// vertices.
	/// @param inScale The scale to use for this shape
	/// @param ioVertices The vertices of the soft body
	/// @param inNumVertices The number of vertices in ioVertices
	/// @param inDeltaTime Delta time of this time step (can be used to extrapolate the position
	/// using the velocity of the particle)
	/// @param inDisplacementDueToGravity Displacement due to gravity during this time step
	/// @param inCollidingShapeIndex Value to store in SoftBodyVertex::mCollidingShapeIndex when a
	/// collision was found
	void CollideSoftBodyVertices(
		[[maybe_unused]] JPH::Mat44Arg p_center_of_mass_transform,
		[[maybe_unused]] JPH::Vec3Arg p_scale,
		[[maybe_unused]] JPH::SoftBodyVertex* p_vertices,
		[[maybe_unused]] JPH::uint p_num_vertices,
		[[maybe_unused]] float p_delta_time,
		[[maybe_unused]] JPH::Vec3Arg p_displacement_due_to_gravity,
		[[maybe_unused]] int p_colliding_shape_index
	) const override { }

	void GetTrianglesStart(
		[[maybe_unused]] GetTrianglesContext& p_context,
		[[maybe_unused]] const JPH::AABox& p_box,
		[[maybe_unused]] JPH::Vec3Arg p_position_com,
		[[maybe_unused]] JPH::QuatArg p_rotation,
		[[maybe_unused]] JPH::Vec3Arg p_scale
	) const override { }

	int GetTrianglesNext(
		[[maybe_unused]] GetTrianglesContext& p_context,
		[[maybe_unused]] int p_max_triangles_requested,
		[[maybe_unused]] JPH::Float3* p_triangle_vertices,
		[[maybe_unused]] const JPH::PhysicsMaterial** p_materials = nullptr
	) const override {
		return 0;
	}

	/// Saves the contents of the shape in binary form to inStream.
	void SaveBinaryState(JPH::StreamOut& inStream) const override;

	/// Get the combined stats of this shape and its children.
	/// @param ioVisitedShapes is used to track which shapes have already been visited, to avoid
	/// calculating the wrong memory size.
	Stats GetStats() const override { return {sizeof(*this), 0}; }

	///< Volume of this shape (m^3). Note that for compound shapes the volume may be incorrect since
	///< child shapes can overlap which is not accounted for.
	float GetVolume() const override {
		return this->size.GetX() * this->size.GetY() * this->size.GetZ();
	}

protected:
	/// This function should not be called directly, it is used by sRestoreFromBinaryState.
	void RestoreBinaryState(JPH::StreamIn& inStream) override;

private:
	// Helper functions called by CollisionDispatch

	static void CollideSphereVsVoxelBox(
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
	);
};
