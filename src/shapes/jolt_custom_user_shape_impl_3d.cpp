#include "jolt_custom_user_shape_impl_3d.hpp"

#include "jolt_custom_voxel_shape.hpp"

Variant JoltCustomUserShapeImpl3D::get_data() const {
	Dictionary data;
	data["size"] = Vector3i(shapeData.sizeX, shapeData.sizeY, shapeData.sizeZ);
	return data;
}

void JoltCustomUserShapeImpl3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		_invalidated();
	};

	destroy();

	const auto type = p_data.get_type();
	// not configured
	if (type == Variant::NIL) {
		return;
	}

	ERR_FAIL_COND(type != Variant::DICTIONARY);

	const Dictionary data = p_data;

	{
		const Variant maybe_size = data.get("size", {});
		ERR_FAIL_COND(maybe_size.get_type() != Variant::VECTOR3I);

		const Vector3i size = (Vector3i)maybe_size;

		shapeData.sizeX = size.x;
		shapeData.sizeY = size.y;
		shapeData.sizeZ = size.z;
	}

	{
		const Variant maybe_data = data.get("voxels", {});
		ERR_FAIL_COND(maybe_data.get_type() != Variant::PACKED_BYTE_ARRAY);

		shapeData.voxels = (PackedByteArray)maybe_data;
	}
}

String JoltCustomUserShapeImpl3D::to_string() const {
	return vformat(
		"{size=(%f, %f, %f), voxels=(%d)}",
		shapeData.sizeX,
		shapeData.sizeY,
		shapeData.sizeZ,
		shapeData.voxels.size()
	);
}

JPH::ShapeRefC JoltCustomUserShapeImpl3D::_build() const {
	ERR_FAIL_COND_D_MSG(
		shapeData.sizeX <= 0 || shapeData.sizeY <= 0 || shapeData.sizeZ <= 0,
		vformat(
			"Godot Jolt failed to build custom user shape (voxel shape) with %s. "
			"Its size vector must be greater than (0, 0, 0). "
			"This shape belongs to %s.",
			to_string(),
			_owners_to_string()
		)
	);

	const auto numVoxels = shapeData.sizeX * shapeData.sizeY * shapeData.sizeZ;

	ERR_FAIL_COND_D_MSG(
		shapeData.voxels.size() != numVoxels,
		vformat(
			"Godot Jolt failed to build custom user shape (voxel shape) with %s. "
			"The size of the voxel array must be exactly %d (sizeX * sizeY * sizeZ)"
			"This shape belongs to %s.",
			to_string(),
			numVoxels,
			_owners_to_string()
		)
	);

	const JoltCustomVoxelShapeSettings
		shape_settings(shapeData.sizeX, shapeData.sizeY, shapeData.sizeZ, shapeData.voxels);
	const JPH::ShapeSettings::ShapeResult shape_result = shape_settings.Create();

	ERR_FAIL_COND_D_MSG(
		shape_result.HasError(),
		vformat(
			"Godot Jolt failed to build custom user shape (voxel shape) with %s. "
			"It returned the following error: '%s'. "
			"This shape belongs to %s.",
			to_string(),
			to_godot(shape_result.GetError()),
			_owners_to_string()
		)
	);

	WARN_PRINT(vformat("Custom user shape successfully created."));

	return shape_result.Get();
}
