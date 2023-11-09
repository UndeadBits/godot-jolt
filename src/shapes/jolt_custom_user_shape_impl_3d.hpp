#pragma once

#include "jolt_shape_impl_3d.hpp"

struct VoxelShapeData {
	int sizeX = 0;
	int sizeY = 0;
	int sizeZ = 0;
	PackedByteArray voxels;
};

class JoltCustomUserShapeImpl3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CUSTOM; }

	bool is_convex() const override { return false; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

	String to_string() const;

private:
	JPH::ShapeRefC _build() const override;

	VoxelShapeData shapeData = VoxelShapeData();
};
