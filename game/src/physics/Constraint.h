#pragma once

#include <memory>

#include "MatMN.h"
#include "Vec2.h"
#include "VecN.h"

class RigidBody;

struct Constraint
{
	std::shared_ptr<RigidBody> a;
	std::shared_ptr<RigidBody> b;

	Vec2 aPoint; // The anchor point in A's local space
	Vec2 bPoint; // The anchor point in B's local space

	Constraint() = default;
	Constraint(const Constraint& shape) = default;
	Constraint& operator =(const Constraint& shape) = default;
	Constraint(Constraint&& shape) = default;
	Constraint& operator =(Constraint&& shape) = default;
	virtual ~Constraint() = default;

	[[nodiscard]] MatMN GetInvM() const;
	[[nodiscard]] VecN GetVelocities() const;

	virtual void PreSolve(float dt) {}
	virtual void Solve() {}
	virtual void PostSolve() {}
};

struct JointConstraint final : Constraint
{
private:
	MatMN jacobian;
	VecN cachedLambda;
	float bias;

public:
	JointConstraint(const std::shared_ptr<RigidBody>& aRb, const std::shared_ptr<RigidBody>& bRb, const Vec2& anchorPoint);
	void PreSolve(float dt) override;
	void Solve() override;
	void PostSolve() override;
};
