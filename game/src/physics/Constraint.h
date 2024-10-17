#pragma once

#include "MatMN.h"
#include "Vec2.h"
#include "VecN.h"

class RigidBody;

struct Constraint
{
	RigidBody* a;
	RigidBody* b;

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

protected:
	MatMN jacobian;
	VecN cachedLambda;
	float bias;
};

struct JointConstraint final : Constraint
{
public:
	JointConstraint(RigidBody* aRb, RigidBody* bRb, const Vec2& anchorPoint);
	void PreSolve(float dt) override;
	void Solve() override;
	void PostSolve() override;
};

struct PenetrationConstraint final : Constraint
{
private:
	Vec2 normal;
	float friction;

public:
	PenetrationConstraint(RigidBody* aRb, RigidBody* bRb, const Vec2& aCollisionPoint, const Vec2& bCollisionPoint, const Vec2& collisionNormal);
	void PreSolve(float dt) override;
	void Solve() override;
	void PostSolve() override;
};
