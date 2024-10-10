#include "Constraint.h"

#include <algorithm>

#include "RigidBody.h"

MatMN Constraint::GetInvM() const
{
	MatMN invM(6, 6);
	invM.Zero();

	invM.rows[0][0] = a->m_invMass;
	invM.rows[1][1] = a->m_invMass;
	invM.rows[2][2] = a->m_invInertia;
	invM.rows[3][3] = b->m_invMass;
	invM.rows[4][4] = b->m_invMass;
	invM.rows[5][5] = b->m_invInertia;

	return invM;
}

VecN Constraint::GetVelocities() const
{
	VecN v(6);
	v.Zero();

	v[0] = a->m_velocity.x;
	v[1] = a->m_velocity.y;
	v[2] = a->m_angularVelocity;
	v[3] = b->m_velocity.x;
	v[4] = b->m_velocity.y;
	v[5] = b->m_angularVelocity;

	return v;
}

JointConstraint::JointConstraint(const std::shared_ptr<RigidBody>& aRb, const std::shared_ptr<RigidBody>& bRb, const Vec2& anchorPoint)
{
	jacobian = MatMN(1, 6);
	cachedLambda = VecN(1);
	bias = 0.0f;

	a = aRb;
	b = bRb;
	aPoint = a->WorldToLocal(anchorPoint);
	bPoint = b->WorldToLocal(anchorPoint);

	cachedLambda.Zero();
}

void JointConstraint::PreSolve(const float dt)
{
	// Get the anchor point position in world space
	const Vec2 pa = a->LocalToWorld(aPoint);
	const Vec2 pb = b->LocalToWorld(bPoint);

	const Vec2 ra = pa - a->m_position;
	const Vec2 rb = pb - b->m_position;

	jacobian.Zero();

	const Vec2 j1 = (pa - pb) * 2.0f;
	jacobian.rows[0][0] = j1.x; // A linear velocity.x
	jacobian.rows[0][1] = j1.y; // A linear velocity.y

	const float j2 = ra.Cross(pa - pb) * 2.0f;
	jacobian.rows[0][2] = j2; // A angular velocity

	const Vec2 j3 = (pb - pa) * 2.0f;
	jacobian.rows[0][3] = j3.x; // B linear velocity.x
	jacobian.rows[0][4] = j3.y; // B linear velocity.y

	const float j4 = rb.Cross(pb - pa) * 2.0f;
	jacobian.rows[0][5] = j4; // B angular velocity

	// Warm starting (apply cached lambda)
	VecN impulses = jacobian.Transpose() * cachedLambda;

	// Apply the impulses to both A and B
	a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	a->ApplyImpulseAngular(impulses[2]);
	b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	b->ApplyImpulseAngular(impulses[5]);

	// Compute the positional error
	float c = (pb - pa).Dot(pb - pa);
	c = std::max(0.0f, c - 0.01f);

	// Compute the bias term (Baumgarte stabilization)
	constexpr float beta = 0.02f;
	bias = beta / dt * c;
}

void JointConstraint::Solve()
{
	const VecN v = GetVelocities();
	const MatMN invM = GetInvM();

	const MatMN j = jacobian;
	const MatMN jt = jacobian.Transpose();

	// Calculate the numerator
	const MatMN lhs = j * invM * jt; // A
	VecN rhs = j * v * -1.0f; // b
	rhs[0] -= bias;

	// Solve the values of lambda using Ax=b (Gauss-Seidel method)
	const VecN lambda = MatMN::SolveGaussSeidel(lhs, rhs);

	// Accumulate the lambda in the cached lambda
	cachedLambda += lambda;

	// Compute the final impulses with direction and magnitude
	VecN impulses = jt * lambda;

	// Apply the impulses to both A and B
	a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	a->ApplyImpulseAngular(impulses[2]);
	b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	b->ApplyImpulseAngular(impulses[5]);
}

void JointConstraint::PostSolve()
{
	// Limit the warm starting to reasonable limits
	cachedLambda[0] = std::clamp(cachedLambda[0], -10000.0f, 10000.0f);
}

PenetrationConstraint::PenetrationConstraint(const std::shared_ptr<RigidBody>& aRb, const std::shared_ptr<RigidBody>& bRb,
                                             const Vec2& aCollisionPoint, const Vec2& bCollisionPoint, const Vec2& collisionNormal)
{
	jacobian = MatMN(2, 6);
	cachedLambda = VecN(2);
	bias = 0.0f;

	a = aRb;
	b = bRb;
	aPoint = a->WorldToLocal(aCollisionPoint);
	bPoint = b->WorldToLocal(bCollisionPoint);
	normal = a->WorldToLocal(collisionNormal);

	friction = 0.0f;

	cachedLambda.Zero();
}

void PenetrationConstraint::PreSolve(const float dt)
{
	// Get the collision points in world space
	const Vec2 pa = a->LocalToWorld(aPoint);
	const Vec2 pb = b->LocalToWorld(bPoint);
	const Vec2 n = a->LocalToWorld(normal);

	const Vec2 ra = pa - a->m_position;
	const Vec2 rb = pb - b->m_position;

	jacobian.Zero();

	jacobian.rows[0][0] = -n.x;			// A linear velocity.x
	jacobian.rows[0][1] = -n.y;			// A linear velocity.y
	jacobian.rows[0][2] = -ra.Cross(n); // A angular velocity
	jacobian.rows[0][3] = n.x;			// B linear velocity.x
	jacobian.rows[0][4] = n.y;			// B linear velocity.y
	jacobian.rows[0][5] = rb.Cross(n);	// B angular velocity

	friction = std::max(a->m_friction, b->m_friction);
	if(friction > 0.0f)
	{
		const Vec2 t = n.Perpendicular(); // Tangent vector
		jacobian.rows[1][0] = -t.x;
		jacobian.rows[1][1] = -t.y;
		jacobian.rows[1][2] = -ra.Cross(t);
		jacobian.rows[1][3] = t.x;
		jacobian.rows[1][4] = t.y;
		jacobian.rows[1][5] = rb.Cross(t);
	}

	// Warm starting (apply cached lambda)
	VecN impulses = jacobian.Transpose() * cachedLambda;

	// Apply the impulses to both A and B
	a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	a->ApplyImpulseAngular(impulses[2]);
	b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	b->ApplyImpulseAngular(impulses[5]);

	// Compute the positional error
	float c = (pb - pa).Dot(-n);
	c = std::min(0.0f, c + 0.01f);

	//// Calculate relative velocity pre-impulse normal, which will be used to compute elasticity
	//const Vec2 va = a->m_velocity + Vec2(-a->m_angularVelocity * ra.y, a->m_angularVelocity * ra.x);
	//const Vec2 vb = b->m_velocity + Vec2(-b->m_angularVelocity * rb.y, b->m_angularVelocity * rb.x);
	//const float vRelDotNormal = (va - vb).Dot(n);

	//// Get the restitution between the two bodies
	//const float e = std::min(a->m_restitution, b->m_restitution);

	// Compute the bias term (Baumgarte stabilization)
	constexpr float beta = 0.2f;
	bias = beta / dt * c /*+ e * vRelDotNormal*/;
}

void PenetrationConstraint::Solve()
{
	const VecN v = GetVelocities();
	const MatMN invM = GetInvM();

	const MatMN j = jacobian;
	const MatMN jt = jacobian.Transpose();

	// Calculate the numerator
	const MatMN lhs = j * invM * jt; // A
	VecN rhs = j * v * -1.0f; // b
	rhs[0] -= bias;

	// Solve the values of lambda using Ax=b (Gauss-Seidel method)
	VecN lambda = MatMN::SolveGaussSeidel(lhs, rhs);

	// Accumulate the lambda and clamp it within constraint limits
	const VecN oldLambda = cachedLambda;
	cachedLambda += lambda;
	cachedLambda[0] = cachedLambda[0] < 0.0f ? 0.0f : cachedLambda[0];

	if(friction > 0.0f)
	{
		const float maxFriction = cachedLambda[0] * friction;
		cachedLambda[1] = std::clamp(cachedLambda[1], -maxFriction, maxFriction);
	}

	lambda = cachedLambda - oldLambda;

	// Compute the final impulses with direction and magnitude
	VecN impulses = jt * lambda;

	// Apply the impulses to both A and B
	a->ApplyImpulseLinear(Vec2(impulses[0], impulses[1]));
	a->ApplyImpulseAngular(impulses[2]);
	b->ApplyImpulseLinear(Vec2(impulses[3], impulses[4]));
	b->ApplyImpulseAngular(impulses[5]);
}

void PenetrationConstraint::PostSolve()
{
	Constraint::PostSolve();
}
