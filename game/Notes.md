# Implementation Notes

## Rigidbody
- Every object is a rigid body with linear motion, angular motion and mass.
- We also have some "static" body (mass equals zero). Those do not move.
- Every rigid body has a Shape

## Shape
- There are 3 types of shapes (Circle, Polygon and Box). Box is a specialization of polygon.
- We only support convex polygons for now.
- Other than the definition of the collision shape, they also contain the moment of inertia for applying torque to rigid bodies.

## Collision Detection
- During the collision detection, we store the collision information in Collision Contact for collision resolution.
- Contact info stores the start, end, normal vector and penetration depth.
- For polygon to polygon collision detection, we use SAT (Separating Axis Theorem).

### SAT Implementation
- We find the normal for each edge of polygon A
- For each normal axis, we loop over **all vertices** of polygon B
- We get the **separation** of each vertex of b by ***projecting*** them onto the normal axis
- We keep track of the **best** projection (separation) for each normal axis
- If the **separation** value is positive, we have **no** overlap
- If the **separation** value is negative, we have an overlap

## Constraints
- We have two type of constraints (Joint constraint and Penetration constraint).
- We solve our constraints in three steps: PreSolve, Solve, PostSolve
- **PreSolve**:
	- We caluclate the **Jacobian Matrix**.
	- We use **Warm Starting** which help us cache the previous frame's constraint forces and re-apply them this frame.
	- We then compute the **bias** (Baumgarte stabilisation) to help stabilise our simulation.
	- For the *Penetration constraint*, we calculate and add the bouciness to the **bias**. 
- **Solve**:
	- We use the **Gauss-Seidel** to solve our system of action to get the **lambda** (Lagrange multipliers).
	- We multiply this **lambda** by the Jacobian transpose to get the velocities vector to apply our linear and angular impulses.
	- For the *Penetration constraint*, we use the friction to calculate the **lambda**.
- **PostSolve**:
	- We limit the **Warm Starting** to reasonable limits.

## Collision Resolution
- We use the penetration constraint for collision resolution.
- We first integrate all the forces applied to the rigidbodies.
- We then create a penetration constraint for every collision based on the contact information.
- We solve all the constraints (Penetration and Joint) and integrate all the velocities.
