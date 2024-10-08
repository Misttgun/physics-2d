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
- 

### SAT Implementation
- We find the normal for each edge of polygon A
- For each normal axis, we loop over **all vertices** of polygon B
- We get the **separation** of each vertex of b by ***projecting*** them onto the normal axis
- We keep track of the **best** projection (separation) for each normal axis
- If the **separation** value is positive, we have **no** overlap
- If the **separation** value is negative, we have an overlap

## Collision Resolution
- We are using the impulse method for collision resolution.
- We apply linear and angular impulse at the point of contact.
- We calculate the impulse along the normal and tangent.
- We then combine those two impulses and apply them to the rigid bodies in opposite directions.

## Constraints
- 
