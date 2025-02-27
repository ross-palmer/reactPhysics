#include <iostream>
#include <vector>

#include <reactphysics3d/reactphysics3d.h>

#pragma comment(lib, "reactphysics3d.lib")

using namespace reactphysics3d;

int main() {
    PhysicsCommon physicsCommon;
    PhysicsWorld* world = physicsCommon.createPhysicsWorld();

    // Create two rigid bodies with bounding box collision shapes
    // Body 1
    Vector3 position1(0, 5, 0);
    Quaternion orientation1 = Quaternion::identity();
    Transform transform1(position1, orientation1);
    RigidBody* body1 = world->createRigidBody(transform1);

    // Create a box collision shape for body 1
    Vector3 boxHalfExtents1(2, 2, 2); // Half-extents of the box
    BoxShape* boxShape1 = physicsCommon.createBoxShape(boxHalfExtents1);
    body1->addCollider(boxShape1, Transform::identity());

    // Body 2
    Vector3 position2(0, 0, 0);
    Quaternion orientation2 = Quaternion::identity();
    Transform transform2(position2, orientation2);
    RigidBody* body2 = world->createRigidBody(transform2);

    // Create a box collision shape for body 2
    Vector3 boxHalfExtents2(3, 1, 3); // Half-extents of the box
    BoxShape* boxShape2 = physicsCommon.createBoxShape(boxHalfExtents2);
    body2->addCollider(boxShape2, Transform::identity());

    const decimal timeStep = 1.0f / 60.0f;

    // Simulate and check for collisions
    for (int i = 0; i < 100; ++i) {
        world->update(timeStep);

        // Check for overlap between the two bodies
        if (world->testOverlap(body1, body2)) {
            std::cout << "Collision detected!" << std::endl;
        }

        //Simple output of body positions.
        const Transform& transform1Updated = body1->getTransform();
        const Vector3& position1Updated = transform1Updated.getPosition();

        const Transform& transform2Updated = body2->getTransform();
        const Vector3& position2Updated = transform2Updated.getPosition();

        std::cout << "Body 1 Position: (" << position1Updated.x << ", " << position1Updated.y << ", " << position1Updated.z << ")" << std::endl;
        std::cout << "Body 2 Position: (" << position2Updated.x << ", " << position2Updated.y << ", " << position2Updated.z << ")" << std::endl;
        std::cout << "----------------------" << std::endl;

    }

    // Cleanup
    //physicsCommon.destroyRigidBody(body1);
    //physicsCommon.destroyRigidBody(body2);
    //physicsCommon.destroyBoxShape(boxShape1);
    //physicsCommon.destroyBoxShape(boxShape2);
    //physicsCommon.destroyPhysicsWorld(world);

    return 0;
}