/**
 * @file SoftBody.cpp
 * @brief Soft body physics simulation
 * @details Implements deformable objects like cloth, ropes, and jelly-like bodies
 *          using mass-spring systems and pressure models
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#include "SoftBody.h"

#include "../dynamics/RigidBody.h"

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

namespace engine::physics {
    SoftBody::~SoftBody() {
        if (bulletSoftBody_) {
            delete bulletSoftBody_;
            bulletSoftBody_ = nullptr;
        }
    }

    SoftBody* SoftBody::CreateCloth(btSoftBodyWorldInfo* worldInfo,
                                     const Vec3& corner00, const Vec3& corner10,
                                     const Vec3& corner01, const Vec3& corner11,
                                     const Int resolutionX, const Int resolutionY,
                                     const SoftBodyConfig& config) {
        const auto softBody = new SoftBody(worldInfo, config);

        // Create bullet soft body
        softBody->bulletSoftBody_ = btSoftBodyHelpers::CreatePatch(
            *worldInfo,
            btVector3(corner00.x, corner00.y, corner00.z),
            btVector3(corner10.x, corner10.y, corner10.z),
            btVector3(corner01.x, corner01.y, corner01.z),
            btVector3(corner11.x, corner11.y, corner11.z),
            resolutionX, resolutionY,
            config.fixedNodes[0] + config.fixedNodes[1] * 2 +
            config.fixedNodes[2] * 4 + config.fixedNodes[3] * 8,
            config.generateBendingConstraints
        );

        softBody->applyConfig();
        return softBody;
    }

    SoftBody* SoftBody::CreateRope(btSoftBodyWorldInfo* worldInfo,
                                    const Vec3& start, const Vec3& end,
                                    const Int resolution,
                                    const SoftBodyConfig& config) {
        const auto softBody = new SoftBody(worldInfo, config);

        Int fixeds = 0;
        if (config.fixedNodes[0]) fixeds |= 1; // Fix start
        if (config.fixedNodes[1]) fixeds |= 2; // Fix end

        softBody->bulletSoftBody_ = btSoftBodyHelpers::CreateRope(
            *worldInfo,
            btVector3(start.x, start.y, start.z),
            btVector3(end.x, end.y, end.z),
            resolution,
            fixeds
        );

        softBody->applyConfig();
        return softBody;
    }

    SoftBody* SoftBody::CreateEllipsoid(btSoftBodyWorldInfo* worldInfo,
                                         const Vec3& center, const Vec3& radius,
                                         const Int resolution,
                                         const SoftBodyConfig& config) {
        const auto softBody = new SoftBody(worldInfo, config);

        softBody->bulletSoftBody_ = btSoftBodyHelpers::CreateEllipsoid(
            *worldInfo,
            btVector3(center.x, center.y, center.z),
            btVector3(radius.x, radius.y, radius.z),
            resolution
        );

        softBody->applyConfig();
        return softBody;
    }

    SoftBody* SoftBody::CreateFromTriangleMesh(btSoftBodyWorldInfo* worldInfo,
                                                const std::vector<Vec3>& vertices,
                                                const std::vector<Int>& indices,
                                                const SoftBodyConfig& config) {
        const auto softBody = new SoftBody(worldInfo, config);

        // Convert to bullet format
        std::vector<btScalar> btVertices;
        btVertices.reserve(vertices.size() * 3);
        for (const auto& v : vertices) {
            btVertices.push_back(v.x);
            btVertices.push_back(v.y);
            btVertices.push_back(v.z);
        }

        softBody->bulletSoftBody_ = btSoftBodyHelpers::CreateFromTriMesh(
            *worldInfo,
            btVertices.data(),
            indices.data(),
            indices.size() / 3,
            config.randomizeConstraints
        );

        softBody->applyConfig();
        return softBody;
    }

    SoftBody* SoftBody::CreateFromTetrahedral(btSoftBodyWorldInfo* worldInfo,
                                               const std::vector<Vec3>& vertices,
                                               const std::vector<Int>& tetrahedra,
                                               const SoftBodyConfig& config) {
        const auto softBody = new SoftBody(worldInfo, config);

        // Create tetrahedral mesh
        // This would require a tetrahedral mesh generator
        // For now, using convex hull as approximation

        std::vector<btVector3> btVerts;
        for (const auto& v : vertices) {
            btVerts.push_back(btVector3(v.x, v.y, v.z));
        }

        softBody->bulletSoftBody_ = btSoftBodyHelpers::CreateFromConvexHull(
            *worldInfo,
            btVerts.data(),
            btVerts.size(),
            config.randomizeConstraints
        );

        softBody->applyConfig();
        return softBody;
    }

    void SoftBody::applyForceToNode(const Int nodeIndex, const Vec3& force) const {
        if (!bulletSoftBody_ || nodeIndex < 0 || nodeIndex >= getNodeCount()) return;

        bulletSoftBody_->addForce(btVector3(force.x, force.y, force.z), nodeIndex);
    }

    void SoftBody::applyForce(const Vec3& force) const {
        if (!bulletSoftBody_) return;

        bulletSoftBody_->addForce(btVector3(force.x, force.y, force.z));
    }

    void SoftBody::applyWindForce(const Vec3& windVelocity) const {
        if (!bulletSoftBody_) return;

        // Calculate wind force for each face
        for (int i = 0; i < bulletSoftBody_->m_faces.size(); ++i) {
            const btSoftBody::Face& face = bulletSoftBody_->m_faces[i];

            // Calculate face normal and area
            Vec3 v0(face.m_n[0]->m_x.x(), face.m_n[0]->m_x.y(), face.m_n[0]->m_x.z());
            Vec3 v1(face.m_n[1]->m_x.x(), face.m_n[1]->m_x.y(), face.m_n[1]->m_x.z());
            Vec3 v2(face.m_n[2]->m_x.x(), face.m_n[2]->m_x.y(), face.m_n[2]->m_x.z());

            Vec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
            Float area = glm::length(glm::cross(v1 - v0, v2 - v0)) * 0.5f;

            // Calculate relative wind velocity
            Vec3 faceVelocity = (Vec3(face.m_n[0]->m_v.x(), face.m_n[0]->m_v.y(), face.m_n[0]->m_v.z()) +
                                Vec3(face.m_n[1]->m_v.x(), face.m_n[1]->m_v.y(), face.m_n[1]->m_v.z()) +
                                Vec3(face.m_n[2]->m_v.x(), face.m_n[2]->m_v.y(), face.m_n[2]->m_v.z())) / 3.0f;

            Vec3 relativeWind = windVelocity - faceVelocity;

            // Calculate wind force (simplified aerodynamics)
            if (Float dot = glm::dot(relativeWind, normal); dot > 0) {
                Vec3 windForce = normal * dot * area * config_.material.drag +
                                glm::cross(normal, relativeWind) * area * config_.material.lift;

                // Apply to face nodes
                windForce /= 3.0f;
                btVector3 btForce(windForce.x, windForce.y, windForce.z);
                face.m_n[0]->m_f += btForce;
                face.m_n[1]->m_f += btForce;
                face.m_n[2]->m_f += btForce;
            }
        }
    }

    void SoftBody::setNodePosition(const Int nodeIndex, const Vec3& position) const {
        if (!bulletSoftBody_ || nodeIndex < 0 || nodeIndex >= getNodeCount()) return;

        bulletSoftBody_->m_nodes[nodeIndex].m_x = btVector3(position.x, position.y, position.z);
    }

    Vec3 SoftBody::getNodePosition(const Int nodeIndex) const {
        if (!bulletSoftBody_ || nodeIndex < 0 || nodeIndex >= getNodeCount()) {
            return Vec3(0);
        }

        const btVector3& pos = bulletSoftBody_->m_nodes[nodeIndex].m_x;
        return Vec3(pos.x(), pos.y(), pos.z());
    }

    void SoftBody::setNodeVelocity(const Int nodeIndex, const Vec3& velocity) const {
        if (!bulletSoftBody_ || nodeIndex < 0 || nodeIndex >= getNodeCount()) return;

        bulletSoftBody_->m_nodes[nodeIndex].m_v = btVector3(velocity.x, velocity.y, velocity.z);
    }

    Vec3 SoftBody::getNodeVelocity(const Int nodeIndex) const {
        if (!bulletSoftBody_ || nodeIndex < 0 || nodeIndex >= getNodeCount()) {
            return Vec3(0);
        }

        const btVector3& vel = bulletSoftBody_->m_nodes[nodeIndex].m_v;
        return Vec3(vel.x(), vel.y(), vel.z());
    }

    void SoftBody::setNodeFixed(const Int nodeIndex, const bool fixed) const {
        if (!bulletSoftBody_ || nodeIndex < 0 || nodeIndex >= getNodeCount()) return;

        if (fixed) {
            bulletSoftBody_->setMass(nodeIndex, 0);
        } else {
            bulletSoftBody_->setMass(nodeIndex, config_.totalMass / getNodeCount());
        }
    }

    void SoftBody::anchorToRigidBody(const Int nodeIndex, const RigidBody* rigidBody, const Vec3& localPosition) const {
        if (!bulletSoftBody_ || !rigidBody || !rigidBody->getBulletBody()) return;
        if (nodeIndex < 0 || nodeIndex >= getNodeCount()) return;

        bulletSoftBody_->appendAnchor(nodeIndex, rigidBody->getBulletBody(),
                                      btVector3(localPosition.x, localPosition.y, localPosition.z),
                                      false, config_.material.anchorHardness);
    }

    Int SoftBody::getNodeCount() const {
        return bulletSoftBody_ ? bulletSoftBody_->m_nodes.size() : 0;
    }

    Int SoftBody::getFaceCount() const {
        return bulletSoftBody_ ? bulletSoftBody_->m_faces.size() : 0;
    }

    Int SoftBody::getLinkCount() const {
        return bulletSoftBody_ ? bulletSoftBody_->m_links.size() : 0;
    }

    void SoftBody::removeAllAnchors() const {
        if (!bulletSoftBody_) return;
        bulletSoftBody_->m_anchors.clear();
    }

    std::vector<Vec3> SoftBody::getNodePositions() const {
        std::vector<Vec3> positions;
        if (!bulletSoftBody_) return positions;

        positions.reserve(bulletSoftBody_->m_nodes.size());

        // Fix: Use traditional indexing loop instead of range-based loop
        for (int i = 0; i < bulletSoftBody_->m_nodes.size(); ++i) {
            const btSoftBody::Node& node = bulletSoftBody_->m_nodes[i];
            positions.push_back(Vec3(node.m_x.x(), node.m_x.y(), node.m_x.z()));
        }

        return positions;
    }

    std::vector<Vec3> SoftBody::getNodeNormals() const {
        std::vector<Vec3> normals;
        if (!bulletSoftBody_) return normals;

        normals.reserve(bulletSoftBody_->m_nodes.size());

        // Fix: Use traditional indexing loop instead of range-based loop
        for (int i = 0; i < bulletSoftBody_->m_nodes.size(); ++i) {
            const btSoftBody::Node& node = bulletSoftBody_->m_nodes[i];
            normals.push_back(Vec3(node.m_n.x(), node.m_n.y(), node.m_n.z()));
        }

        return normals;
    }

    std::vector<Int> SoftBody::getFaceIndices() const {
        std::vector<Int> indices;
        if (!bulletSoftBody_) return indices;

        indices.reserve(bulletSoftBody_->m_faces.size() * 3);

        // Fix: Use traditional indexing loop instead of range-based loop
        for (int faceIdx = 0; faceIdx < bulletSoftBody_->m_faces.size(); ++faceIdx) {
            const btSoftBody::Face& face = bulletSoftBody_->m_faces[faceIdx];

            for (int i = 0; i < 3; ++i) {
                // Find node index
                for (int j = 0; j < bulletSoftBody_->m_nodes.size(); ++j) {
                    if (&bulletSoftBody_->m_nodes[j] == face.m_n[i]) {
                        indices.push_back(j);
                        break;
                    }
                }
            }
        }

        return indices;
    }

    AABB SoftBody::getBoundingBox() const {
        AABB aabb;
        if (!bulletSoftBody_) return aabb;

        btVector3 aabbMin, aabbMax;
        bulletSoftBody_->getAabb(aabbMin, aabbMax);

        aabb.min = Vec3(aabbMin.x(), aabbMin.y(), aabbMin.z());
        aabb.max = Vec3(aabbMax.x(), aabbMax.y(), aabbMax.z());

        return aabb;
    }

    Float SoftBody::getTotalMass() const {
        return bulletSoftBody_ ? bulletSoftBody_->getTotalMass() : 0.0f;
    }

    Float SoftBody::getVolume() const {
        return bulletSoftBody_ ? bulletSoftBody_->getVolume() : 0.0f;
    }

    Vec3 SoftBody::getCenterOfMass() const {
        if (!bulletSoftBody_) return Vec3(0);

        const btVector3 com = bulletSoftBody_->getCenterOfMass();
        return Vec3(com.x(), com.y(), com.z());
    }

    void SoftBody::setMaterial(const SoftBodyMaterial& material) {
        config_.material = material;
        applyMaterial();
    }

    void SoftBody::setTotalMass(const Float mass) {
        if (!bulletSoftBody_) return;

        config_.totalMass = mass;
        bulletSoftBody_->setTotalMass(mass);
    }

    void SoftBody::applyConfig() const {
        if (!bulletSoftBody_) return;

        // Apply material
        applyMaterial();

        // Set solver iterations
        bulletSoftBody_->m_cfg.viterations = config_.velocityIterations;
        bulletSoftBody_->m_cfg.piterations = config_.positionIterations;
        bulletSoftBody_->m_cfg.diterations = config_.driftIterations;
        bulletSoftBody_->m_cfg.citerations = config_.clusterIterations;

        // Set collision flags
        if (config_.selfCollision) {
            bulletSoftBody_->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
        }

        bulletSoftBody_->getCollisionShape()->setMargin(config_.collisionMargin);

        // Generate bending constraints
        if (config_.generateBendingConstraints) {
            bulletSoftBody_->generateBendingConstraints(2);
        }

        // Generate clusters for deformation
        if (config_.generateClusters) {
            bulletSoftBody_->generateClusters(config_.clusterCount);
        }

        // Set total mass
        bulletSoftBody_->setTotalMass(config_.totalMass);

        // Set wind velocity
        if (glm::length2(config_.windVelocity) > 0) {
            bulletSoftBody_->setWindVelocity(btVector3(
                config_.windVelocity.x,
                config_.windVelocity.y,
                config_.windVelocity.z
            ));
        }
    }

    void SoftBody::applyMaterial() const {
        if (!bulletSoftBody_) return;

        btSoftBody::Material* mat = bulletSoftBody_->m_materials[0];

        mat->m_kLST = config_.material.linearStiffness;
        mat->m_kAST = config_.material.angularStiffness;
        mat->m_kVST = config_.material.volumeStiffness;

        bulletSoftBody_->m_cfg.kDF = config_.material.dynamicFriction;
        bulletSoftBody_->m_cfg.kDP = config_.material.damping;
        bulletSoftBody_->m_cfg.kDG = config_.material.drag;
        bulletSoftBody_->m_cfg.kLF = config_.material.lift;
        bulletSoftBody_->m_cfg.kPR = config_.material.pressure;
        bulletSoftBody_->m_cfg.kVC = config_.material.volumeStiffness;
        bulletSoftBody_->m_cfg.kMT = config_.material.poseMatching;
        bulletSoftBody_->m_cfg.kCHR = config_.material.rigidContactHardness;
        bulletSoftBody_->m_cfg.kKHR = config_.material.kineticContactHardness;
        bulletSoftBody_->m_cfg.kSHR = config_.material.softContactHardness;
        bulletSoftBody_->m_cfg.kAHR = config_.material.anchorHardness;
    }

    ClothSimulation::ClothSimulation(btSoftBodyWorldInfo* worldInfo, const ClothParams& params)
            : SoftBody(worldInfo) {  // Initialize base class first

        SoftBodyConfig config;
        config.type = SoftBodyType::CLOTH;
        config.material = params.material;
        config.totalMass = params.mass;
        config.fixedNodes[0] = params.fixTopCorners;
        config.fixedNodes[1] = params.fixTopCorners;
        config.generateBendingConstraints = true;

        // Fix: Now we can access protected member
        config_ = config;

        // Create cloth
        // Fix: Now we can access protected member
        bulletSoftBody_ = btSoftBodyHelpers::CreatePatch(
            *worldInfo,
            btVector3(params.topLeft.x, params.topLeft.y, params.topLeft.z),
            btVector3(params.topRight.x, params.topRight.y, params.topRight.z),
            btVector3(params.bottomLeft.x, params.bottomLeft.y, params.bottomLeft.z),
            btVector3(params.bottomRight.x, params.bottomRight.y, params.bottomRight.z),
            params.resolutionX, params.resolutionY,
            params.fixTopCorners ? 3 : 0,  // Fix top corners
            true  // Generate diagonals
        );

        // Fix: Now we can access protected method
        applyConfig();

        // Fix all top nodes if requested
        if (params.fixAllTop) {
            for (int i = 0; i < params.resolutionX; ++i) {
                setNodeFixed(i, true);
            }
        }
    }

    void ClothSimulation::applyTearing(const Float tearThreshold) const {
        // Fix: Now we can access protected member
        if (!bulletSoftBody_) return;

        std::vector<int> linksToRemove;

        // Fix: Use traditional indexing loop for btAlignedObjectArray
        for (int i = 0; i < bulletSoftBody_->m_links.size(); ++i) {
            const btSoftBody::Link& link = bulletSoftBody_->m_links[i];

            Vec3 pos0(link.m_n[0]->m_x.x(), link.m_n[0]->m_x.y(), link.m_n[0]->m_x.z());
            Vec3 pos1(link.m_n[1]->m_x.x(), link.m_n[1]->m_x.y(), link.m_n[1]->m_x.z());

            const Float currentLength = glm::length(pos1 - pos0);
            const Float restLength = link.m_rl;

            if (const Float strain = (currentLength - restLength) / restLength; strain > tearThreshold) {
                linksToRemove.push_back(i);
            }
        }

        // Remove overstressed links (tear the cloth)
        // Important: Remove from back to front to maintain indices
        for (int i = static_cast<int>(linksToRemove.size()) - 1; i >= 0; --i) {
            bulletSoftBody_->m_links.removeAtIndex(linksToRemove[i]);
        }
    }

    RopeSimulation::RopeSimulation(btSoftBodyWorldInfo* worldInfo, const RopeParams& params)
            : SoftBody(worldInfo) {  // Initialize base class first

        SoftBodyConfig config;
        config.type = SoftBodyType::ROPE;
        config.material = params.material;
        config.totalMass = params.mass;
        config.fixedNodes[0] = params.fixStart;
        config.fixedNodes[1] = params.fixEnd;

        // Fix: Now we can access protected member
        config_ = config;

        Int fixeds = 0;
        if (params.fixStart) fixeds |= 1;
        if (params.fixEnd) fixeds |= 2;

        // Fix: Now we can access protected member
        bulletSoftBody_ = btSoftBodyHelpers::CreateRope(
            *worldInfo,
            btVector3(params.startPoint.x, params.startPoint.y, params.startPoint.z),
            btVector3(params.endPoint.x, params.endPoint.y, params.endPoint.z),
            params.segments,
            fixeds
        );

        // Fix: Now we can access protected method
        applyConfig();

        thickness_ = params.thickness;
    }
} // namespace engine::physics
