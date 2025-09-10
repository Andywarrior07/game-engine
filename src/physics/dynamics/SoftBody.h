/**
 * @file SoftBody.h
 * @brief Soft body physics simulation
 * @details Implements deformable objects like cloth, ropes, and jelly-like bodies
 *          using mass-spring systems and pressure models
 * @author Andrés Guerrero
 * @date 31-08-2025
 */

#pragma once

#include "../core/PhysicsTypes.h"
// #include "../dynamics/RigidBody.h"

#include "../../math/MathSystem.h"

// #include <BulletSoftBody/btSoftBody.h>

// Forward declaration
struct btSoftBodyWorldInfo;
class btSoftBody;

namespace engine::physics {

    using namespace engine::math;

    /**
     * @brief Soft body type
     */
    enum class SoftBodyType : std::uint8_t {
        CLOTH,      // 2D sheet (curtains, flags, clothing)
        ROPE,       // 1D line (cables, chains)
        VOLUME,     // 3D solid (jelly, rubber objects)
        PATCH,      // Surface patch
        TETRAHEDRAL // Tetrahedral mesh for accurate deformation
    };

    /**
     * @brief Soft body material properties
     */
    struct SoftBodyMaterial {
        Float linearStiffness = 0.5f;    // Linear stiffness coefficient [0,1]
        Float angularStiffness = 0.5f;   // Angular stiffness coefficient [0,1]
        Float volumeStiffness = 0.5f;    // Volume preservation stiffness [0,1]
        Float damping = 0.01f;           // Velocity damping
        Float drag = 0.0f;               // Drag coefficient
        Float lift = 0.0f;               // Lift coefficient
        Float pressure = 0.0f;           // Internal pressure
        Float dynamicFriction = 0.2f;   // Dynamic friction
        Float poseMatching = 0.0f;      // Pose matching coefficient [0,1]
        Float rigidContactHardness = 1.0f; // Contact hardness with rigid bodies
        Float kineticContactHardness = 0.1f; // Contact hardness with kinematic bodies
        Float softContactHardness = 0.1f; // Contact hardness with other soft bodies
        Float anchorHardness = 0.7f;    // Anchor hardness

        // Presets
        static SoftBodyMaterial Cloth() {
            SoftBodyMaterial mat;
            mat.linearStiffness = 0.8f;
            mat.angularStiffness = 0.5f;
            mat.damping = 0.01f;
            mat.drag = 0.001f;
            mat.lift = 0.001f;
            return mat;
        }

        static SoftBodyMaterial Rubber() {
            SoftBodyMaterial mat;
            mat.linearStiffness = 0.1f;
            mat.angularStiffness = 0.1f;
            mat.volumeStiffness = 0.9f;
            mat.damping = 0.05f;
            mat.pressure = 100.0f;
            return mat;
        }

        static SoftBodyMaterial Jelly() {
            SoftBodyMaterial mat;
            mat.linearStiffness = 0.05f;
            mat.angularStiffness = 0.05f;
            mat.volumeStiffness = 0.8f;
            mat.damping = 0.1f;
            mat.pressure = 50.0f;
            mat.poseMatching = 0.2f;
            return mat;
        }

        static SoftBodyMaterial Rope() {
            SoftBodyMaterial mat;
            mat.linearStiffness = 0.9f;
            mat.angularStiffness = 0.1f;
            mat.damping = 0.02f;
            mat.drag = 0.01f;
            return mat;
        }
    };

    /**
     * @brief Soft body configuration
     */
    struct SoftBodyConfig {
        SoftBodyType type = SoftBodyType::VOLUME;
        SoftBodyMaterial material;

        // Simulation settings
        Int velocityIterations = 1;     // Velocity solver iterations
        Int positionIterations = 2;     // Position solver iterations
        Int driftIterations = 0;        // Drift solver iterations
        Int clusterIterations = 4;      // Cluster solver iterations

        // Collision settings
        bool selfCollision = false;     // Enable self-collision
        Float collisionMargin = 0.02f;  // Collision margin
        Float contactThreshold = 0.02f; // Contact threshold distance

        // Features
        bool generateBendingConstraints = true;
        bool randomizeConstraints = false;
        bool generateClusters = false;
        Int clusterCount = 8;

        // Mass distribution
        Float totalMass = 1.0f;
        bool fixedNodes[4] = {false, false, false, false}; // For cloth: corners

        // Wind settings
        Vec3 windVelocity = Vec3(0);
        Float windStrength = 0.0f;
    };

    /**
     * @brief Soft body node/vertex
     */
    struct SoftBodyNode {
        Vec3 position;
        Vec3 velocity;
        Vec3 force;
        Float mass;
        Float invMass;
        bool fixed;

        SoftBodyNode() : position(0), velocity(0), force(0),
                        mass(1.0f), invMass(1.0f), fixed(false) {}
    };

    /**
     * @brief Soft body constraint/spring
     */
    struct SoftBodySpring {
        Int nodeA;
        Int nodeB;
        Float restLength;
        Float stiffness;
        Float damping;

        SoftBodySpring(const Int a, const Int b, const Float length, const Float stiff = 1.0f, const Float damp = 0.1f)
            : nodeA(a), nodeB(b), restLength(length), stiffness(stiff), damping(damp) {}
    };

    /**
     * @brief Soft body physics object
     */
    class SoftBody {
    public:
        explicit SoftBody(btSoftBodyWorldInfo* worldInfo, const SoftBodyConfig& config = SoftBodyConfig())
            : worldInfo_(worldInfo), bulletSoftBody_(nullptr), config_(config) {}


        ~SoftBody();

        /**
         * @brief Create cloth soft body
         */
        static SoftBody* CreateCloth(btSoftBodyWorldInfo* worldInfo,
                                     const Vec3& corner00, const Vec3& corner10,
                                     const Vec3& corner01, const Vec3& corner11,
                                     const Int resolutionX, const Int resolutionY,
                                     const SoftBodyConfig& config = SoftBodyConfig());

        /**
         * @brief Create rope soft body
         */
        static SoftBody* CreateRope(btSoftBodyWorldInfo* worldInfo,
                                    const Vec3& start, const Vec3& end,
                                    const Int resolution,
                                    const SoftBodyConfig& config = SoftBodyConfig());

        /**
         * @brief Create ellipsoid soft body
         */
        static SoftBody* CreateEllipsoid(btSoftBodyWorldInfo* worldInfo,
                                         const Vec3& center, const Vec3& radius,
                                         const Int resolution,
                                         const SoftBodyConfig& config = SoftBodyConfig());

        /**
         * @brief Create soft body from triangle mesh
         */
        static SoftBody* CreateFromTriangleMesh(btSoftBodyWorldInfo* worldInfo,
                                                const std::vector<Vec3>& vertices,
                                                const std::vector<Int>& indices,
                                                const SoftBodyConfig& config = SoftBodyConfig());

        /**
         * @brief Create tetrahedral soft body from mesh
         */
        static SoftBody* CreateFromTetrahedral(btSoftBodyWorldInfo* worldInfo,
                                               const std::vector<Vec3>& vertices,
                                               const std::vector<Int>& tetrahedra,
                                               const SoftBodyConfig& config = SoftBodyConfig());

        // ============================================================================
        // Control Methods
        // ============================================================================

        /**
         * @brief Apply force to node
         */
        void applyForceToNode(Int nodeIndex, const Vec3& force) const;

        /**
         * @brief Apply force to all nodes
         */
        void applyForce(const Vec3& force) const;

        /**
         * @brief Apply wind force
         */
        void applyWindForce(const Vec3& windVelocity) const;

        /**
         * @brief Set node position
         */
        void setNodePosition(const Int nodeIndex, const Vec3& position) const;

        /**
         * @brief Get node position
         */
        Vec3 getNodePosition(const Int nodeIndex) const;

        /**
         * @brief Set node velocity
         */
        void setNodeVelocity(const Int nodeIndex, const Vec3& velocity) const;

        /**
         * @brief Get node velocity
         */
        Vec3 getNodeVelocity(const Int nodeIndex) const;

        /**
         * @brief Fix node in space
         */
        void setNodeFixed(const Int nodeIndex, const bool fixed) const;

        /**
         * @brief Anchor node to rigid body
         */
        void anchorToRigidBody(const Int nodeIndex, const RigidBody* rigidBody, const Vec3& localPosition) const;

        /**
         * @brief Remove all anchors
         */
        void removeAllAnchors() const;

        // ============================================================================
        // Query Methods
        // ============================================================================

        Int getNodeCount() const;

        Int getFaceCount() const;

        Int getLinkCount() const;

        /**
         * @brief Get all node positions for rendering
         * @details Fixed: Use traditional loop instead of range-based loop
         *          because btAlignedObjectArray doesn't support begin()/end()
         */
        std::vector<Vec3> getNodePositions() const;

        /**
         * @brief Get all node normals for rendering
         * @details Fixed: Use traditional loop instead of range-based loop
         */
        std::vector<Vec3> getNodeNormals() const;

        /**
         * @brief Get face indices for rendering
         * @details Fixed: Use traditional loop instead of range-based loop
         */
        std::vector<Int> getFaceIndices() const;

        /**
         * @brief Get bounding box
         */
        AABB getBoundingBox() const;

        /**
         * @brief Get total mass
         */
        Float getTotalMass() const;

        /**
         * @brief Get volume
         */
        Float getVolume() const;

        /**
         * @brief Get center of mass
         */
        Vec3 getCenterOfMass() const;

        // ============================================================================
        // Configuration
        // ============================================================================

        void setMaterial(const SoftBodyMaterial& material);

        void setTotalMass(const Float mass);

        void setCollisionGroup(const std::uint16_t group) {
            // Would need to re-add to world with new group
            collisionGroup_ = group;
        }

        void setCollisionMask(const std::uint16_t mask) {
            // Would need to re-add to world with new mask
            collisionMask_ = mask;
        }

        btSoftBody* getBulletSoftBody() { return bulletSoftBody_; }
        const btSoftBody* getBulletSoftBody() const { return bulletSoftBody_; }

        const SoftBodyConfig& getConfig() const { return config_; }

    // ============================================================================
    // PROTECTED SECTION - Accessible to derived classes
    // ============================================================================
    protected:
        btSoftBodyWorldInfo* worldInfo_;
        btSoftBody* bulletSoftBody_;        // Now protected instead of private
        SoftBodyConfig config_;             // Now protected instead of private
        std::uint16_t collisionGroup_ = DEFAULT;
        std::uint16_t collisionMask_ = ALL;

        /**
         * @brief Apply configuration to bullet soft body
         * @details Now protected so derived classes can access it
         */
        void applyConfig() const;

        /**
         * @brief Apply material properties to bullet soft body
         * @details Now protected so derived classes can access it
         */
        void applyMaterial() const;
    };

    // TODO: In the future, move it to the module manager, as it is a preset
    /**
     * @brief Specialized cloth simulation
     * @details Fixed: Now can access protected members from base class
     */
    class ClothSimulation : public SoftBody {
    public:
        struct ClothParams {
            Vec3 topLeft = Vec3(-1, 1, 0);
            Vec3 topRight = Vec3(1, 1, 0);
            Vec3 bottomLeft = Vec3(-1, -1, 0);
            Vec3 bottomRight = Vec3(1, -1, 0);
            Int resolutionX = 20;
            Int resolutionY = 20;
            bool fixTopCorners = true;
            bool fixAllTop = false;
            Float mass = 1.0f;
            SoftBodyMaterial material = SoftBodyMaterial::Cloth();
        };

        ClothSimulation(btSoftBodyWorldInfo* worldInfo, const ClothParams& params);

        /**
         * @brief Apply tearing at stressed links
         * @details Fixed: Now can access protected m_bulletSoftBody member
         */
        void applyTearing(const Float tearThreshold = 10.0f) const;
    };

    /**
     * @brief Specialized rope/cable simulation
     * @details Fixed: Now can access protected members from base class
     */
    class RopeSimulation : public SoftBody {
    public:
        struct RopeParams {
            Vec3 startPoint = Vec3(0, 1, 0);
            Vec3 endPoint = Vec3(0, -1, 0);
            Int segments = 20;
            Float thickness = 0.05f;
            Float mass = 1.0f;
            bool fixStart = true;
            bool fixEnd = false;
            SoftBodyMaterial material = SoftBodyMaterial::Rope();
        };

        RopeSimulation(btSoftBodyWorldInfo* worldInfo, const RopeParams& params);

        Float getThickness() const { return thickness_; }

    private:
        Float thickness_;
    };
} // namespace engine::physics
