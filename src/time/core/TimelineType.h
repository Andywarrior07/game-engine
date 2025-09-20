/**
 * @file TimelineType.h
 * @brief Timeline type definitions, priorities, and capability metadata
 * @details Defines the different timeline types supported by the time system,
 *          their execution priorities, capabilities, and type-safe identification.
 *          Each timeline type has specific characteristics that affect how time
 *          flows through them and what operations they support.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "TimeTypes.h"

#include <string_view>
#include <array>
#include <bitset>

namespace engine::time {
    // =============================================================================
    // Timeline Type Enumeration
    // =============================================================================

    /**
     * @brief Core timeline types supported by the time system
     * @details Each timeline type has specific behavior and use cases:
     *          - Real: Never pauseable, for networking and profiling
     *          - Game: Main gameplay timeline, pauseable and scalable
     *          - UI: Menu/HUD timeline, independent of game pause
     *          - Physics: Fixed timestep for deterministic simulation
     *          - Audio: Audio synchronization, can be paused but not scaled
     *          - Custom: User-defined timelines for special purposes
     */
    enum class TimelineType : std::uint8_t {
        REAL_TIME = 0, ///< Real-world time, never pauses or scales
        GAME_TIME = 1, ///< Main gameplay time, fully controllable
        UI_TIME = 2, ///< UI/Menu time, runs during game pause
        PHYSICS_TIME = 3, ///< Physics simulation time, fixed timestep
        AUDIO_TIME = 4, ///< Audio synchronization time
        CUSTOM_1 = 5, ///< First custom timeline slot
        CUSTOM_2 = 6, ///< Second custom timeline slot
        CUSTOM_3 = 7, ///< Third custom timeline slot

        COUNT = 8, ///< Total number of timeline types
        INVALID = 255 ///< Invalid timeline type sentinel
    };

    // =============================================================================
    // Timeline Capability Flags
    // =============================================================================

    /**
     * @brief Capability flags defining what operations a timeline supports
     * @details Bitflags used to query and validate timeline operations.
     *          Checked at compile-time where possible for performance.
     */
    enum class TimelineCapability : std::uint32_t {
        NONE = 0, ///< No special capabilities
        PAUSEABLE = 1 << 0, ///< Timeline can be paused
        SCALABLE = 1 << 1, ///< Timeline supports time scaling
        REVERSIBLE = 1 << 2, ///< Timeline can run backwards
        SERIALIZABLE = 1 << 3, ///< Timeline state can be saved/loaded
        INTERPOLATABLE = 1 << 4, ///< Supports frame interpolation
        FIXED_TIMESTEP = 1 << 5, ///< Uses fixed timestep updates
        VARIABLE_TIMESTEP = 1 << 6, ///< Uses variable timestep updates
        NETWORK_SYNCED = 1 << 7, ///< Synchronized across network
        FRAME_INDEPENDENT = 1 << 8, ///< Updates independent of frame rate
        RECORDABLE = 1 << 9, ///< Can record timeline for replay
        DEBUGGABLE = 1 << 10, ///< Supports debug time controls
        HIGH_PRECISION = 1 << 11, ///< Requires high precision timing
        LOW_LATENCY = 1 << 12, ///< Requires minimal input latency
        DETERMINISTIC = 1 << 13, ///< Guarantees deterministic behavior
        ALWAYS_ACTIVE = 1 << 14, ///< Never stops updating
    };

    /**
     * @brief Bitwise OR operator for combining capabilities
     */
    [[nodiscard]] constexpr TimelineCapability operator|(TimelineCapability a, TimelineCapability b) noexcept {
        return static_cast<TimelineCapability>(
            static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b)
        );
    }

    /**
     * @brief Bitwise AND operator for checking capabilities
     */
    [[nodiscard]] constexpr TimelineCapability operator&(TimelineCapability a, TimelineCapability b) noexcept {
        return static_cast<TimelineCapability>(
            static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b)
        );
    }

    /**
     * @brief Check if timeline has specific capability
     */
    [[nodiscard]] constexpr bool hasCapability(TimelineCapability caps, TimelineCapability cap) noexcept {
        return (caps & cap) == cap;
    }

    // =============================================================================
    // Timeline Priority
    // =============================================================================

    /**
     * @brief Update priority for timeline execution order
     * @details Higher priority timelines update first.
     *          Critical for dependency management between timelines.
     */
    enum class TimelinePriority : std::uint8_t {
        CRITICAL = 0, ///< Highest priority (real-time, networking)
        HIGH = 64, ///< High priority (physics simulation)
        NORMAL = 128, ///< Normal priority (gameplay)
        LOW = 192, ///< Low priority (UI, effects)
        IDLE = 255 ///< Lowest priority (background tasks)
    };

    // =============================================================================
    // Timeline Metadata
    // =============================================================================

    /**
     * @brief Complete metadata for a timeline type
     * @details Compile-time configuration for timeline behavior.
     *          Used to initialize and validate timeline instances.
     */
    struct TimelineMetadata {
        TimelineType type; ///< Timeline type identifier
        std::string_view name; ///< Human-readable name
        TimelineCapability capabilities; ///< Supported capabilities
        TimelinePriority defaultPriority; ///< Default execution priority
        Duration defaultTimestep; ///< Default update timestep
        TimeScale defaultScale; ///< Default time scale
        bool defaultPaused; ///< Initially paused?
        std::size_t memoryBudget; ///< Memory budget in bytes
        std::uint16_t maxTimers; ///< Maximum concurrent timers
        std::uint8_t updateFrequency; ///< Updates per second (0 = every frame)
    };

    /**
     * @brief Static metadata for all timeline types
     * @details Compile-time configuration ensures zero runtime overhead.
     *          These settings define the behavior contract for each timeline type.
     */
    constexpr std::array<TimelineMetadata, static_cast<std::size_t>(TimelineType::COUNT)> TIMELINE_METADATA = {
        {
            // REAL_TIME - Never pauses, always 1:1 with wall clock
            {
                TimelineType::REAL_TIME,
                "RealTime",
                TimelineCapability::ALWAYS_ACTIVE |
                TimelineCapability::HIGH_PRECISION |
                TimelineCapability::NETWORK_SYNCED |
                TimelineCapability::VARIABLE_TIMESTEP |
                TimelineCapability::FRAME_INDEPENDENT,
                TimelinePriority::CRITICAL,
                Duration::zero(), // Variable timestep
                1.0, // Always 1:1 scale
                false, // Never paused
                1024 * 1024, // 1MB memory budget
                256, // Few timers needed
                0 // Update every frame
            },

            // GAME_TIME - Main gameplay timeline, fully controllable
            {
                TimelineType::GAME_TIME,
                "GameTime",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::SCALABLE |
                TimelineCapability::REVERSIBLE |
                TimelineCapability::SERIALIZABLE |
                TimelineCapability::INTERPOLATABLE |
                TimelineCapability::VARIABLE_TIMESTEP |
                TimelineCapability::RECORDABLE |
                TimelineCapability::DEBUGGABLE,
                TimelinePriority::NORMAL,
                Duration::zero(), // Variable timestep
                1.0, // Normal speed default
                false, // Not paused initially
                4 * 1024 * 1024, // 4MB memory budget
                2048, // Many gameplay timers
                0 // Update every frame
            },

            // UI_TIME - Runs independently of game pause
            {
                TimelineType::UI_TIME,
                "UITime",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::SCALABLE |
                TimelineCapability::VARIABLE_TIMESTEP |
                TimelineCapability::FRAME_INDEPENDENT |
                TimelineCapability::LOW_LATENCY,
                TimelinePriority::LOW,
                Duration::zero(), // Variable timestep
                1.0, // Normal speed
                false, // Not paused
                2 * 1024 * 1024, // 2MB memory budget
                512, // Moderate timer count
                60 // 60Hz update rate
            },

            // PHYSICS_TIME - Deterministic fixed timestep
            {
                TimelineType::PHYSICS_TIME,
                "PhysicsTime",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::SCALABLE |
                TimelineCapability::SERIALIZABLE |
                TimelineCapability::FIXED_TIMESTEP |
                TimelineCapability::DETERMINISTIC |
                TimelineCapability::HIGH_PRECISION |
                TimelineCapability::RECORDABLE,
                TimelinePriority::HIGH,
                constants::DEFAULT_FIXED_TIMESTEP, // 60Hz fixed
                1.0, // Normal speed
                false, // Not paused
                8 * 1024 * 1024, // 8MB memory budget
                128, // Few physics timers
                60 // 60Hz fixed rate
            },

            // AUDIO_TIME - Audio synchronization timeline
            {
                TimelineType::AUDIO_TIME,
                "AudioTime",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::HIGH_PRECISION |
                TimelineCapability::LOW_LATENCY |
                TimelineCapability::FRAME_INDEPENDENT |
                TimelineCapability::VARIABLE_TIMESTEP,
                TimelinePriority::HIGH,
                Duration::zero(), // Variable timestep
                1.0, // No scaling (pitch issues)
                false, // Not paused
                2 * 1024 * 1024, // 2MB memory budget
                256, // Audio event timers
                0 // Update with audio thread
            },

            // CUSTOM_1 - User-defined timeline
            {
                TimelineType::CUSTOM_1,
                "Custom1",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::SCALABLE |
                TimelineCapability::VARIABLE_TIMESTEP,
                TimelinePriority::NORMAL,
                Duration::zero(), // Variable by default
                1.0, // Normal speed
                false, // Not paused
                1024 * 1024, // 1MB memory budget
                256, // Moderate timers
                0 // Update every frame
            },

            // CUSTOM_2 - User-defined timeline
            {
                TimelineType::CUSTOM_2,
                "Custom2",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::SCALABLE |
                TimelineCapability::VARIABLE_TIMESTEP,
                TimelinePriority::NORMAL,
                Duration::zero(), // Variable by default
                1.0, // Normal speed
                false, // Not paused
                1024 * 1024, // 1MB memory budget
                256, // Moderate timers
                0 // Update every frame
            },

            // CUSTOM_3 - User-defined timeline
            {
                TimelineType::CUSTOM_3,
                "Custom3",
                TimelineCapability::PAUSEABLE |
                TimelineCapability::SCALABLE |
                TimelineCapability::VARIABLE_TIMESTEP,
                TimelinePriority::NORMAL,
                Duration::zero(), // Variable by default
                1.0, // Normal speed
                false, // Not paused
                1024 * 1024, // 1MB memory budget
                256, // Moderate timers
                0 // Update every frame
            }
        }
    };

    // =============================================================================
    // Timeline Utility Functions
    // =============================================================================

    /**
     * @brief Get metadata for a specific timeline type
     * @param type Timeline type to query
     * @return Reference to timeline metadata
     * @throws std::out_of_range if type is invalid
     */
    [[nodiscard]] inline const TimelineMetadata& getTimelineMetadata(TimelineType type) {
        if (type >= TimelineType::COUNT) {
            throw std::out_of_range("Invalid timeline type");
        }
        return TIMELINE_METADATA[static_cast<std::size_t>(type)];
    }

    /**
     * @brief Get human-readable name for timeline type
     * @param type Timeline type
     * @return String view of timeline name
     */
    [[nodiscard]] constexpr std::string_view getTimelineName(TimelineType type) noexcept {
        if (type >= TimelineType::COUNT) {
            return "Invalid";
        }
        return TIMELINE_METADATA[static_cast<std::size_t>(type)].name;
    }

    /**
     * @brief Check if timeline type supports a specific capability
     * @param type Timeline type to check
     * @param capability Capability to query
     * @return True if timeline has the capability
     */
    [[nodiscard]] constexpr bool timelineHasCapability(
        TimelineType type,
        const TimelineCapability capability) noexcept {
        if (type >= TimelineType::COUNT) {
            return false;
        }
        return hasCapability(
            TIMELINE_METADATA[static_cast<std::size_t>(type)].capabilities,
            capability
        );
    }

    /**
     * @brief Check if timeline type can be paused
     */
    [[nodiscard]] constexpr bool canPause(const TimelineType type) noexcept {
        return timelineHasCapability(type, TimelineCapability::PAUSEABLE);
    }

    /**
     * @brief Check if timeline type supports time scaling
     */
    [[nodiscard]] constexpr bool canScale(const TimelineType type) noexcept {
        return timelineHasCapability(type, TimelineCapability::SCALABLE);
    }

    /**
     * @brief Check if timeline uses fixed timestep
     */
    [[nodiscard]] constexpr bool isFixedTimestep(const TimelineType type) noexcept {
        return timelineHasCapability(type, TimelineCapability::FIXED_TIMESTEP);
    }

    /**
     * @brief Check if timeline is deterministic
     */
    [[nodiscard]] constexpr bool isDeterministic(const TimelineType type) noexcept {
        return timelineHasCapability(type, TimelineCapability::DETERMINISTIC);
    }

    /**
     * @brief Get default priority for timeline type
     */
    [[nodiscard]] constexpr TimelinePriority getDefaultPriority(TimelineType type) noexcept {
        if (type >= TimelineType::COUNT) {
            return TimelinePriority::IDLE;
        }
        return TIMELINE_METADATA[static_cast<std::size_t>(type)].defaultPriority;
    }

    /**
     * @brief Calculate total memory budget for all timelines
     */
    [[nodiscard]] constexpr std::size_t getTotalTimelineMemoryBudget() noexcept {
        std::size_t total = 0;
        for (const auto& metadata : TIMELINE_METADATA) {
            total += metadata.memoryBudget;
        }
        return total;
    }

    // =============================================================================
    // Timeline ID Management
    // =============================================================================

    /**
     * @brief Type-safe timeline identifier with validation
     * @details Wraps TimelineID with type safety and metadata access.
     *          Zero-cost abstraction with validation in debug builds.
     */
    class TimelineIdentifier {
    public:
        /**
         * @brief Default constructor creates invalid identifier
         */
        constexpr TimelineIdentifier() noexcept
            : id_(constants::INVALID_TIMELINE_ID), type_(TimelineType::INVALID) {
        }

        /**
         * @brief Construct from timeline type
         * @param type Timeline type
         */
        explicit constexpr TimelineIdentifier(TimelineType type) noexcept
            : id_(static_cast<TimelineID>(type)), type_(type) {
        }

        /**
         * @brief Construct from raw ID (validates type)
         * @param id Raw timeline ID
         */
        explicit constexpr TimelineIdentifier(TimelineID id) noexcept
            : id_(id),
              type_(id < static_cast<TimelineID>(TimelineType::COUNT)
                        ? static_cast<TimelineType>(id)
                        : TimelineType::INVALID) {
        }

        /**
         * @brief Get raw timeline ID
         */
        [[nodiscard]] constexpr TimelineID getId() const noexcept { return id_; }

        /**
         * @brief Get timeline type
         */
        [[nodiscard]] constexpr TimelineType getType() const noexcept { return type_; }

        /**
         * @brief Check if identifier is valid
         */
        [[nodiscard]] constexpr bool isValid() const noexcept {
            return type_ != TimelineType::INVALID;
        }

        /**
         * @brief Get timeline metadata
         */
        [[nodiscard]] const TimelineMetadata& getMetadata() const {
            return getTimelineMetadata(type_);
        }

        /**
         * @brief Equality comparison
         */
        [[nodiscard]] constexpr bool operator==(const TimelineIdentifier& other) const noexcept = default;

        /**
         * @brief Ordering for container storage
         */
        [[nodiscard]] constexpr auto operator<=>(const TimelineIdentifier& other) const noexcept = default;

    private:
        TimelineID id_; ///< Raw timeline identifier
        TimelineType type_; ///< Validated timeline type
    };
} // namespace engine::time

// Hash specialization for TimelineIdentifier
template <>
struct std::hash<engine::time::TimelineIdentifier> {
    [[nodiscard]] std::size_t operator()(const engine::time::TimelineIdentifier& id) const noexcept {
        return std::hash<engine::time::TimelineID>{}(id.getId());
    }
}; // namespace std
