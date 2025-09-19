/**
 * @file InputSnapshot.h
 * @brief Consolidated input state for deterministic frame-based processing
 * @author Andrés Guerrero
 * @date 10-09-2025
 *
 * The InputSnapshot represents the complete state of all input devices at a
 * specific frame. This is critical for deterministic gameplay, replay systems,
 * and networked multiplayer.
 */

#pragma once

#include "../devices/keyboard/KeyboardState.h"
#include "../devices/mouse/MouseState.h"

#include <array>
#include <ranges>
#include <unordered_map>

#include "../devices/gamepad/GamepadState.h"


namespace engine::input {
    /**
     * @brief Touch input state
     */
    struct TouchState {
        static constexpr std::size_t MAX_TOUCHES = 10;

        struct TouchPoint {
            std::uint32_t id;
            math::Vec2 position;
            math::Vec2 startPosition;
            math::Vec2 delta;
            float pressure;
            TouchPhase phase;
            InputTimestamp startTime;
            bool active;

            TouchPoint() noexcept
                : id(0)
                  , position(0, 0)
                  , startPosition(0, 0)
                  , delta(0, 0)
                  , pressure(1.0f)
                  , phase(TouchPhase::ENDED)
                  , active(false) {
            }
        };

        std::array<TouchPoint, MAX_TOUCHES> touches;
        std::size_t activeTouchCount;

        // Gesture recognition
        GestureType lastGesture;
        math::Vec2 gestureCenter;
        float gestureScale;
        float gestureRotation;

        TouchState() noexcept
            : touches{}
              , activeTouchCount(0)
              , lastGesture(GestureType::NONE)
              , gestureCenter(0, 0)
              , gestureScale(1.0f)
              , gestureRotation(0.0f) {
        }

        /**
         * @brief Get active touch by index
         */
        [[nodiscard]] const TouchPoint* getTouch(const std::size_t index) const noexcept {
            if (index >= activeTouchCount) return nullptr;

            std::size_t activeIndex = 0;
            for (const auto& touch : touches) {
                if (touch.active) {
                    if (activeIndex == index) return &touch;
                    activeIndex++;
                }
            }
            return nullptr;
        }

        /**
         * @brief Find touch by ID
         */
        TouchPoint* findTouch(const std::uint32_t touchId) noexcept {
            for (auto& touch : touches) {
                if (touch.active && touch.id == touchId) {
                    return &touch;
                }
            }
            return nullptr;
        }

        /**
         * @brief Clear frame-specific state
         */
        void clearFrameState() noexcept {
            for (auto& touch : touches) {
                touch.delta = math::VEC2_ZERO;

                // Clear ended touches
                if (touch.phase == TouchPhase::ENDED ||
                    touch.phase == TouchPhase::CANCELLED) {
                    touch.active = false;
                }
            }

            // Recalculate active count
            activeTouchCount = 0;
            for (const auto& touch : touches) {
                if (touch.active) activeTouchCount++;
            }
        }
    };

    // ============================================================================
    // Action State
    // ============================================================================

    /**
     * @brief State of a mapped action
     */
    struct ActionState {
        ActionID actionId;
        ActionType type;

        // Value based on action type
        union {
            bool buttonValue;
            float axis1DValue;
            math::Vec2 axis2DValue;

            struct {
                float x, y, z;
            } axis3DValue;
        };

        // State tracking
        TriggerEvent triggerState;
        float heldDuration;
        bool consumed;

        ActionState() noexcept
            : actionId(INVALID_ACTION_ID)
              , type(ActionType::BUTTON)
              , buttonValue(false)
              , triggerState(TriggerEvent::NONE)
              , heldDuration(0.0f)
              , consumed(false) {
        }

        /**
         * @brief Get action value as float
         */
        [[nodiscard]] float getFloatValue() const noexcept {
            switch (type) {
            case ActionType::BUTTON: return buttonValue ? 1.0f : 0.0f;
            case ActionType::AXIS_1D: return axis1DValue;
            case ActionType::AXIS_2D: return axis2DValue.length();
            case ActionType::AXIS_3D: return std::sqrt(axis3DValue.x * axis3DValue.x +
                    axis3DValue.y * axis3DValue.y +
                    axis3DValue.z * axis3DValue.z);
            default: return 0.0f;
            }
        }

        /**
         * @brief Check if action is triggered
         */
        [[nodiscard]] bool isTriggered() const noexcept {
            return triggerState == TriggerEvent::STARTED ||
                triggerState == TriggerEvent::ONGOING;
        }
    };

    // ============================================================================
    // Complete Input Snapshot
    // ============================================================================

    /**
     * @brief Complete input state for a single frame
     *
     * This structure contains the entire input state at a specific point in time.
     * It's designed to be deterministic and serializable for replay/networking.
     */
    struct InputSnapshot {
        // Frame identification
        std::uint64_t frameNumber;
        InputTimestamp timestamp;
        float deltaTime; // Time since last snapshot

        // Device states
        KeyboardState keyboard;
        MouseState mouse;
        std::array<GamepadState, MAX_PLAYERS> gamepads;
        TouchState touch;

        // Mapped actions
        std::unordered_map<ActionID, ActionState> actions;

        // Active context
        std::string activeContext;
        ContextPriority contextPriority;

        // Global input state
        bool hasFocus;
        bool acceptingTextInput;
        bool consumeAllInput; // For modal UI

        // Statistics
        std::uint32_t eventCount; // Events processed this frame
        std::uint32_t droppedEvents; // Events dropped due to overflow

        InputSnapshot() noexcept
            : frameNumber(0)
              , timestamp(InputTimestamp::clock::now())
              , deltaTime(0.0f)
              , activeContext("Gameplay")
              , contextPriority(ContextPriority::NORMAL)
              , hasFocus(true)
              , acceptingTextInput(false)
              , consumeAllInput(false)
              , eventCount(0)
              , droppedEvents(0) {
        }

        /**
         * @brief Clear all frame-specific state
         */
        void clearFrameState() noexcept {
            keyboard.clearFrameState();
            mouse.clearFrameState();
            for (auto& gamepad : gamepads) {
                gamepad.clearFrameState();
            }
            touch.clearFrameState();

            // Clear action trigger states
            for (auto& action : actions | std::views::values) {
                if (action.triggerState == TriggerEvent::STARTED) {
                    action.triggerState = TriggerEvent::ONGOING;
                }
                else if (action.triggerState == TriggerEvent::COMPLETED) {
                    action.triggerState = TriggerEvent::NONE;
                }
            }

            eventCount = 0;
            droppedEvents = 0;
        }

        /**
         * @brief Advance to next frame
         */
        void nextFrame(const float dt) noexcept {
            frameNumber++;
            deltaTime = dt;
            timestamp = InputTimestamp::clock::now();

            // Update held durations
            for (auto& action : actions | std::views::values) {
                if (action.isTriggered()) {
                    action.heldDuration += dt;
                }
                else {
                    action.heldDuration = 0.0f;
                }
            }

            // Update gamepad rumble
            for (auto& gamepad : gamepads) {
                if (gamepad.rumbleDuration > 0) {
                    gamepad.rumbleDuration -= dt;
                    if (gamepad.rumbleDuration <= 0) {
                        gamepad.leftMotor = 0;
                        gamepad.rightMotor = 0;
                        gamepad.rumbleDuration = 0;
                    }
                }
            }

            clearFrameState();
        }

        /**
         * @brief Check if any input is active
         */
        [[nodiscard]] bool hasAnyInput() const noexcept {
            if (keyboard.hasAnyKeyPressed()) return true;
            if (mouse.buttons.any()) return true;

            for (const auto& gamepad : gamepads) {
                if (gamepad.isConnected && gamepad.hasAnyInput()) return true;
            }

            if (touch.activeTouchCount > 0) return true;

            return false;
        }

        /**
         * @brief Get active gamepad for player
         */
        [[nodiscard]] const GamepadState* getGamepad(const PlayerID player) const noexcept {
            if (player >= MAX_PLAYERS) return nullptr;
            const auto& gamepad = gamepads[player];
            return gamepad.isConnected ? &gamepad : nullptr;
        }

        /**
         * @brief Get mutable gamepad for player
         */
        GamepadState* getGamepadMutable(const PlayerID player) noexcept {
            if (player >= MAX_PLAYERS) return nullptr;
            auto& gamepad = gamepads[player];
            return gamepad.isConnected ? &gamepad : nullptr;
        }

        /**
         * @brief Get action state by ID
         */
        [[nodiscard]] const ActionState* getAction(const ActionID id) const noexcept {
            const auto it = actions.find(id);
            return it != actions.end() ? &it->second : nullptr;
        }

        /**
         * @brief Check if action is triggered
         */
        [[nodiscard]] bool isActionTriggered(const ActionID id) const noexcept {
            auto* action = getAction(id);
            return action && action->isTriggered();
        }

        /**
         * @brief Get action float value
         */
        [[nodiscard]] float getActionValue(const ActionID id) const noexcept {
            auto* action = getAction(id);
            return action ? action->getFloatValue() : 0.0f;
        }

        /**
         * @brief Get action 2D value
         */
        [[nodiscard]] math::Vec2 getActionValue2D(const ActionID id) const noexcept {
            auto* action = getAction(id);
            if (!action || action->type != ActionType::AXIS_2D) {
                return math::VEC2_ZERO;
            }
            return action->axis2DValue;
        }

        /**
         * @brief Clone snapshot for deterministic replay
         */
        [[nodiscard]] InputSnapshot clone() const noexcept {
            return *this; // Default copy is sufficient
        }

        /**
         * @brief Serialize snapshot for networking/replay
         */
        [[nodiscard]] std::vector<std::uint8_t> serialize() const {
            std::vector<std::uint8_t> buffer;

            // Serializar campos básicos
            auto writeValue = [&buffer](const auto& value) {
                const auto* data = reinterpret_cast<const std::uint8_t*>(&value);
                buffer.insert(buffer.end(), data, data + sizeof(value));
            };

            writeValue(frameNumber);
            writeValue(timestamp);
            writeValue(deltaTime);
            writeValue(keyboard);
            writeValue(mouse);
            writeValue(gamepads);
            writeValue(touch);

            // Serializar string
            const auto contextSize = static_cast<std::uint32_t>(activeContext.size());
            writeValue(contextSize);
            buffer.insert(buffer.end(), activeContext.begin(), activeContext.end());

            writeValue(contextPriority);
            writeValue(hasFocus);
            writeValue(acceptingTextInput);
            writeValue(consumeAllInput);
            writeValue(eventCount);
            writeValue(droppedEvents);

            // Serializar mapa de acciones
            const auto actionsSize = static_cast<std::uint32_t>(actions.size());
            writeValue(actionsSize);
            for (const auto& [id, state] : actions) {
                writeValue(id);
                writeValue(state);
            }

            return buffer;
        }

        /**
         * @brief Deserialize snapshot
         */
        static InputSnapshot deserialize(const std::vector<std::uint8_t>& buffer) {
            if (buffer.empty()) return {};

            InputSnapshot snapshot;
            std::size_t offset = 0;

            auto readValue = [&buffer, &offset](auto& value) -> bool {
                if (offset + sizeof(value) > buffer.size()) return false;
                // TODO: URGENTE Revisar esto detenidamente ya que GamepadState es un tipo muy complejo
                // y necesitaría implementar metodos de serializacion personalizados para este dato
                std::memcpy(&value, buffer.data() + offset, sizeof(value));
                offset += sizeof(value);
                return true;
            };

            if (!readValue(snapshot.frameNumber) ||
                !readValue(snapshot.timestamp) ||
                !readValue(snapshot.deltaTime) ||
                !readValue(snapshot.keyboard) ||
                !readValue(snapshot.mouse) ||
                // !readValue(snapshot.gamepads) || // => GamepadState complejo para hacer el memcpy de arriba
                !readValue(snapshot.touch)) {
                return {};
            }

            // Leer string
            std::uint32_t contextSize;
            if (!readValue(contextSize) || offset + contextSize > buffer.size()) return {};

            snapshot.activeContext.resize(contextSize);
            std::memcpy(snapshot.activeContext.data(), buffer.data() + offset, contextSize);
            offset += contextSize;

            if (!readValue(snapshot.contextPriority) ||
                !readValue(snapshot.hasFocus) ||
                !readValue(snapshot.acceptingTextInput) ||
                !readValue(snapshot.consumeAllInput) ||
                !readValue(snapshot.eventCount) ||
                !readValue(snapshot.droppedEvents)) {
                return {};
            }

            // Leer mapa de acciones
            std::uint32_t actionsSize;
            if (!readValue(actionsSize)) return {};

            for (std::uint32_t i = 0; i < actionsSize; ++i) {
                ActionID id;
                ActionState state;
                if (!readValue(id) || !readValue(state)) return {};
                snapshot.actions[id] = state;
            }

            return snapshot;
        }

        /**
         * @brief Get memory size of snapshot
         */
        static constexpr std::size_t getMemorySize() noexcept {
            return sizeof(InputSnapshot);
        }
    };

    // ============================================================================
    // Snapshot History for Replay
    // ============================================================================

    /**
     * @brief Ring buffer for input history
     */
    class InputHistory {
    public:
        static constexpr std::size_t DEFAULT_HISTORY_SIZE = 600; // 10 seconds at 60fps

        explicit InputHistory(const std::size_t maxFrames = DEFAULT_HISTORY_SIZE)
            : maxFrames_(maxFrames)
              , writeIndex_(0)
              , frameCount_(0) {
            history_.reserve(maxFrames);
        }

        /**
         * @brief Add snapshot to history
         */
        void recordSnapshot(const InputSnapshot& snapshot) {
            if (history_.size() < maxFrames_) {
                history_.push_back(snapshot);
            }
            else {
                history_[writeIndex_] = snapshot;
            }

            writeIndex_ = (writeIndex_ + 1) % maxFrames_;
            frameCount_++;
        }

        /**
         * @brief Get snapshot at relative frame offset
         */
        [[nodiscard]] const InputSnapshot* getSnapshot(const int frameOffset) const noexcept {
            if (frameOffset >= 0 || static_cast<std::size_t>(-frameOffset) > history_.size()) {
                return nullptr;
            }

            const std::size_t index = (writeIndex_ + maxFrames_ + frameOffset) % maxFrames_;
            return &history_[index];
        }

        /**
         * @brief Get snapshot by absolute frame number
         */
        [[nodiscard]] const InputSnapshot* getSnapshotByFrame(const std::uint64_t frameNumber) const noexcept {
            if (history_.empty()) return nullptr;

            // Search backwards from most recent
            for (int i = -1; i >= -static_cast<int>(history_.size()); --i) {
                if (auto* snapshot = getSnapshot(i); snapshot && snapshot->frameNumber == frameNumber) {
                    return snapshot;
                }
            }
            return nullptr;
        }

        /**
         * @brief Clear history
         */
        void clear() noexcept {
            history_.clear();
            writeIndex_ = 0;
            frameCount_ = 0;
        }

        /**
         * @brief Get total frames recorded
         */
        [[nodiscard]] std::uint64_t getFrameCount() const noexcept {
            return frameCount_;
        }

        /**
         * @brief Export history for replay file
         */
        [[nodiscard]] std::vector<std::uint8_t> exportHistory() const {
            std::vector<std::uint8_t> data;

            // Header
            data.resize(sizeof(std::uint64_t) * 2);
            *reinterpret_cast<std::uint64_t*>(data.data()) = history_.size();
            *reinterpret_cast<std::uint64_t*>(data.data() + sizeof(std::uint64_t)) = frameCount_;

            // Snapshots
            for (const auto& snapshot : history_) {
                auto serialized = snapshot.serialize();
                data.insert(data.end(), serialized.begin(), serialized.end());
            }

            return data;
        }

    private:
        std::vector<InputSnapshot> history_;
        std::size_t maxFrames_;
        std::size_t writeIndex_;
        std::uint64_t frameCount_;
    };
} // namespace engine::input
