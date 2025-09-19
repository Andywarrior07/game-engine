/**
 * @file InputEvent.h
 * @brief Raw input event structure for event-driven input handling
 * @author Andrés Guerrero
 * @date 10-09-2025
 *
 * Represents a single input event with timestamp for deterministic replay.
 * Events are allocated from memory pools for zero-allocation in hot paths.
 */

#pragma once

#include "InputTypes.h"

#include "../../memory/MemorySystem.h"

#include <variant>

namespace engine::input {
    // ============================================================================
    // Event Data Structures
    // ============================================================================

    /**
     * @brief Keyboard event data
     */
    struct KeyboardEventData {
        KeyCode key;
        KeyModifier modifiers;
        bool isRepeat;
        std::uint32_t scancode; // Platform-specific scancode

        KeyboardEventData() noexcept
            : key(KeyCode::UNKNOWN)
              , modifiers(KeyModifier::NONE)
              , isRepeat(false)
              , scancode(0) {
        }

        KeyboardEventData(const KeyCode k, const KeyModifier mods, const bool repeat = false, const std::uint32_t scan = 0) noexcept
            : key(k)
              , modifiers(mods)
              , isRepeat(repeat)
              , scancode(scan) {
        }
    };

    /**
     * @brief Text input event data (for IME support)
     */
    struct TextEventData {
        char text[32]{}; // UTF-8 encoded text
        std::uint32_t codepoint; // Unicode codepoint

        TextEventData() noexcept
            : codepoint(0) {
        }

        TextEventData(const char* str, const std::uint32_t cp) noexcept
            : codepoint(cp) {
            std::size_t len = 0;
            while (str[len] && len < 31) {
                text[len] = str[len];
                len++;
            }
            text[len] = '\0';
        }
    };

    /**
     * @brief Mouse button event data
     */
    struct MouseButtonEventData {
        MouseButton button;
        math::Vec2 position; // Screen position
        std::uint8_t clickCount; // For double/triple clicks

        MouseButtonEventData() noexcept
            : button(MouseButton::NONE)
              , position(0, 0)
              , clickCount(1) {
        }

        MouseButtonEventData(const MouseButton b, const math::Vec2& pos, const std::uint8_t clicks = 1) noexcept
            : button(b)
              , position(pos)
              , clickCount(clicks) {
        }
    };

    /**
     * @brief Mouse motion event data
     */
    struct MouseMotionEventData {
        math::Vec2 position; // Current position
        math::Vec2 delta; // Movement delta
        math::Vec2 normalizedDelta; // Delta normalized to [-1, 1]
        bool isRelative; // True if in relative/locked mode

        MouseMotionEventData() noexcept
            : position(0, 0)
              , delta(0, 0)
              , normalizedDelta(0, 0)
              , isRelative(false) {
        }

        MouseMotionEventData(const math::Vec2& pos, const math::Vec2& d, const bool relative = false) noexcept
            : position(pos)
              , delta(d)
              , normalizedDelta(d)
              , isRelative(relative) {
        }
    };

    /**
     * @brief Mouse wheel event data
     */
    struct MouseWheelEventData {
        math::Vec2 delta; // X for horizontal, Y for vertical
        math::Vec2 position; // Mouse position when scrolling
        bool isPrecise; // True for trackpad, false for discrete wheel

        MouseWheelEventData() noexcept
            : delta(0, 0)
              , position(0, 0)
              , isPrecise(false) {
        }

        MouseWheelEventData(const math::Vec2& d, const math::Vec2& pos, const bool precise = false) noexcept
            : delta(d)
              , position(pos)
              , isPrecise(precise) {
        }
    };

    /**
     * @brief Gamepad button event data
     */
    struct GamepadButtonEventData {
        GamepadButton button;
        float pressure; // For pressure-sensitive buttons (0-1)

        GamepadButtonEventData() noexcept
            : button(GamepadButton::NONE)
              , pressure(1.0f) {
        }

        explicit GamepadButtonEventData(const GamepadButton b, const float p = 1.0f) noexcept
            : button(b)
              , pressure(p) {
        }
    };

    /**
     * @brief Gamepad axis event data
     */
    struct GamepadAxisEventData {
        GamepadAxis axis;
        float value; // Raw value (-1 to 1 for sticks, 0 to 1 for triggers)
        float deltaValue; // Change from last frame

        GamepadAxisEventData() noexcept
            : axis(GamepadAxis::NONE)
              , value(0.0f)
              , deltaValue(0.0f) {
        }

        GamepadAxisEventData(const GamepadAxis a, const float v, const float dv = 0.0f) noexcept
            : axis(a)
              , value(v)
              , deltaValue(dv) {
        }
    };

    /**
     * @brief Touch event data
     */
    struct TouchEventData {
        std::uint32_t touchId; // Unique ID for this touch point
        math::Vec2 position; // Current position
        math::Vec2 startPosition; // Where touch began
        math::Vec2 delta; // Movement since last frame
        float pressure; // Touch pressure (0-1)
        TouchPhase phase;

        TouchEventData() noexcept
            : touchId(0)
              , position(0, 0)
              , startPosition(0, 0)
              , delta(0, 0)
              , pressure(1.0f)
              , phase(TouchPhase::BEGAN) {
        }

        TouchEventData(const std::uint32_t id, const math::Vec2& pos, const TouchPhase p) noexcept
            : touchId(id)
              , position(pos)
              , startPosition(pos)
              , delta(0, 0)
              , pressure(1.0f)
              , phase(p) {
        }
    };

    /**
     * @brief Gesture event data
     */
    struct GestureEventData {
        GestureType type;
        math::Vec2 position; // Center position of gesture
        math::Vec2 delta; // Movement/rotation/scale delta
        float scale; // For pinch gestures
        float rotation; // For rotation gestures
        std::uint8_t fingerCount;

        GestureEventData() noexcept
            : type(GestureType::NONE)
              , position(0, 0)
              , delta(0, 0)
              , scale(1.0f)
              , rotation(0.0f)
              , fingerCount(1) {
        }
    };

    /**
     * @brief Device connection event data
     */
    struct DeviceEventData {
        DeviceType type;
        DeviceID deviceId;
        std::uint8_t playerIndex;
        bool connected;

        DeviceEventData() noexcept
            : type(DeviceType::NONE)
              , deviceId(INVALID_DEVICE_ID)
              , playerIndex(0)
              , connected(false) {
        }

        DeviceEventData(const DeviceType t, const DeviceID id, const std::uint8_t player, const bool conn) noexcept
            : type(t)
              , deviceId(id)
              , playerIndex(player)
              , connected(conn) {
        }
    };

    // ============================================================================
    // Input Event Structure
    // ============================================================================

    /**
     * @brief Unified input event structure
     *
     * Uses std::variant for type-safe event data storage.
     * Events are timestamped for deterministic replay and networking.
     */
    struct InputEvent {
        using EventData = std::variant<
            std::monostate, // Empty state
            KeyboardEventData,
            TextEventData,
            MouseButtonEventData,
            MouseMotionEventData,
            MouseWheelEventData,
            GamepadButtonEventData,
            GamepadAxisEventData,
            TouchEventData,
            GestureEventData,
            DeviceEventData
        >;

        // Event metadata
        InputEventType type;
        DeviceID deviceId;
        PlayerID playerId;
        InputTimestamp timestamp;
        std::uint64_t frameNumber; // For frame-perfect replay

        // Event-specific data
        EventData data;

        // Event state tracking
        bool consumed; // Has this event been handled?
        bool synthetic; // Was this event generated programmatically?

        /**
         * @brief Default constructor
         */
        InputEvent() noexcept
            : type(InputEventType::NONE)
              , deviceId(INVALID_DEVICE_ID)
              , playerId(0)
              , timestamp(InputTimestamp::clock::now())
              , frameNumber(0)
              , data(std::monostate{})
              , consumed(false)
              , synthetic(false) {
        }

        /**
         * @brief Constructor with type and data
         */
        template <typename T>
        InputEvent(const InputEventType t, const DeviceID dev, const T& eventData) noexcept
            : type(t)
              , deviceId(dev)
              , playerId(0)
              , timestamp(InputTimestamp::clock::now())
              , frameNumber(0)
              , data(eventData)
              , consumed(false)
              , synthetic(false) {
        }

        /**
         * @brief Get event data of specific type
         */
        template <typename T>
        [[nodiscard]] const T* getEventData() const noexcept {
            return std::get_if<T>(&data);
        }

        /**
         * @brief Get mutable event data
         */
        template <typename T>
        T* getMutableEventData() noexcept {
            return std::get_if<T>(&data);
        }

        /**
         * @brief Check if event is of specific type
         */
        [[nodiscard]] bool isType(const InputEventType t) const noexcept {
            return type == t;
        }

        /**
         * @brief Mark event as consumed
         */
        void consume() noexcept {
            consumed = true;
        }

        /**
         * @brief Get time since event occurred
         */
        [[nodiscard]] InputDuration getAge() const noexcept {
            return InputTimestamp::clock::now() - timestamp;
        }

        /**
         * @brief Check if event is a press event
         */
        [[nodiscard]] bool isPress() const noexcept {
            return type == InputEventType::KEY_PRESSED ||
                type == InputEventType::MOUSE_BUTTON_PRESSED ||
                type == InputEventType::GAMEPAD_BUTTON_PRESSED ||
                type == InputEventType::TOUCH_BEGAN;
        }

        /**
         * @brief Check if event is a release event
         */
        [[nodiscard]] bool isRelease() const noexcept {
            return type == InputEventType::KEY_RELEASED ||
                type == InputEventType::MOUSE_BUTTON_RELEASED ||
                type == InputEventType::GAMEPAD_BUTTON_RELEASED ||
                type == InputEventType::TOUCH_ENDED;
        }

        /**
         * @brief Check if event is analog motion
         */
        [[nodiscard]] bool isAnalogMotion() const noexcept {
            return type == InputEventType::MOUSE_MOVED ||
                type == InputEventType::GAMEPAD_AXIS_MOVED ||
                type == InputEventType::GAMEPAD_TRIGGER_MOVED ||
                type == InputEventType::TOUCH_MOVED;
        }

        /**
         * @brief Clone event with new timestamp
         */
        [[nodiscard]] InputEvent clone() const noexcept {
            InputEvent copy = *this;
            copy.timestamp = InputTimestamp::clock::now();
            copy.consumed = false;
            return copy;
        }

        /**
         * @brief Get string representation for debugging
         */
        [[nodiscard]] std::string toString() const {
            std::string result = "InputEvent[";

            switch (type) {
            case InputEventType::KEY_PRESSED:
            case InputEventType::KEY_RELEASED: {
                if (auto* kbd = getEventData<KeyboardEventData>()) {
                    result += std::string(type == InputEventType::KEY_PRESSED ? "KeyPress: " : "KeyRelease: ") +
                        keyCodeToString(kbd->key);
                }
                break;
            }
            case InputEventType::MOUSE_BUTTON_PRESSED:
            case InputEventType::MOUSE_BUTTON_RELEASED: {
                if (auto* mouse = getEventData<MouseButtonEventData>()) {
                    result += std::string(type == InputEventType::MOUSE_BUTTON_PRESSED
                                              ? "MousePress: "
                                              : "MouseRelease: ") +
                        mouseButtonToString(mouse->button);
                }
                break;
            }
            case InputEventType::GAMEPAD_BUTTON_PRESSED:
            case InputEventType::GAMEPAD_BUTTON_RELEASED: {
                if (auto* gamepad = getEventData<GamepadButtonEventData>()) {
                    result += std::string(type == InputEventType::GAMEPAD_BUTTON_PRESSED
                                              ? "GamepadPress: "
                                              : "GamepadRelease: ") +
                        gamepadButtonToString(gamepad->button);
                }
                break;
            }
            case InputEventType::MOUSE_MOVED: {
                if (auto* motion = getEventData<MouseMotionEventData>()) {
                    result += "MouseMove: (" + std::to_string(motion->position.x) + ", " +
                        std::to_string(motion->position.y) + ")";
                }
                break;
            }
            case InputEventType::GAMEPAD_AXIS_MOVED: {
                if (auto* axis = getEventData<GamepadAxisEventData>()) {
                    result += "GamepadAxis: " + std::to_string(static_cast<int>(axis->axis)) +
                        " = " + std::to_string(axis->value);
                }
                break;
            }
            default:
                result += "Type=" + std::to_string(static_cast<int>(type));
                break;
            }

            result += ", Device=" + std::to_string(deviceId);
            result += ", Frame=" + std::to_string(frameNumber);
            if (consumed) result += ", CONSUMED";
            result += "]";

            return result;
        }
    };

    // ============================================================================
    // Event Pool Allocator Integration
    // ============================================================================

    /**
     * @brief Event pool for zero-allocation event handling
     */
    class InputEventPool {
    public:
        explicit InputEventPool(memory::PoolAllocator* pool) noexcept
            : allocator_(pool) {
        }

        /**
         * @brief Allocate new event from pool
         */
        [[nodiscard]] InputEvent* allocate() const noexcept {
            if (!allocator_) return nullptr;

            void* memory = allocator_->allocate(sizeof(InputEvent), alignof(InputEvent));
            if (!memory) return nullptr;

            return new(memory) InputEvent();
        }

        /**
         * @brief Return event to pool
         */
        void deallocate(InputEvent* event) const noexcept {
            if (!event || !allocator_) return;

            event->~InputEvent();
            allocator_->deallocate(event);
        }

        /**
         * @brief Create event in pool
         */
        template <typename T>
        InputEvent* create(const InputEventType type, const DeviceID device, const T& data) noexcept {
            InputEvent* event = allocate();
            if (!event) return nullptr;

            event->type = type;
            event->deviceId = device;
            event->data = data;
            event->timestamp = InputTimestamp::clock::now();

            return event;
        }

    private:
        memory::PoolAllocator* allocator_;
    };
} // namespace engine::input
