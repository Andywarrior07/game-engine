/**
 * @file ConfigLoader.h
 * @brief Production-ready configuration loading utilities with critical fixes
 * @author Andrés Guerrero
 * @date 14-09-2024
 *
 * Fixed critical issues:
 * - Thread-safe error handling with error queue
 * - Added extractFromActionMap methods
 * - Path validation in constructors
 * - Improved thread safety patterns
 */

#pragma once

#include "../core/InputTypes.h"
#include "../mapping/ActionMap.h"
#include "../mapping/ContextStack.h"

#include <string>
#include <vector>
#include <functional>
#include <optional>
#include <unordered_map>
#include <filesystem>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <chrono>
#include <future>
#include <queue>
#include <thread>

namespace engine::input::utils {
    /**
     * @brief Configuration file format enumeration
     */
    enum class ConfigFormat : std::uint8_t {
        JSON,
        XML,
        INI,
        BINARY,
        AUTO_DETECT
    };

    /**
     * @brief Configuration type hierarchy for path resolution
     */
    enum class ConfigType : std::uint8_t {
        ENGINE_CORE,
        ENGINE_PRESET,
        USER_PROFILE,
        GAME_CONFIG
    };

    /**
     * @brief Action binding configuration with essential metadata
     */
    struct ActionBindingConfig {
        ActionID actionId;
        std::string actionName;
        ActionType actionType;
        std::vector<InputBinding> bindings;
        std::string description;
        std::string category;
        bool allowConflicts = false;

        ActionBindingConfig() noexcept
            : actionId(INVALID_ACTION_ID)
            , actionType(ActionType::BUTTON) {}
    };

    /**
     * @brief Context configuration
     */
    struct ContextConfig {
        std::string name;
        ContextPriority priority;
        std::vector<ActionBindingConfig> actions;
        bool consumeInput = true;
        bool allowFallthrough = false;
        std::string parentContext;

        ContextConfig() noexcept
            : priority(ContextPriority::NORMAL) {}
    };

    /**
     * @brief Complete input configuration with essential data
     */
    struct InputConfigData {
        // Version and metadata
        std::string version = "1.0.0";
        std::string engineVersion = "1.0.0";
        std::chrono::system_clock::time_point modifiedDate;

        // Device settings
        struct DeviceSettings {
            bool enableKeyboard = true;
            bool enableMouse = true;
            bool enableGamepad = true;
            bool enableTouch = true;

            float mouseSensitivity = 1.0f;
            float gamepadDeadzone = 0.15f;
            float triggerThreshold = 0.1f;
            bool invertMouseY = false;
            bool rawMouseInput = true;

            // Enhanced device settings
            std::uint32_t mousePollingRate = 1000;
            bool mouseAcceleration = false;
            float gamepadVibrationStrength = 1.0f;
            bool keyboardNKeyRollover = false;
        } devices;

        // Contexts and bindings
        std::vector<ContextConfig> contexts;
        std::string defaultContext = "Default";
        std::vector<ActionBindingConfig> globalActions;

        // Profile information
        std::string profileName = "Default";
        std::string author;

        // Validation
        bool validated = false;

        InputConfigData() noexcept {
            modifiedDate = std::chrono::system_clock::now();
        }
    };

    /**
     * @brief Thread-safe error information
     */
    struct ErrorInfo {
        std::string message;
        std::chrono::steady_clock::time_point timestamp;
        std::thread::id threadId;

        explicit ErrorInfo(std::string msg)
            : message(std::move(msg))
            , timestamp(std::chrono::steady_clock::now())
            , threadId(std::this_thread::get_id()) {}
    };

    /**
     * @brief Simple cache entry for configurations
     */
    struct ConfigCacheEntry {
        std::shared_ptr<InputConfigData> config;
        std::filesystem::file_time_type fileTime;
        std::chrono::steady_clock::time_point accessTime;
        // std::atomic<std::uint32_t> accessCount{0};

        ConfigCacheEntry() = default;

        ConfigCacheEntry(std::shared_ptr<InputConfigData> cfg,
                         const std::filesystem::file_time_type ft) noexcept
            : config(std::move(cfg))
            , fileTime(ft)
            , accessTime(std::chrono::steady_clock::now()) {}
    };

    /**
     * @brief Production-ready configuration loader with all critical fixes
     */
    class ConfigLoader {
    public:
        using ValidationCallback = std::function<bool(const InputConfigData& config, std::string& error)>;

        /**
         * @brief Cache configuration settings
         */
        struct CacheSettings {
            std::size_t maxEntries = 16;
            std::chrono::minutes entryTTL{15};
            bool enableLRU = true;
        };

        /**
         * @brief Performance settings
         */
        struct PerformanceSettings {
            bool enableAsyncLoading = true;
            std::uint32_t maxConcurrentLoads = 2;
            std::chrono::milliseconds ioTimeout{3000};
        };

        /**
         * @brief Constructor with path validation
         */
        explicit ConfigLoader(
            const std::filesystem::path& engineDataPath,
            const std::filesystem::path& userConfigPath = {});

        /**
         * @brief Advanced constructor with detailed settings
         */
        ConfigLoader(
            const std::filesystem::path& engineDataPath,
            const std::filesystem::path& userConfigPath,
            CacheSettings cacheSettings,
            PerformanceSettings perfSettings);

        ~ConfigLoader();

        // Disable copy, enable move
        ConfigLoader(const ConfigLoader&) = delete;
        ConfigLoader& operator=(const ConfigLoader&) = delete;
        ConfigLoader(ConfigLoader&&) noexcept = default;
        ConfigLoader& operator=(ConfigLoader&&) noexcept = default;

        // ============================================================================
        // Core Loading Methods
        // ============================================================================

        [[nodiscard]] std::optional<InputConfigData> loadFromFile(
            const std::string& filepath,
            ConfigFormat format = ConfigFormat::AUTO_DETECT,
            bool useCache = true);

        [[nodiscard]] std::optional<InputConfigData> loadConfiguration(
            const std::string& configName,
            ConfigType type,
            ConfigFormat format = ConfigFormat::JSON,
            bool useCache = true);

        [[nodiscard]] std::future<std::optional<InputConfigData>> loadConfigurationAsync(
            const std::string& configName,
            ConfigType type,
            ConfigFormat format = ConfigFormat::JSON);

        [[nodiscard]] std::optional<InputConfigData> resolveConfigurationHierarchy(
            const std::optional<std::string>& presetName = std::nullopt,
            const std::optional<std::string>& userProfile = std::nullopt);

        // ============================================================================
        // ActionMap Integration Methods - CRITICAL ADDITIONS
        // ============================================================================

        /**
         * @brief Extract current configuration from ActionMap and ContextStack
         */
        [[nodiscard]] static InputConfigData extractCurrentConfiguration(
            const ActionMap* actionMap,
            const mapping::ContextStack* contextStack = nullptr);

        /**
         * @brief Extract configuration data from an ActionMap
         * @param actionMap Source action map to extract from
         * @return Configuration data representing the action map state
         */
        [[nodiscard]] InputConfigData extractFromActionMap(const ActionMap* actionMap) const;

        /**
         * @brief Extract configuration data from an ActionMap (overload for non-const)
         * @param actionMap Source action map to extract from
         * @return Configuration data representing the action map state
         */
        [[nodiscard]] InputConfigData extractFromActionMap(ActionMap* actionMap) const;

        /**
         * @brief Apply configuration to ActionMap
         */
        bool applyToActionMap(const InputConfigData& config, ActionMap* actionMap) const;

        /**
         * @brief Apply configuration to ContextStack
         */
        bool applyToContextStack(const InputConfigData& config, mapping::ContextStack* contextStack) const;

        // ============================================================================
        // Saving Methods
        // ============================================================================

        bool saveToFile(
            const InputConfigData& config,
            const std::string& filepath,
            ConfigFormat format = ConfigFormat::JSON,
            bool createBackup = true) const;

        bool saveUserProfile(
            const InputConfigData& config,
            const std::string& profileName = "default",
            bool createBackup = true) const;

        // ============================================================================
        // Preset Management
        // ============================================================================

        [[nodiscard]] static InputConfigData getDefaultConfiguration(const std::string& gameType);
        [[nodiscard]] static InputConfigData createFPSPreset();
        [[nodiscard]] static InputConfigData createRPGPreset();

        // ============================================================================
        // Configuration Discovery
        // ============================================================================

        [[nodiscard]] std::vector<std::string> getAvailablePresets() const;
        [[nodiscard]] std::vector<std::string> getAvailableUserProfiles() const;
        [[nodiscard]] bool configurationExists(
            const std::string& configName,
            ConfigType type,
            ConfigFormat format = ConfigFormat::JSON) const;

        // ============================================================================
        // Validation and Integrity
        // ============================================================================

        [[nodiscard]] std::vector<std::string> validateConfiguration(const InputConfigData& config) const;
        [[nodiscard]] static std::vector<std::string> checkBindingConflicts(const InputConfigData& config);
        void registerValidator(ValidationCallback validator);

        // ============================================================================
        // Cache Management
        // ============================================================================

        void clearCache() const;

        struct CacheStats {
            std::size_t totalEntries;
            std::size_t hitCount;
            std::size_t missCount;
            double hitRatio;
        };

        [[nodiscard]] CacheStats getCacheStatistics() const;

        // ============================================================================
        // Hot Reload Support
        // ============================================================================

        void enableConfigurationHotReload(std::function<void()> reloadCallback);
        [[nodiscard]] bool hasConfigurationChanged() const;

        // ============================================================================
        // Thread-Safe Error Handling - CRITICAL FIX
        // ============================================================================

        /**
         * @brief Get all errors from all threads
         * @param maxErrors Maximum number of errors to retrieve
         * @return Vector of error information
         */
        [[nodiscard]] std::vector<ErrorInfo> getErrors(std::size_t maxErrors = 100) const;

        /**
         * @brief Get last error from current thread
         * @return Error message or empty if no error
         */
        [[nodiscard]] std::string getLastError() const;

        /**
         * @brief Clear all errors
         */
        void clearErrors();

        /**
         * @brief Check if there are any errors
         */
        [[nodiscard]] bool hasErrors() const;

    private:
        // ============================================================================
        // Internal Implementation
        // ============================================================================

        // Path management - validated in constructor
        std::filesystem::path engineDataPath_;
        std::filesystem::path userConfigPath_;
        bool pathsValidated_ = false;

        // Configuration settings
        CacheSettings cacheSettings_;
        PerformanceSettings perfSettings_;

        // Cache management with better thread safety
        mutable std::unordered_map<std::string, ConfigCacheEntry> configCache_;
        mutable std::shared_mutex cacheMutex_;  // Changed to shared_mutex for read-write locking
        mutable std::atomic<std::size_t> cacheHitCount_{0};
        mutable std::atomic<std::size_t> cacheMissCount_{0};

        // Validation
        std::vector<ValidationCallback> validators_;
        mutable std::shared_mutex validatorsMutex_;  // Changed to shared_mutex

        // Thread-safe error handling - CRITICAL FIX
        mutable std::queue<ErrorInfo> errorQueue_;
        mutable std::mutex errorMutex_;
        static constexpr std::size_t MAX_ERROR_QUEUE_SIZE = 1000;

        // Thread management
        mutable std::atomic<std::uint32_t> activeLoads_{0};

        // Hot reload
        std::function<void()> reloadCallback_;
        mutable std::atomic<bool> configChanged_{false};

        // ============================================================================
        // Private Helper Methods
        // ============================================================================

        /**
         * @brief Validate and create paths if needed
         */
        bool validatePaths();

        /**
         * @brief Thread-safe error recording
         */
        void recordError(const std::string& message) const;

        [[nodiscard]] std::filesystem::path resolvePath(
            const std::string& configName,
            ConfigType type,
            ConfigFormat format) const;

        [[nodiscard]] std::string generateCacheKey(
            const std::string& configName,
            ConfigType type,
            ConfigFormat format) const;

        [[nodiscard]] bool isCacheEntryValid(
            const ConfigCacheEntry& entry,
            const std::filesystem::path& filePath) const;

        void addToCache(
            const std::string& key,
            const std::shared_ptr<InputConfigData>& config,
            const std::filesystem::path& filePath) const;

        void evictOldEntries() const;

        // Format-specific parsers
        [[nodiscard]] std::optional<InputConfigData> parseJSON(const std::string& data) const;
        [[nodiscard]] std::optional<InputConfigData> parseXML(const std::string& data);
        [[nodiscard]] std::optional<InputConfigData> parseINI(const std::string& data);
        [[nodiscard]] std::optional<InputConfigData> parseBinary(const std::string& data);

        // Format-specific serializers
        [[nodiscard]] static std::string serializeJSON(const InputConfigData& config);
        [[nodiscard]] std::string serializeXML(const InputConfigData& config) const;
        [[nodiscard]] std::string serializeINI(const InputConfigData& config) const;
        [[nodiscard]] static std::string serializeBinary(const InputConfigData& config);

        // Utility methods
        [[nodiscard]] static ConfigFormat detectFormat(const std::filesystem::path& filepath);
        [[nodiscard]] static std::string getFileExtension(ConfigFormat format);
        [[nodiscard]] static std::optional<std::string> readFileContents(const std::filesystem::path& path);
        static bool writeFileContents(const std::filesystem::path& path, const std::string& content, bool createBackup);
        static bool atomicWriteFile(const std::filesystem::path& path, const std::string& content);
        [[nodiscard]] static std::filesystem::path getUserConfigDirectory(const std::string& appName = "GameEngine");
    };

    // ============================================================================
    // Configuration Builder
    // ============================================================================

    class ConfigBuilder {
    public:
        ConfigBuilder() = default;

        ConfigBuilder& withMetadata(const std::string& name, const std::string& author = "");
        ConfigBuilder& withDeviceSettings(bool keyboard = true, bool mouse = true, bool gamepad = true);
        ConfigBuilder& withMouseSettings(float sensitivity = 1.0f, bool invertY = false);
        ConfigBuilder& withGamepadSettings(float deadzone = 0.15f, float triggerThreshold = 0.1f);
        ConfigBuilder& addContext(const ContextConfig& context);
        ConfigBuilder& addAction(const ActionBindingConfig& action);
        ConfigBuilder& setDefaultContext(const std::string& context);

        [[nodiscard]] InputConfigData build();
        [[nodiscard]] std::vector<std::string> validate() const;
        ConfigBuilder& reset();

    private:
        InputConfigData config_;
        void updateMetadata();
    };

    // ============================================================================
    // Utility Functions
    // ============================================================================

    [[nodiscard]] constexpr const char* toString(const ConfigFormat format) noexcept {
        switch (format) {
        case ConfigFormat::JSON: return "JSON";
        case ConfigFormat::XML: return "XML";
        case ConfigFormat::INI: return "INI";
        case ConfigFormat::BINARY: return "BINARY";
        case ConfigFormat::AUTO_DETECT: return "AUTO_DETECT";
        default: return "UNKNOWN";
        }
    }

    [[nodiscard]] constexpr const char* toString(const ConfigType type) noexcept {
        switch (type) {
        case ConfigType::ENGINE_CORE: return "ENGINE_CORE";
        case ConfigType::ENGINE_PRESET: return "ENGINE_PRESET";
        case ConfigType::USER_PROFILE: return "USER_PROFILE";
        case ConfigType::GAME_CONFIG: return "GAME_CONFIG";
        default: return "UNKNOWN";
        }
    }

    [[nodiscard]] std::optional<ConfigFormat> parseConfigFormat(const std::string& str);
    [[nodiscard]] std::optional<ConfigType> parseConfigType(const std::string& str);
    [[nodiscard]] std::filesystem::path getUserConfigDirectory(const std::string& applicationName = "GameEngine");
    [[nodiscard]] bool isVersionCompatible(const std::string& configVersion, const std::string& engineVersion);

    // ============================================================================
    // Constants
    // ============================================================================

    namespace defaults {
        constexpr auto INPUT_CONFIG_FILE = "input_config.json";
        constexpr auto DEVICE_DEFAULTS_FILE = "device_defaults.json";
        constexpr auto ENGINE_BINDINGS_FILE = "engine_bindings.json";

        constexpr auto CORE_CONFIG_DIR = "core";
        constexpr auto PRESETS_DIR = "presets";
        constexpr auto USER_PROFILES_DIR = "profiles";

        constexpr float DEFAULT_MOUSE_SENSITIVITY = 1.0f;
        constexpr float DEFAULT_GAMEPAD_DEADZONE = 0.15f;
        constexpr float DEFAULT_TRIGGER_THRESHOLD = 0.1f;

        constexpr auto CONFIG_FORMAT_VERSION = "1.0.0";
    }
} // namespace engine::input::utils