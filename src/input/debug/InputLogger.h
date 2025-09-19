/**
 * @file InputLogger.h
 * @brief Input system debug logging utilities
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Provides comprehensive logging for input events, snapshots, and system state.
 * Supports multiple output formats and filtering options.
 */

#pragma once

#include "../core/InputEvent.h"

#include <string>
#include <fstream>
#include <mutex>
#include <memory>
#include <vector>
#include <chrono>
#include <atomic>

namespace engine::input::debug {
    /**
     * @brief Log level for input logging
     */
    enum class LogLevel : std::uint8_t {
        NONE = 0,
        ERROR = 1,
        WARNING = 2,
        INFO = 3,
        DEBUG = 4,
        VERBOSE = 5
    };

    /**
     * @brief Log entry type
     */
    enum class LogEntryType : std::uint8_t {
        EVENT,
        SNAPSHOT,
        DEVICE,
        ACTION,
        CONTEXT,
        PERFORMANCE,
        ERROR,
        SYSTEM
    };

    /**
     * @brief Log output format
     */
    enum class LogFormat : std::uint8_t {
        TEXT,
        JSON,
        CSV,
        BINARY,
        CUSTOM
    };

    /**
     * @brief Log entry
     */
    struct LogEntry {
        LogLevel level;
        LogEntryType type;
        std::chrono::steady_clock::time_point timestamp;
        std::uint64_t frameNumber;
        std::string message;
        std::string category;
        std::string details;

        LogEntry() noexcept
            : level(LogLevel::INFO)
              , type(LogEntryType::SYSTEM)
              , timestamp(std::chrono::steady_clock::now())
              , frameNumber(0) {
        }
    };

    /**
     * @brief Logger configuration
     */
    struct LoggerConfig {
        LogLevel minLevel = LogLevel::INFO;
        LogFormat format = LogFormat::TEXT;
        bool logToFile = true;
        bool logToConsole = true;
        bool logToMemory = false;
        std::string logFilePath = "input_debug.log";
        std::size_t maxMemoryEntries = 10000;
        std::size_t maxFileSize = 10 * 1024 * 1024; // 10MB
        bool rotateFiles = true;
        std::uint32_t maxRotatedFiles = 5;
        bool includeTimestamp = true;
        bool includeFrameNumber = true;
        bool flushImmediately = false;

        LoggerConfig() = default;
    };

    /**
     * @brief Input system debug logger
     *
     * Thread-safe logger for debugging input system behavior.
     */
    class InputLogger {
    public:
        using LogCallback = std::function<void(const LogEntry&)>;
        using FormatFunction = std::function<std::string(const LogEntry&)>;

        /**
         * @brief Constructor
         */
        explicit InputLogger() noexcept;

        /**
         * @brief Destructor
         */
        ~InputLogger();

        // Disable copy, enable move
        InputLogger(const InputLogger&) = delete;
        InputLogger& operator=(const InputLogger&) = delete;
        InputLogger(InputLogger&&) noexcept = default;
        InputLogger& operator=(InputLogger&&) noexcept = default;

        // ============================================================================
        // Initialization
        // ============================================================================

        /**
         * @brief Initialize the logger
         * @param config Logger configuration
         * @return True if successful
         */
        bool initialize(const LoggerConfig& config = {});

        /**
         * @brief Shutdown the logger
         */
        void shutdown();

        /**
         * @brief Set configuration
         * @param config New configuration
         */
        void setConfig(const LoggerConfig& config);

        /**
         * @brief Get current configuration
         */
        [[nodiscard]] const LoggerConfig& getConfig() const noexcept {
            return config_;
        }

        // ============================================================================
        // Event Logging
        // ============================================================================

        /**
         * @brief Log input event
         * @param event Event to log
         * @param level Log level
         */
        void logEvent(const InputEvent& event, LogLevel level = LogLevel::DEBUG);

        /**
         * @brief Log multiple events
         * @param events Events to log
         * @param level Log level
         */
        void logEvents(const std::vector<InputEvent>& events, LogLevel level = LogLevel::DEBUG);

        // ============================================================================
        // Snapshot Logging
        // ============================================================================

        /**
         * @brief Log input snapshot
         * @param snapshot Snapshot to log
         * @param level Log level
         */
        void logSnapshot(const InputSnapshot& snapshot, LogLevel level = LogLevel::VERBOSE);

        /**
         * @brief Log snapshot diff
         * @param current Current snapshot
         * @param previous Previous snapshot
         * @param level Log level
         */
        void logSnapshotDiff(const InputSnapshot& current,
                             const InputSnapshot& previous,
                             LogLevel level = LogLevel::DEBUG);

        // ============================================================================
        // General Logging
        // ============================================================================

        /**
         * @brief Log message
         * @param level Log level
         * @param type Entry type
         * @param message Log message
         * @param category Optional category
         * @param details Optional details
         */
        void log(LogLevel level,
                 LogEntryType type,
                 const std::string& message,
                 const std::string& category = "",
                 const std::string& details = "");

        /**
         * @brief Log error
         */
        void error(const std::string& message, const std::string& details = "") {
            log(LogLevel::ERROR, LogEntryType::ERROR, message, "Error", details);
        }

        /**
         * @brief Log warning
         */
        void warning(const std::string& message, const std::string& details = "") {
            log(LogLevel::WARNING, LogEntryType::SYSTEM, message, "Warning", details);
        }

        /**
         * @brief Log info
         */
        void info(const std::string& message, const std::string& details = "") {
            log(LogLevel::INFO, LogEntryType::SYSTEM, message, "Info", details);
        }

        /**
         * @brief Log debug
         */
        void debug(const std::string& message, const std::string& details = "") {
            log(LogLevel::DEBUG, LogEntryType::SYSTEM, message, "Debug", details);
        }

        /**
         * @brief Log verbose
         */
        void verbose(const std::string& message, const std::string& details = "") {
            log(LogLevel::VERBOSE, LogEntryType::SYSTEM, message, "Verbose", details);
        }

        // ============================================================================
        // Performance Logging
        // ============================================================================

        /**
         * @brief Log performance metrics
         * @param metric Metric name
         * @param value Metric value
         * @param unit Unit of measurement
         */
        void logPerformance(const std::string& metric, float value, const std::string& unit = "ms");

        /**
         * @brief Start performance timer
         * @param name Timer name
         */
        void startTimer(const std::string& name);

        /**
         * @brief End performance timer and log result
         * @param name Timer name
         */
        void endTimer(const std::string& name);

        // ============================================================================
        // Output Management
        // ============================================================================

        /**
         * @brief Flush all pending log entries
         */
        void flush();

        /**
         * @brief Clear memory log
         */
        void clearMemoryLog();

        /**
         * @brief Get memory log entries
         * @param maxEntries Maximum entries to retrieve
         * @return Log entries
         */
        [[nodiscard]] std::vector<LogEntry> getMemoryLog(std::size_t maxEntries = 0) const;

        /**
         * @brief Export log to file
         * @param filepath Output file path
         * @param format Export format
         * @return True if successful
         */
        bool exportLog(const std::string& filepath, LogFormat format = LogFormat::TEXT);

        /**
         * @brief Register custom log callback
         * @param callback Callback function
         */
        void registerCallback(LogCallback callback);

        /**
         * @brief Set custom format function
         * @param format Format function
         */
        void setCustomFormat(FormatFunction format);

        // ============================================================================
        // Filtering
        // ============================================================================

        /**
         * @brief Set minimum log level
         * @param level Minimum level to log
         */
        void setLogLevel(const LogLevel level) noexcept {
            config_.minLevel = level;
        }

        /**
         * @brief Enable/disable category
         * @param category Category name
         * @param enabled Enable state
         */
        void setCategoryEnabled(const std::string& category, bool enabled);

        /**
         * @brief Check if category is enabled
         * @param category Category name
         * @return True if enabled
         */
        [[nodiscard]] bool isCategoryEnabled(const std::string& category) const;

        // ============================================================================
        // Statistics
        // ============================================================================

        struct Statistics {
            std::atomic<std::uint64_t> totalEntries{0};
            std::atomic<std::uint64_t> droppedEntries{0};
            std::atomic<std::uint64_t> fileWrites{0};
            std::atomic<std::uint64_t> consoleWrites{0};
            std::atomic<std::size_t> currentFileSize{0};

            void reset() noexcept {
                totalEntries = 0;
                droppedEntries = 0;
                fileWrites = 0;
                consoleWrites = 0;
                currentFileSize = 0;
            }
        };

        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        void resetStatistics() const noexcept {
            stats_.reset();
        }

        // ============================================================================
        // Frame Management
        // ============================================================================

        /**
         * @brief Set current frame number
         * @param frameNumber Frame number
         */
        void setFrameNumber(const std::uint64_t frameNumber) noexcept {
            currentFrame_.store(frameNumber, std::memory_order_release);
        }

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        LoggerConfig config_;
        std::atomic<bool> initialized_{false};

        // Output streams
        std::unique_ptr<std::ofstream> fileStream_;
        mutable std::mutex fileMutex_;

        // Memory log
        std::vector<LogEntry> memoryLog_;
        mutable std::mutex memoryMutex_;

        // Callbacks
        std::vector<LogCallback> callbacks_;
        mutable std::mutex callbackMutex_;

        // Custom format
        FormatFunction customFormat_;

        // Category filters
        std::unordered_map<std::string, bool> categoryFilters_;
        mutable std::mutex filterMutex_;

        // Performance timers
        std::unordered_map<std::string, std::chrono::steady_clock::time_point> timers_;
        mutable std::mutex timerMutex_;

        // Statistics
        mutable Statistics stats_;

        // Frame tracking
        std::atomic<std::uint64_t> currentFrame_{0};

        // File rotation
        std::uint32_t currentRotation_ = 0;

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Write log entry
         * @param entry Entry to write
         */
        void writeEntry(const LogEntry& entry);

        /**
         * @brief Format log entry
         * @param entry Entry to format
         * @return Formatted string
         */
        [[nodiscard]] std::string formatEntry(const LogEntry& entry) const;

        /**
         * @brief Format as text
         */
        [[nodiscard]] std::string formatText(const LogEntry& entry) const;

        /**
         * @brief Format as JSON
         */
        [[nodiscard]] std::string formatJSON(const LogEntry& entry) const;

        /**
         * @brief Format as CSV
         */
        [[nodiscard]] std::string formatCSV(const LogEntry& entry) const;

        /**
         * @brief Format event
         */
        [[nodiscard]] std::string formatEvent(const InputEvent& event) const;

        /**
         * @brief Format snapshot
         */
        [[nodiscard]] std::string formatSnapshot(const InputSnapshot& snapshot) const;

        /**
         * @brief Rotate log file
         */
        void rotateLogFile();

        /**
         * @brief Check file size and rotate if needed
         */
        void checkFileRotation();

        /**
         * @brief Open log file
         */
        bool openLogFile();

        /**
         * @brief Close log file
         */
        void closeLogFile();

        /**
         * @brief Should log entry
         * @param entry Entry to check
         * @return True if should be logged
         */
        [[nodiscard]] bool shouldLog(const LogEntry& entry) const;
    };
} // namespace engine::input::debug
