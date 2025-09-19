/**
 * @file InputLogger.cpp
 * @brief Input system debug logging utilities implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "InputLogger.h"

#include "../processing/SnapshotManager.h"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace engine::input::debug {
    InputLogger::InputLogger() noexcept = default;

    InputLogger::~InputLogger() {
        shutdown();
    }

    // ============================================================================
    // Initialization
    // ============================================================================

    bool InputLogger::initialize(const LoggerConfig& config) {
        if (initialized_.load(std::memory_order_acquire)) {
            return false;
        }

        config_ = config;

        // Open log file if needed
        if (config_.logToFile && !openLogFile()) {
            return false;
        }

        // Reserve memory log space
        if (config_.logToMemory) {
            std::lock_guard lock(memoryMutex_);
            memoryLog_.reserve(config_.maxMemoryEntries);
        }

        initialized_.store(true, std::memory_order_release);
        return true;
    }

    void InputLogger::shutdown() {
        if (!initialized_.load(std::memory_order_acquire)) {
            return;
        }

        // Flush any remaining entries
        flush();

        // Close file stream
        closeLogFile();

        // Clear memory log
        clearMemoryLog();

        // Clear callbacks
        {
            std::lock_guard lock(callbackMutex_);
            callbacks_.clear();
        }

        initialized_.store(false, std::memory_order_release);
    }

    void InputLogger::setConfig(const LoggerConfig& config) {
        config_ = config;

        // Reopen file if path changed
        if (config_.logToFile) {
            closeLogFile();
            openLogFile();
        }
    }

    // ============================================================================
    // Event Logging
    // ============================================================================

    void InputLogger::logEvent(const InputEvent& event, const LogLevel level) {
        if (!initialized_.load(std::memory_order_acquire) || level < config_.minLevel) {
            return;
        }

        LogEntry entry;
        entry.level = level;
        entry.type = LogEntryType::EVENT;
        entry.timestamp = std::chrono::steady_clock::now();
        entry.frameNumber = currentFrame_.load(std::memory_order_acquire);
        entry.category = "Event";
        entry.message = event.toString();
        entry.details = formatEvent(event);

        writeEntry(entry);
    }

    void InputLogger::logEvents(const std::vector<InputEvent>& events, const LogLevel level) {
        if (!initialized_.load(std::memory_order_acquire) || level < config_.minLevel) {
            return;
        }

        LogEntry entry;
        entry.level = level;
        entry.type = LogEntryType::EVENT;
        entry.timestamp = std::chrono::steady_clock::now();
        entry.frameNumber = currentFrame_.load(std::memory_order_acquire);
        entry.category = "Events";
        entry.message = "Batch of " + std::to_string(events.size()) + " events";

        std::stringstream details;
        for (const auto& event : events) {
            details << "  " << event.toString() << "\n";
        }
        entry.details = details.str();

        writeEntry(entry);
    }

    // ============================================================================
    // Snapshot Logging
    // ============================================================================

    void InputLogger::logSnapshot(const InputSnapshot& snapshot, const LogLevel level) {
        if (!initialized_.load(std::memory_order_acquire) || level < config_.minLevel) {
            return;
        }

        LogEntry entry;
        entry.level = level;
        entry.type = LogEntryType::SNAPSHOT;
        entry.timestamp = std::chrono::steady_clock::now();
        entry.frameNumber = snapshot.frameNumber;
        entry.category = "Snapshot";
        entry.message = "Input snapshot for frame " + std::to_string(snapshot.frameNumber);
        entry.details = formatSnapshot(snapshot);

        writeEntry(entry);
    }

    void InputLogger::logSnapshotDiff(const InputSnapshot& current, const InputSnapshot& previous,
                                      const LogLevel level) {
        if (!initialized_.load(std::memory_order_acquire) || level < config_.minLevel) {
            return;
        }

        LogEntry entry;
        entry.level = level;
        entry.type = LogEntryType::SNAPSHOT;
        entry.timestamp = std::chrono::steady_clock::now();
        entry.frameNumber = current.frameNumber;
        entry.category = "SnapshotDiff";
        entry.message = "Snapshot diff between frames " + std::to_string(previous.frameNumber) +
            " and " + std::to_string(current.frameNumber);

        std::stringstream details;

        // Compare keyboard state
        if (current.keyboard.pressed != previous.keyboard.pressed) {
            details << "Keyboard changes:\n";

            for (std::uint16_t i = 0; i < static_cast<std::uint16_t>(KeyCode::KEY_COUNT); ++i) {
                const KeyCode key = static_cast<KeyCode>(i);
                const bool wasPrevious = previous.keyboard.isKeyPressed(key);
                const bool isCurrent = current.keyboard.isKeyPressed(key);

                if (wasPrevious != isCurrent) {
                    details << "  " << keyCodeToString(key) << ": "
                        << (isCurrent ? "PRESSED" : "RELEASED") << "\n";
                }
            }
        }

        // Compare mouse state
        if (current.mouse.buttons != previous.mouse.buttons ||
            current.mouse.position != previous.mouse.position) {
            details << "Mouse changes:\n";

            if (current.mouse.position != previous.mouse.position) {
                details << "  Position: (" << current.mouse.position.x << ", " << current.mouse.position.y << ")\n";
            }

            for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(MouseButton::BUTTON_COUNT); ++i) {
                const MouseButton button = static_cast<MouseButton>(i);
                const bool wasPrevious = previous.mouse.isButtonPressed(button);
                const bool isCurrent = current.mouse.isButtonPressed(button);

                if (wasPrevious != isCurrent) {
                    details << "  " << mouseButtonToString(button) << ": "
                        << (isCurrent ? "PRESSED" : "RELEASED") << "\n";
                }
            }
        }

        // Compare gamepad states
        for (PlayerID player = 0; player < MAX_PLAYERS; ++player) {
            const auto* currentGamepad = current.getGamepad(player);
            const auto* previousGamepad = previous.getGamepad(player);

            if (currentGamepad && previousGamepad) {
                if (currentGamepad->buttons != previousGamepad->buttons ||
                    currentGamepad->leftStick != previousGamepad->leftStick ||
                    currentGamepad->rightStick != previousGamepad->rightStick ||
                    currentGamepad->leftTrigger != previousGamepad->leftTrigger ||
                    currentGamepad->rightTrigger != previousGamepad->rightTrigger) {
                    details << "Gamepad " << static_cast<int>(player) << " changes:\n";
                    // Add detailed gamepad diff here
                }
            }
        }

        entry.details = details.str();
        writeEntry(entry);
    }

    // ============================================================================
    // General Logging
    // ============================================================================

    void InputLogger::log(const LogLevel level, const LogEntryType type, const std::string& message,
                          const std::string& category, const std::string& details) {
        if (!initialized_.load(std::memory_order_acquire) || level < config_.minLevel) {
            return;
        }

        LogEntry entry;
        entry.level = level;
        entry.type = type;
        entry.timestamp = std::chrono::steady_clock::now();
        entry.frameNumber = currentFrame_.load(std::memory_order_acquire);
        entry.message = message;
        entry.category = category.empty() ? "General" : category;
        entry.details = details;

        writeEntry(entry);
    }

    // ============================================================================
    // Performance Logging
    // ============================================================================

    void InputLogger::logPerformance(const std::string& metric, const float value, const std::string& unit) {
        LogEntry entry;
        entry.level = LogLevel::INFO;
        entry.type = LogEntryType::PERFORMANCE;
        entry.timestamp = std::chrono::steady_clock::now();
        entry.frameNumber = currentFrame_.load(std::memory_order_acquire);
        entry.category = "Performance";
        entry.message = metric + ": " + std::to_string(value) + " " + unit;

        writeEntry(entry);
    }

    void InputLogger::startTimer(const std::string& name) {
        std::lock_guard lock(timerMutex_);
        timers_[name] = std::chrono::steady_clock::now();
    }

    void InputLogger::endTimer(const std::string& name) {
        const auto endTime = std::chrono::steady_clock::now();

        std::lock_guard lock(timerMutex_);
        const auto it = timers_.find(name);
        if (it != timers_.end()) {
            const auto duration = std::chrono::duration<float, std::milli>(endTime - it->second).count();
            timers_.erase(it);

            // Log the timing result
            logPerformance(name, duration, "ms");
        }
    }

    // ============================================================================
    // Output Management
    // ============================================================================

    void InputLogger::flush() {
        if (fileStream_) {
            std::lock_guard lock(fileMutex_);
            fileStream_->flush();
        }
    }

    void InputLogger::clearMemoryLog() {
        std::lock_guard lock(memoryMutex_);
        memoryLog_.clear();
        memoryLog_.shrink_to_fit();
    }

    std::vector<LogEntry> InputLogger::getMemoryLog(const std::size_t maxEntries) const {
        std::lock_guard lock(memoryMutex_);

        if (maxEntries == 0 || maxEntries >= memoryLog_.size()) {
            return memoryLog_;
        }

        // Return the most recent entries
        const std::size_t startIndex = memoryLog_.size() - maxEntries;
        return std::vector<LogEntry>(memoryLog_.begin() + startIndex, memoryLog_.end());
    }

    bool InputLogger::exportLog(const std::string& filepath, const LogFormat format) {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            return false;
        }

        std::lock_guard lock(memoryMutex_);

        // Write header based on format
        switch (format) {
        case LogFormat::JSON:
            file << "{\n  \"entries\": [\n";
            break;
        case LogFormat::CSV:
            file << "timestamp,level,type,category,message,details\n";
            break;
        case LogFormat::TEXT:
        default:
            break;
        }

        // Write entries
        for (std::size_t i = 0; i < memoryLog_.size(); ++i) {
            const auto& entry = memoryLog_[i];

            switch (format) {
            case LogFormat::JSON:
                file << "    " << formatJSON(entry);
                if (i < memoryLog_.size() - 1) file << ",";
                file << "\n";
                break;
            case LogFormat::CSV:
                file << formatCSV(entry) << "\n";
                break;
            case LogFormat::TEXT:
            default:
                file << formatText(entry) << "\n";
                break;
            }
        }

        // Write footer based on format
        switch (format) {
        case LogFormat::JSON:
            file << "  ]\n}\n";
            break;
        default:
            break;
        }

        return true;
    }

    void InputLogger::registerCallback(LogCallback callback) {
        std::lock_guard lock(callbackMutex_);
        callbacks_.push_back(std::move(callback));
    }

    void InputLogger::setCustomFormat(FormatFunction format) {
        customFormat_ = std::move(format);
    }

    // ============================================================================
    // Filtering
    // ============================================================================

    void InputLogger::setCategoryEnabled(const std::string& category, const bool enabled) {
        std::lock_guard lock(filterMutex_);
        categoryFilters_[category] = enabled;
    }

    bool InputLogger::isCategoryEnabled(const std::string& category) const {
        std::lock_guard lock(filterMutex_);
        const auto it = categoryFilters_.find(category);
        return it == categoryFilters_.end() || it->second; // Default enabled
    }

    // ============================================================================
    // Private Implementation
    // ============================================================================

    void InputLogger::writeEntry(const LogEntry& entry) {
        if (!shouldLog(entry)) {
            return;
        }

        stats_.totalEntries.fetch_add(1, std::memory_order_relaxed);

        // Write to file
        if (config_.logToFile && fileStream_) {
            std::lock_guard lock(fileMutex_);

            const std::string formatted = formatEntry(entry);
            *fileStream_ << formatted << "\n";

            if (config_.flushImmediately) {
                fileStream_->flush();
            }

            stats_.fileWrites.fetch_add(1, std::memory_order_relaxed);
            stats_.currentFileSize.fetch_add(formatted.size() + 1, std::memory_order_relaxed);

            checkFileRotation();
        }

        // Write to console
        if (config_.logToConsole) {
            const std::string formatted = formatEntry(entry);

            // Use stderr for errors and warnings
            if (entry.level <= LogLevel::WARNING) {
                std::cerr << formatted << std::endl;
            }
            else {
                std::cout << formatted << std::endl;
            }

            stats_.consoleWrites.fetch_add(1, std::memory_order_relaxed);
        }

        // Store in memory
        if (config_.logToMemory) {
            std::lock_guard lock(memoryMutex_);

            if (memoryLog_.size() >= config_.maxMemoryEntries) {
                memoryLog_.erase(memoryLog_.begin()); // Remove oldest
            }

            memoryLog_.push_back(entry);
        }

        // Invoke callbacks
        {
            std::lock_guard lock(callbackMutex_);
            for (const auto& callback : callbacks_) {
                if (callback) {
                    try {
                        callback(entry);
                    }
                    catch (const std::exception&) {
                        // Ignore callback exceptions to prevent logging loops
                    }
                }
            }
        }
    }

    std::string InputLogger::formatEntry(const LogEntry& entry) const {
        if (customFormat_) {
            return customFormat_(entry);
        }

        switch (config_.format) {
        case LogFormat::JSON:
            return formatJSON(entry);
        case LogFormat::CSV:
            return formatCSV(entry);
        case LogFormat::TEXT:
        default:
            return formatText(entry);
        }
    }

    std::string InputLogger::formatText(const LogEntry& entry) const {
        std::stringstream ss;

        // Timestamp
        if (config_.includeTimestamp) {
            const auto steady_duration = entry.timestamp - std::chrono::steady_clock::now();
            const auto system_duration = std::chrono::duration_cast<std::chrono::system_clock::duration>(steady_duration);
            const auto system_time_point = std::chrono::system_clock::now() + system_duration;
            const auto time_t = std::chrono::system_clock::to_time_t(system_time_point);

            ss << std::put_time(std::localtime(&time_t), "%H:%M:%S.")
                << std::setfill('0') << std::setw(3)
                << (std::chrono::duration_cast<std::chrono::milliseconds>(
                    entry.timestamp.time_since_epoch()).count() % 1000);
        }

        // Frame number
        if (config_.includeFrameNumber) {
            ss << " [F" << entry.frameNumber << "]";
        }

        // Level
        const char* levelStr = "UNKNOWN";
        switch (entry.level) {
        case LogLevel::ERROR: levelStr = "ERROR";
            break;
        case LogLevel::WARNING: levelStr = "WARN";
            break;
        case LogLevel::INFO: levelStr = "INFO";
            break;
        case LogLevel::DEBUG: levelStr = "DEBUG";
            break;
        case LogLevel::VERBOSE: levelStr = "VERBOSE";
            break;
        case LogLevel::NONE: levelStr = "NONE";
            break;
        }
        ss << " [" << levelStr << "]";

        // Category
        if (!entry.category.empty()) {
            ss << " [" << entry.category << "]";
        }

        // Message
        ss << " " << entry.message;

        // Details
        if (!entry.details.empty()) {
            ss << "\n  Details: " << entry.details;
        }

        return ss.str();
    }

    std::string InputLogger::formatJSON(const LogEntry& entry) const {
        std::stringstream ss;
        ss << "{";
        ss << "\"timestamp\":" << entry.timestamp.time_since_epoch().count() << ",";
        ss << "\"level\":\"" << static_cast<int>(entry.level) << "\",";
        ss << "\"type\":\"" << static_cast<int>(entry.type) << "\",";
        ss << "\"frame\":" << entry.frameNumber << ",";
        ss << "\"category\":\"" << entry.category << "\",";
        ss << "\"message\":\"" << entry.message << "\"";
        if (!entry.details.empty()) {
            ss << ",\"details\":\"" << entry.details << "\"";
        }
        ss << "}";
        return ss.str();
    }

    std::string InputLogger::formatCSV(const LogEntry& entry) const {
        std::stringstream ss;
        ss << entry.timestamp.time_since_epoch().count() << ",";
        ss << static_cast<int>(entry.level) << ",";
        ss << static_cast<int>(entry.type) << ",";
        ss << "\"" << entry.category << "\",";
        ss << "\"" << entry.message << "\",";
        ss << "\"" << entry.details << "\"";
        return ss.str();
    }

    std::string InputLogger::formatEvent(const InputEvent& event) const {
        std::stringstream ss;
        ss << "Type: " << static_cast<int>(event.type);
        ss << ", Device: " << event.deviceId;
        ss << ", Consumed: " << (event.consumed ? "true" : "false");
        ss << ", Synthetic: " << (event.synthetic ? "true" : "false");
        return ss.str();
    }

    std::string InputLogger::formatSnapshot(const InputSnapshot& snapshot) const {
        std::stringstream ss;
        ss << "Frame: " << snapshot.frameNumber;
        ss << ", Delta: " << snapshot.deltaTime << "s";
        ss << ", Events: " << snapshot.eventCount;
        ss << ", Context: " << snapshot.activeContext;
        ss << ", HasFocus: " << (snapshot.hasFocus ? "true" : "false");

        // Add device states summary
        if (snapshot.keyboard.hasAnyKeyPressed()) {
            ss << ", Keyboard: active";
        }

        if (snapshot.mouse.buttons.any()) {
            ss << ", Mouse: active";
        }

        for (PlayerID player = 0; player < MAX_PLAYERS; ++player) {
            if (const auto* gamepad = snapshot.getGamepad(player)) {
                if (gamepad->hasAnyInput()) {
                    ss << ", Gamepad" << static_cast<int>(player) << ": active";
                }
            }
        }

        return ss.str();
    }

    void InputLogger::rotateLogFile() {
        if (!fileStream_) {
            return;
        }

        closeLogFile();

        // Rotate existing files
        const std::filesystem::path logPath(config_.logFilePath);
        const std::string baseName = logPath.stem().string();
        const std::string extension = logPath.extension().string();
        const std::string directory = logPath.parent_path().string();

        for (std::uint32_t i = config_.maxRotatedFiles; i > 1; --i) {
            const std::string oldFile = directory + "/" + baseName + "." + std::to_string(i - 1) + extension;
            const std::string newFile = directory + "/" + baseName + "." + std::to_string(i) + extension;

            if (std::filesystem::exists(oldFile)) {
                std::filesystem::rename(oldFile, newFile);
            }
        }

        // Rename current log to .1
        if (std::filesystem::exists(config_.logFilePath)) {
            const std::string rotatedFile = directory + "/" + baseName + ".1" + extension;
            std::filesystem::rename(config_.logFilePath, rotatedFile);
        }

        // Open new log file
        openLogFile();
        currentRotation_++;
    }

    void InputLogger::checkFileRotation() {
        if (config_.rotateFiles &&
            stats_.currentFileSize.load(std::memory_order_acquire) > config_.maxFileSize) {
            rotateLogFile();
            stats_.currentFileSize.store(0, std::memory_order_release);
        }
    }

    bool InputLogger::openLogFile() {
        try {
            // Create directory if it doesn't exist
            const std::filesystem::path logPath(config_.logFilePath);
            const std::filesystem::path directory = logPath.parent_path();

            if (!directory.empty()) {
                std::filesystem::create_directories(directory);
            }

            std::lock_guard lock(fileMutex_);
            fileStream_ = std::make_unique<std::ofstream>(config_.logFilePath, std::ios::app);

            if (!fileStream_->is_open()) {
                fileStream_.reset();
                return false;
            }

            return true;
        }
        catch (const std::exception&) {
            return false;
        }
    }

    void InputLogger::closeLogFile() {
        std::lock_guard lock(fileMutex_);
        if (fileStream_) {
            fileStream_->close();
            fileStream_.reset();
        }
    }

    bool InputLogger::shouldLog(const LogEntry& entry) const {
        // Check level
        if (entry.level < config_.minLevel) {
            return false;
        }

        // Check category filter
        if (!isCategoryEnabled(entry.category)) {
            return false;
        }

        return true;
    }
} // namespace engine::input::debug
