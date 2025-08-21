//
// Created by Andres Guerrero on 20-08-25.
//

#pragma once

#include <stack>
#include <type_traits>

#include "../core/Resource.h"

namespace engine::resources {
    template <typename T>
    class ResourcePool {
        static_assert(std::is_base_of_v<Resource, T>, "T must derive from Resource");

    public:
        using CreateFunc = std::function<std::unique_ptr<T>()>;
        using ResetFunc = std::function<void(T*)>;

        explicit ResourcePool(const std::size_t initialSize = 10,
                     const std::size_t maxSize = 100,
                     CreateFunc creator = nullptr,
                     ResetFunc resetter = nullptr)
            : maxSize_(maxSize), createFunc_(creator), resetFunc_(resetter) {
            // Pre-allocate initial objects
            for (std::size_t i = 0; i < initialSize; ++i) {
                if (createFunc_) {
                    available_.push(createFunc_());
                }
            }
        }

        ~ResourcePool() {
            // Clean up all pooled objects
            while (!available_.empty()) {
                available_.pop();
            }
        }

        // Acquire object from pool
        std::unique_ptr<T> acquire() {
            std::unique_lock lock(mutex_);

            if (available_.empty()) {
                // Create new object if under limit
                if (totalCreated_ < maxSize_ && createFunc_) {
                    ++totalCreated_;
                    return createFunc_();
                }

                return nullptr;
            }

            auto obj = std::move(available_.top());
            available_.pop();
            ++acquiredCount_;

            return obj;
        }

        // Return object to pool
        void release(std::unique_ptr<T> obj) {
            if (!obj) return;

            std::unique_lock lock(mutex_);

            // Reset object state
            if (resetFunc_) {
                resetFunc_(obj.get());
            }

            // Only keep if under limit
            if (available_.size() < maxSize_) {
                available_.push(std::move(obj));
                --acquiredCount_;
            }
        }

        // Get pool statistics
        struct PoolStats {
            std::size_t availableCount;
            std::size_t acquiredCount;
            std::size_t totalCreated;
            std::size_t maxSize;
        };

        PoolStats getStats() const {
            std::shared_lock lock(mutex_);

            return {
            available_.size(),
                acquiredCount_,
                totalCreated_,
                maxSize_
            };
        }

        // Clear pool
        void clear() {
            std::unique_lock lock(mutex_);

            while (!available_.empty()) {
                available_.pop();
            }
        }

        // Resize pool
        void resize(const std::size_t newMaxSize) {
            std::unique_lock lock(mutex_);
            maxSize_ = newMaxSize;

            // Remove excess objects
            while (available_.size() > maxSize_) {
                available_.pop();
            }
        }

    private:
        std::stack<std::unique_ptr<T>> available_;
        std::size_t maxSize_;
        std::size_t totalCreated_ = 0;
        std::size_t acquiredCount_ = 0;
        CreateFunc createFunc_;
        ResetFunc resetFunc_;
        mutable std::shared_mutex mutex_;
    };
}
