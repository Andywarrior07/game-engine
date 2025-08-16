//
// Created by Andres Guerrero on 15-08-25.
//

#pragma once

#include "core/Types.h"
#include "core/IAllocator.h"
#include "core/MemoryStats.h"
#include "core/AllocationInfo.h"

#include "allocators/PoolAllocator.h"
#include "allocators/RingBufferAllocator.h"
#include "allocators/LinearAllocator.h"
#include "allocators/StackAllocator.h"

#include "utils/ScopedAllocator.h"

#include "manager/MemoryManager.h"
#include "manager/MemoryConfig.h"