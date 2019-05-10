#pragma once
#include "ByteAlignment.h"
#include "UnOrderedVector.h"
#include <vector>

#define USE_SIMD

template<class T>
using SimdStdVector = std::vector<T, AlignmentAllocator<T, 16>>;

template<class T>
using SimdUnOrderedVector = UnOrderedVector<T, AlignmentAllocator<T, 16>>;