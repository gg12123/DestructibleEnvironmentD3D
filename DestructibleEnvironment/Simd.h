#pragma once
#include "ByteAlignment.h"
#include <vector>

#define USE_SIMD

template<class T>
using SimdStdVector = std::vector<T, AlignmentAllocator<T, 16>>;