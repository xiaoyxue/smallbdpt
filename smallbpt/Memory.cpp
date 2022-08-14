#include "Memory.h"
#include <memory>

void* AllocAligned(size_t size) {
	return _aligned_malloc(size, L1_CACHE_LINE_SIZE);

}

void FreeAligned(void* ptr) {
	if (!ptr) return;
	_aligned_free(ptr);
}