#ifndef THREADING_H
#define THREADING_H

#include <atomic>

class Spinlock {
protected:
	std::atomic<bool> latch;

public:
	Spinlock() : Spinlock(false) {}

	Spinlock(bool flag) {
		latch.store(flag);
	}

	Spinlock(int flag) : Spinlock(flag != 0){}

	void lock() {
		bool unlatched = false;
		while (!latch.compare_exchange_weak(unlatched, true, std::memory_order_acquire)) {
			unlatched = false;
		}
	}

	void unlock() {
		latch.store(false, std::memory_order_release);
	}

	Spinlock(const Spinlock &o) {
		// We just ignore racing condition here...
		latch.store(o.latch.load());
	}

	Spinlock &operator=(const Spinlock &o) {
		// We just ignore racing condition here...
		latch.store(o.latch.load());
		return *this;
	}
};

#endif
