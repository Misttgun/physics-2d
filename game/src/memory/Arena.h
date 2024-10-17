#pragma once

#include <cstddef>

constexpr std::size_t DEFAULT_ALIGNMENT = 2 * sizeof(void*);

class Arena
{
public:
	Arena();
	~Arena();

	void* Allocate(std::size_t size, std::size_t alignment = DEFAULT_ALIGNMENT);
	void Init(std::size_t totalSize);
	void FreeAll();

	[[nodiscard]] std::size_t Used() const
	{
		return m_currOffset;
	}

	[[nodiscard]] std::size_t Capacity() const
	{
		return m_bufferLen;
	}

	Arena(Arena& arena) = delete;
	Arena(Arena&& arena) = delete;
	Arena& operator=(const Arena& arena) = delete;
	Arena& operator=(Arena&& arena) = delete;

private:
	static bool IsPowerOfTwo(std::size_t x);
	static std::size_t AlignForward(std::size_t ptr, std::size_t align);

private:
	unsigned char* m_buffer = nullptr;
	std::size_t m_bufferLen;
	std::size_t m_currOffset;
};
