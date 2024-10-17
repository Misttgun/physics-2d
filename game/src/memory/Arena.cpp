#include "Arena.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>

Arena::Arena() : m_bufferLen{0}, m_currOffset{0} {}

void Arena::Init(const std::size_t totalSize)
{
	if(m_buffer != nullptr)
		free(m_buffer);

	m_bufferLen = totalSize;
	m_buffer = new unsigned char[m_bufferLen];
	m_currOffset = 0;
}

void* Arena::Allocate(const std::size_t size, const std::size_t alignment)
{
	// Align 'currOffset' forward to the specified alignment
	const std::size_t currPtr = reinterpret_cast<std::size_t>(m_buffer) + m_currOffset;
	std::size_t offset = AlignForward(currPtr, alignment);
	offset -= reinterpret_cast<std::size_t>(m_buffer);

	// Check to see if the backing memory has space left
	if (offset + size <= m_bufferLen)
	{
		void* ptr = &m_buffer[offset];
		m_currOffset = offset + size;

		// Zero the new memory by default
		memset(ptr, 0, size);
		return ptr;
	}

	assert(true);
	// We are out of memory, so we return nullptr
	return nullptr;
}

Arena::~Arena()
{
	delete[] m_buffer;
	m_buffer = nullptr;
}

void Arena::FreeAll()
{
	m_currOffset = 0;
}

bool Arena::IsPowerOfTwo(const std::size_t x)
{
	return (x & (x - 1)) == 0;
}

std::size_t Arena::AlignForward(const std::size_t ptr, const std::size_t align)
{
	assert(IsPowerOfTwo(align));

	std::size_t p = ptr;
	const std::size_t a = align;
	const std::size_t modulo = p & (a - 1); // Same as (p % a) but faster as 'a' is a power of two

	if(modulo != 0)
		p += a - modulo; // If 'p' address is not aligned, push the address to the next value which is aligned

	return p;
}
