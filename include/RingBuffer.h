#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

template<class T> class RingBuffer {
public:
	RingBuffer(unsigned int size = 16384)
		: _size(size), _iIn(0), _iOut(0), _buffer(new T[size]) {
	}

	RingBuffer(const RingBuffer &source)
		: _size(source._size), _iIn(source._iIn), _iOut(source._iOut), _buffer(new T[source._size]) {
		memcpy(_buffer, source._buffer, _size);
	}

	~RingBuffer() {
		delete[] _buffer;
	}

	int add(unsigned int length) {
		if (length > free()) return -1;
		_iIn = (_iIn + length) % _size;
		return 0;
	}

	int write(T *data, unsigned int count) {
		if (count > free()) return -1;
		for(; count > 0; --count) {
			_buffer[_iIn] = *(data++);
			_iIn = (_iIn + 1) % _size;
		}
		return 0;
	}

	int read(T *data, unsigned int maxCount) {
		unsigned int count = 0;
		
		if (!data) {
			count = maxCount;
			if (length() < count) count = length();
			_iOut = (_iOut + count) % _size;
			return count;
		}

		while(length() > 0 && count < maxCount) {
			*(data++) = _buffer[_iOut];
			_iOut = (_iOut + 1) % _size;
			count++;
		}

		return count;
	}

	int copy(T *data, unsigned int maxCount, int offset = 0) {
		unsigned int count = 0;
		unsigned int cOut = (_iOut + offset) % _size;
		
		if (!data) return 0;

		while((_iIn - cOut) % _size > 0 && count < maxCount) {
			*(data++) = _buffer[cOut];
			cOut = (cOut + 1) % _size;
			count++;
		}
		return count;
	}

	T get() {
		T value = 0;
		read(&value, 1);
		return value;
	}

	void flush() {
		_iIn = 0;
		_iOut = 0;
	}

	T* in() {
		return &(_buffer[_iIn]);
	}

	T* out() {
		return &(_buffer[_iOut]);
	}

	unsigned int free() const {
		return (_iOut - _iIn - 1) % _size;
	}

	unsigned int remaining() const {
		return (_iIn >= _iOut) ? (_iOut == 0 ? (_size - _iIn - 1) : (_size - _iIn)) : (_iOut - _iIn - 1);
	}

	unsigned int size() const {
		return _size;
	}

	unsigned int length() const {
		return (_iIn - _iOut) % _size;
	}

	T& operator[] (unsigned int i) {
		return *(_buffer + ((_iOut + i) % _size));
	}

	int find(T search[], unsigned int count) {
		unsigned int found = 0;

		for(unsigned int i = 0; i < length(); i++) {
			if ((*this)[i] == search[found]) {
				found++;
				if (found == count) return i - count + 1;
			} else {
				found = 0;
			}
		}

		return -1;
	}

private:
	unsigned int _size;
	unsigned int _iIn;
	unsigned int _iOut;
	T* const _buffer;
};

#endif // RINGBUFFER_H_
