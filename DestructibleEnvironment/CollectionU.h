#pragma once

class CollectionU
{
public:
	template<class T>
	static inline int GetNextIndex(const T& coll, int curr)
	{
		return (curr + 1) % coll.size();
	}

	template<class T>
	static inline int GetPrevIndex(const T& coll, int curr)
	{
		auto c = coll.size();
		return (curr - 1 + c) % c;
	}
};
