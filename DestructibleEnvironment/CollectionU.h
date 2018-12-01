#pragma once
#include <vector>

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

	template<class Tcollec, class Tval>
	static inline bool Contains(const Tcollec& collec, const Tval& val)
	{
		return std::find(collec.cbegin(), collec.cend(), val) != collec.cend();
	}
};
