#pragma once
#include <vector>

// When using these hashes always...

// First reset the next hash counter
// Then assign hashes as and when required by the algorithm
// Finally, un-assign any assigned hashes ready for the next algorithm that wants to use them

template<class T>
class ObjectWithHash
{
public:
	static void ResetNextHashCounter()
	{
		NextHash = 0;
	}

	static int GetNumHashesAssigned()
	{
		return NextHash;
	}

	void ResetHash()
	{
		m_Hash = UnAssignedHash;
	}

	bool HashIsAssigned() const
	{
		return m_Hash != UnAssignedHash;
	}

	int GetHash() const
	{
		return m_Hash;
	}

	void TryAssignHash()
	{
		if (!HashIsAssigned())
			AssignHash();
	}

	void AssignHash()
	{
		m_Hash = NextHash;
		NextHash++;
	}

	static void ResetHashes(const std::vector<T*>& objects)
	{
		for (auto o : objects)
			o->ResetHash();
	}

private:
	static int NextHash;
	static constexpr int UnAssignedHash = -1;

	int m_Hash = UnAssignedHash;
};


template<class T>
int ObjectWithHash<T>::NextHash;