#ifndef __RNG_H__
#define __RNG_H__
#include <random>
class Rng
{
public:
	Rng(int aSeed = 1234) :
		mRng(aSeed)
	{}

	int GetInt()
	{
		return mDistInt(mRng);
	}

	double GetFloat()
	{
		//return (double)(std::rand()) / RAND_MAX;

		return mDistdouble(mRng);
	}


private:
	std::mt19937_64 mRng;
	std::uniform_int_distribution<int>    mDistInt;
	std::uniform_real_distribution<double> mDistdouble;
};

#endif