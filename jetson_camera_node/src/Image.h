#pragma once
#include <array>
#include <assert.h>

template <typename T>
class Image
{
public:

	Image(int _resW, int _resH, T fill_value) : resH(_resH), resW(_resW), length(resH*resW)
	{
		data = new T[length];
		Fill(fill_value);
	}

	void Init(int _resW, int _resH, T fill_value)
	{
		delete[] data;
		resH = _resH;
		resW = _resW;
		length = resW * resH;
		data = new T[length];
		Fill(fill_value);

	};

	void Fill(T fill_value)
	{
		for (int i = 0; i < length; i++)
		{
			data[i] = fill_value;
		}
	}

	void FillFromCamera(void* source, float scale)
	{
		uint16_t* _source = (uint16_t*)source;
		for (int i = 0; i < length; i++)
		{
			data[i] = _source[i] * scale;
		}
	}

	T& At(int w, int h)
	{
		int index = GetIndex(w, h);
		return data[index];
	};

	T At(int w, int h) const
	{
		int index = GetIndex(w, h);
		return data[index];
	};

	void StoreLarger(T value, int w, int h)
	{
		if (value > At(w, h))
			At(w, h) = value;
	};
	void StoreSmaller(T value, int w, int h)
	{
		if (value < At(w, h))
			At(w, h) = value;

	};

	void Compare(const void* input_data, Image<bool>& output_mask, float dt = 1.f) const
	{
		const float* input_float_data = (const float*)input_data;
		for (int i = 0; i < length; i++)
		{
			float value = input_float_data[i];
			if (value == 0)
				output_mask[i] = false;
			else
			{
				output_mask[i] = input_float_data[i] < data[i]* dt;
				//output_mask[i] = input_float_data[i] < 9.f;
			}
		}
	};

	T* GetPtr() { return data; }

	cv::Mat ToOpenCV()
	{
		cv::Mat image(resH, resW, CV_8UC1, 255);

		for(int h = 0; h < resH; h++)
		{
			for (int w = 0; w < resW; w++)
			{
				image.at<uchar>(cv::Point(w, h)) = (char)( At(w, h) * 255);
			}
		}

		return image;
	}

	void FillResolution(int& _resH, int& _resW) const
	{
		_resH = resH;
		_resW = resW;
	}


	~Image()
	{
		delete [] data;
		data = nullptr;
	};


	constexpr inline T& operator [] (int index) { return data[index]; }
	constexpr inline T operator [] (int index) const { return  data[index]; }

private:

	inline int GetIndex(int w, int h) const
	{
		int index = resW * h + w;
		assert(index < length);
		return index;
	}

	T* data = nullptr;
	int resH = 0;
	int resW = 0;
	int length = 0;

};
