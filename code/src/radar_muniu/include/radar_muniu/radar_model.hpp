// radar_model.hpp
#pragma once
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <cctype>
#include <iostream>

namespace radar_muniu
{

	class RadarModel
	{
	public:
		RadarModel();
		~RadarModel();

	public:

		static bool isLocalBigEndian()
		{
			int iData = 1;
			char* p = (char*)&iData;
			if (*p == 1)
			{
				return false;
			}
			else
			{
				return true;
			}
		}

		template <typename T>
		static unsigned char convert(T& val, unsigned char* src, bool isBigEndian = true)
		{
			unsigned char size = sizeof(T);
			unsigned char* dest = (unsigned char*)&val;
			if (isBigEndian == isLocalBigEndian())
			{
				std::copy(src, src + size, dest);
			}
			else
			{
				std::reverse_copy(src, src + size, dest);
			}
			return size;
		}

		template <typename ElemType, std::size_t N>
		static unsigned char convert(std::array<ElemType, N>& arr, unsigned char* src, bool isBigEndian = true)
		{
			unsigned char totalSize = 0;
			unsigned char elemSize = sizeof(ElemType);
			for (std::size_t i = 0; i < N; ++i)
			{
				totalSize += convert(arr[i], src + totalSize, isBigEndian);
			}
			return totalSize;
		}


		static bool check_signal(std::string& signal, std::string& symbol) {
			if (symbol.empty()) {
				return false;
			}

			if (symbol.back() == '_') {
				return signal.find(symbol) != std::string::npos;
			}
			else {
				if (signal.length() < symbol.length()) {
					return false;
				}
				std::string tail = signal.substr(signal.length() - symbol.length());
				return tail == symbol;
			}
		}


		static std::vector<std::string> splitByUnderscore(const std::string& s) {
			std::vector<std::string> tokens;
			std::string token;
			std::istringstream tokenStream(s);
			while (std::getline(tokenStream, token, '_')) {
				if (!token.empty()) {
					tokens.push_back(token);
				}
			}
			return tokens;
		}

		static std::string getNumberFromString(
			const std::string& input_str,
			size_t part_index,
			const std::string& default_val = "")
		{
			std::vector<std::string> parts = splitByUnderscore(input_str);

			if (part_index >= parts.size()) {
				return default_val;
			}

			const std::string& target_part = parts[part_index];
			std::string first_continuous_digits;
			bool is_collecting = false;

			for (char c : target_part) {
				if (std::isdigit(c)) {
					is_collecting = true;
					first_continuous_digits += c;
				}
				else {
					if (is_collecting) {
						break;
					}
				}
			}

			return first_continuous_digits.empty() ? default_val : first_continuous_digits;
		}

	};

}  // namespace radar_muniu