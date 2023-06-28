/*
 * scales.hpp
 *
 *  Created on: Oct 20, 2019
 *      Author: delta
 */

#ifndef SCALES_HPP_
#define SCALES_HPP_

enum SCALE_INDEX{
	SCALE_INDEX_12_ET = 0,
	SCALE_INDEX_24_ET = 1
};

const uint16_t scales[2][128] = {{
		// Twelve tone equal temperament binary codes, indexed by MIDI note
			0,     390,   803,   1240,  1703,  2194,  2715,  3266,  3850,  4468,  5124,  5818,  // C-1
			6553,  6943,  7356,  7793,	8257,  8748,  9268,  9819,	10403, 11022, 11677, 12371, // C0 [12]
			13107, 13497, 13910, 14347, 14810, 15301, 15822, 16373, 16957, 17575, 18231, 18925, // C1 [24]
			19660, 20050, 20463, 20900, 21364, 21855, 22375, 22926, 23510, 24129, 24784, 25478, // C2 [36]
			26214, 26604, 27017, 27454, 27917, 28408, 28929, 29480, 30064, 30682, 31338, 32032, // C3 [48]
			32767, 33157, 33570, 34007, 34471, 34962, 35482, 36033, 36617, 37236, 37891, 38585, // C4 [60] (0V)
			39321, 39711, 40124, 40561, 41024, 41515, 42036, 42587,	43171, 43789, 44445, 45139, // C5 [72]
			45874, 46264, 46677, 47114, 47578, 48069, 48589, 49140, 49724, 50343, 50998, 51692, // C6 [84]
			52428, 52818, 53231, 53668,	54131, 54622, 55143, 55694, 56278, 56896, 57552, 58246, // C7 [96]
			58981, 59371, 59784, 60221, 60685, 61176, 61696, 62247, 62831, 63450, 64105, 64799, // C8 [108]
			65535},{}

};
const char notes[2][24]{
	{'C','d','D','e','E','F','g','G','a','A','b','B'}
};




#endif /* SCALES_HPP_ */
