/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 * *
 * @author D. Duberg, Edvin von Platen, KTH Royal Institute of Technology, Copyright (c)
 * 2021.
 * @see https://github.com/edvinvp/ufomap
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, E. von Platen, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_TYPES_H
#define UFO_MAP_TYPES_H

// UFO
#include <ufo/map/color.h>
#include <ufo/math/vector3.h>

// STD
#include <cstdint>

namespace ufo::map
{
using CodeType = uint64_t;
using KeyType = unsigned int;
using DepthType = unsigned int;
using InstanceType = uint32_t;

using Point3 = ufo::math::Vector3;

class Point3Color : public Point3
{
 public:
	Point3Color() {}

	Point3Color(Point3 const& point, Color const& color) : Point3(point), color_(color) {}

	Point3Color(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b)
	    : Point3(x, y, z), color_(r, g, b)
	{
	}

	Point3Color(Point3 const& point) : Point3(point) {}

	Point3Color(double x, double y, double z) : Point3(x, y, z) {}

	Point3Color(uint8_t r, uint8_t g, uint8_t b) : color_(r, g, b) {}

	Point3Color(Color const& color) : color_(color) {}

	Point3Color& operator=(Point3Color const& rhs)
	{
		Point3::operator=(rhs);
		color_ = rhs.color_;
		return *this;
	}

	Color const& getColor() const { return color_; }

	Color& getColor() { return color_; }

	void setColor(Color const& new_color) { color_ = new_color; }

	void setColor(uint8_t r, uint8_t g, uint8_t b)
	{
		color_.r = r;
		color_.g = g;
		color_.b = b;
	}

 protected:
	Color color_;
};

class Point3ColorInstance : public Point3
{
 public:
	Point3ColorInstance() {}

	Point3ColorInstance(Point3 const& point, Color const& color)
	    : Point3(point), color_(color)
	{
	}

	Point3ColorInstance(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b)
	    : Point3(x, y, z), color_(r, g, b)
	{
	}

	Point3ColorInstance(Point3 const& point) : Point3(point) {}

	Point3ColorInstance(double x, double y, double z) : Point3(x, y, z) {}

	Point3ColorInstance(uint8_t r, uint8_t g, uint8_t b) : color_(r, g, b) {}

	Point3ColorInstance(Color const& color) : color_(color) {}

	Point3ColorInstance(Point3 const& point, Color const& color, InstanceType instance)
	    : Point3(point), color_(color), instance_(instance)
	{
	}

	Point3ColorInstance(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b,
	                    InstanceType instance)
	    : Point3(x, y, z), color_(r, g, b), instance_(instance)
	{
	}

	Point3ColorInstance(Point3 const& point, InstanceType instance)
	    : Point3(point), instance_(instance)
	{
	}

	Point3ColorInstance(double x, double y, double z, InstanceType instance)
	    : Point3(x, y, z), instance_(instance)
	{
	}

	Point3ColorInstance(uint8_t r, uint8_t g, uint8_t b, InstanceType instance)
	    : color_(r, g, b), instance_(instance)
	{
	}

	Point3ColorInstance(Color const& color, InstanceType instance)
	    : color_(color), instance_(instance)
	{
	}

	Point3ColorInstance(InstanceType instance) : instance_(instance) {}

	Point3ColorInstance& operator=(Point3ColorInstance const& rhs)
	{
		Point3::operator=(rhs);
		color_ = rhs.color_;
		instance_ = rhs.instance_;
		return *this;
	}

	Color const& getColor() const { return color_; }

	Color& getColor() { return color_; }

	void setColor(Color const& new_color) { color_ = new_color; }

	void setColor(uint8_t r, uint8_t g, uint8_t b)
	{
		color_.r = r;
		color_.g = g;
		color_.b = b;
	}

	InstanceType const getInstance() const { return instance_; }
	void setInstance(InstanceType instance) { instance_ = instance; }

 protected:
	Color color_;
	InstanceType instance_;
};

}  // namespace ufo::map

#endif  // UFO_MAP_TYPES_H