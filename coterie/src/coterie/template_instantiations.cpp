/**
 * \file template_instantiations.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-22
 *
 * \copyright
 *
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "coterie/PointSet.hpp"
#include "coterie/RasterSet.hpp"
#include "coterie/EllipsoidalSet.hpp"
#include "coterie/PolytopeSet.hpp"

namespace coterie
{
template class PointSet<1>;
template class PointSet<2>;
template class PointSet<3>;
template class PointSet<4>;
template class PointSet<6>;
template class RasterSet<1>;
template class RasterSet<2>;
template class RasterSet<3>;
template class RasterSet<4>;
template class RasterSet<6>;
template class EllipsoidalSet<1>;
template class EllipsoidalSet<2>;
template class EllipsoidalSet<3>;
template class EllipsoidalSet<4>;
template class EllipsoidalSet<6>;
template class PolytopeSet<1>;
template class PolytopeSet<2>;
template class PolytopeSet<3>;
template class PolytopeSet<4>;
template class PolytopeSet<6>;
}
