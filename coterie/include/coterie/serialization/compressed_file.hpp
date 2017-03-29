/**
 * \file compressed_file.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-3-28
 *
 * \copyright
 *
 * Copyright (c) 2017, Georgia Tech Research Corporation
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

#ifndef COMPRESSED_FILE_HPP
#define COMPRESSED_FILE_HPP

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <iostream>
#include <fstream>

namespace coterie
{
template <class T>
inline void saveGZFile(const T & o, const char* fname)
{
	std::ofstream ofs(fname, std::ios::out|std::ios::binary|std::ios::trunc);
	boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
	out.push(boost::iostreams::gzip_compressor());
	out.push(ofs);
	boost::archive::binary_oarchive oa(out);
	oa << o;
}
template <class T>
inline void loadGZFile(T & o, const char* fname)
{
	std::ifstream ifs(fname, std::ios::in|std::ios::binary);
	assert(ifs.good()); // catch if file not found
	boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
	in.push(boost::iostreams::gzip_decompressor());
	in.push(ifs);
	boost::archive::binary_iarchive ia(in);
	ia >> o;
}
}

#endif // COMPRESSED_FILE_HPP
