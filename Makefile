# Copyright (c) 2020, Intel Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# This file incorporates work covered by the following copyright and permission notice:  
#
# Copyright 2020 Jee W. Choi, Marat Dukhan, and Xing Liu
# Permission is hereby granted, free of charge, to any person obtaining a copy of 
# this software and associated documentation files (the "Software"), to deal in 
# the Software without restriction, including without limitation the rights to use, 
# copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
# Software, and to permit persons to whom the Software is furnished to do so, subject 
# to the following conditions:
# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
GEOPM_LIB_DIR = ${HOME}/build/geopm/lib
GEOPM_INC_DIR = ${HOME}/build/geopm/include

CXXFLAGS = -std=c++11 -g -O2 -xHost
ifndef NOGEOPM
    LDLIBS += -L$(GEOPM_LIB_DIR) -lgeopm
    CPPFLAGS += -I$(GEOPM_INC_DIR)
    GEOPM_H_DEPENDENCY = geopm_empty.h
else
    CXXFLAGS += -DNOGEOPM
endif
CXX = mpicxx
LINK.o = $(LINK.cc)
LDFLAGS = -Wl,-z,noexecstack


.PHONY: all
all: bench_sse bench_avx2 bench_avx512

bench_avx512: main.o bench.o sumsq_avx512.o
	$(LINK.o) $^ $(LOADLIBS) $(LDLIBS) -o $@

bench_avx2: main.o bench.o sumsq_avx2.o
	$(LINK.o) $^ $(LOADLIBS) $(LDLIBS) -o $@

bench_sse: main.o bench.o sumsq_sse.o
	$(LINK.o) $^ $(LOADLIBS) $(LDLIBS) -o $@

main.o: main.cpp bench.hpp
	$(COMPILE.cc) $< $(OUTPUT_OPTION)

bench.o: bench.cpp bench.hpp $(GEOPM_H_DEPENDENCY)
	$(COMPILE.cc) $< $(OUTPUT_OPTION)

sumsq_avx512.o: sumsq_avx512.asm
	nasm -f elf64 -o $@ $^

sumsq_avx2.o: sumsq_avx2.asm
	nasm -f elf64 -o $@ $^

sumsq_sse.o: sumsq_sse.asm
	nasm -f elf64 -o $@ $^

.PHONY: clean
clean:
	rm -f sumsq_avx512.o sumsq_avx2.o sumsq_sse.o main.o bench.o main
