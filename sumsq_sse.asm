; Copyright (c) 2020, Intel Corporation
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
;     * Redistributions of source code must retain the above copyright
;       notice, this list of conditions and the following disclaimer.
;
;     * Redistributions in binary form must reproduce the above copyright
;       notice, this list of conditions and the following disclaimer in
;       the documentation and/or other materials provided with the
;       distribution.
;
;     * Neither the name of Intel Corporation nor the names of its
;       contributors may be used to endorse or promote products derived
;       from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
; This file incorporates work covered by the following copyright and permission notice:  
;
; Copyright 2020 Jee W. Choi, Marat Dukhan, and Xing Liu
; Permission is hereby granted, free of charge, to any person obtaining a copy of 
; this software and associated documentation files (the "Software"), to deal in 
; the Software without restriction, including without limitation the rights to use, 
; copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
; Software, and to permit persons to whom the Software is furnished to do so, subject 
; to the following conditions:
; The above copyright notice and this permission notice shall be included in all 
; copies or substantial portions of the Software.
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
; INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
; PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
; HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
; CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
; OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
section .text align=64

%macro SUMSQ 1
global sumsq_%1
sumsq_%1:
	sub rdi, -64
	vzeroall
	mov eax, 128
	sub rsi, 16

	jb .restore
	align 32
.process_by_16:
	movapd xmm0, [rdi - 64]
	movapd xmm1, [rdi - 48]
	movapd xmm2, [rdi - 32]
	movapd xmm3, [rdi - 16]
	movapd xmm4, [rdi]
	movapd xmm5, [rdi + 16]
	movapd xmm6, [rdi + 32]
	movapd xmm7, [rdi + 48]


%rep %1
	vfmadd231pd xmm8, xmm0, xmm0
	vfmadd231pd xmm9, xmm1, xmm1
	vfmadd231pd xmm10, xmm2, xmm2
	vfmadd231pd xmm11, xmm3, xmm3
	vfmadd231pd xmm12, xmm4, xmm4
	vfmadd231pd xmm13, xmm5, xmm5
	vfmadd231pd xmm14, xmm6, xmm6
	vfmadd231pd xmm15, xmm7, xmm7
%endrep
	add rdi, rax
	sub rsi, 16
	jae .process_by_16
.restore:
	add rsi, 16
	jz .finish
	int 3
.finish:
	vzeroupper
	ret
%endmacro

%macro SUMSQF 1
global sumsqf_%1
sumsqf_%1:
	sub rdi, -64
	mov eax, 128
	vzeroall
	sub rsi, 32

	jb .restore
	align 32
.process_by_32:
	movaps xmm0, [rdi - 64]
	movaps xmm1, [rdi - 48]
	movaps xmm2, [rdi - 32]
	movaps xmm3, [rdi - 16]
	movaps xmm4, [rdi]
	movaps xmm5, [rdi + 16]
	movaps xmm6, [rdi + 32]
	movaps xmm7, [rdi + 48]


%rep %1
	vfmadd231ps xmm8, xmm0, xmm0
	vfmadd231ps xmm9, xmm1, xmm1
	vfmadd231ps xmm10, xmm2, xmm2
	vfmadd231ps xmm11, xmm3, xmm3
	vfmadd231ps xmm12, xmm4, xmm4
	vfmadd231ps xmm13, xmm5, xmm5
	vfmadd231ps xmm14, xmm6, xmm6
	vfmadd231ps xmm15, xmm7, xmm7
%endrep
	add rdi, rax
	sub rsi, 32
	jae .process_by_32
.restore:
	add rsi, 32
	jz .finish
	int 3
.finish:
	vzeroupper
	ret
%endmacro

SUMSQ 0
SUMSQ 1
SUMSQ 2
SUMSQ 4
SUMSQ 8
SUMSQ 16
SUMSQ 32
SUMSQ 64
SUMSQ 128

SUMSQF 0
SUMSQF 1
SUMSQF 2
SUMSQF 4
SUMSQF 8
SUMSQF 16
SUMSQF 32
SUMSQF 64
SUMSQF 128

section .data
; Make the stride available externally
global SUMSQ_BASE_STRIDE
SUMSQ_BASE_STRIDE: dw 16
