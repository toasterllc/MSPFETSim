REM  convertEtFetTxt.bat
REM
REM  Copyright (C) 2007 - 2013 Texas Instruments Incorporated - http://www.ti.com/ 
REM
REM
REM  Redistribution and use in source and binary forms, with or without 
REM  modification, are permitted provided that the following conditions 
REM  are met:
REM
REM    Redistributions of source code must retain the above copyright 
REM    notice, this list of conditions and the following disclaimer.
REM
REM    Redistributions in binary form must reproduce the above copyright
REM    notice, this list of conditions and the following disclaimer in the 
REM    documentation and/or other materials provided with the   
REM    distribution.
REM
REM    Neither the name of Texas Instruments Incorporated nor the names of
REM    its contributors may be used to endorse or promote products derived
REM    from this software without specific prior written permission.
REM
REM  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
REM  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
REM  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
REM  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
REM  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
REM  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
REM  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
REM  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
REM  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
REM  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
REM  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set "TMP=%1%\tools\tmp"
set "SED=%1%\tools\sed.exe"
set "SRECCAT=%1%\tools\srec_cat.exe"
set "SRC=%1\objects\image\%2.txt"
set "DST=%1\include\%2.h"
set "TMP_TXT=%TMP%\%2_tmp.txt"
set "TMP_HEADER=%TMP%\%2_tmp.h"

if not exist %TMP% mkdir %TMP%
if not %ERRORLEVEL% == 0 (
    @echo "Failed creating tmp directory"
    exit 1
)

%SRECCAT% %SRC% -guess -fill 0xFF -within %SRC% -guess -range-padding 2 -o %TMP_TXT% -ti_txt
if not %ERRORLEVEL% == 0 (
    @echo "Failed creating tmp file"
    exit 1
)

%SRECCAT% %TMP_TXT% -guess -o %TMP_HEADER% -ca %2%Image -ow -c_comp
if not %ERRORLEVEL% == 0 (
    @echo "Failed creating header file"
    exit 1
)

%SED% -i "s/unsigned short/uint16_t/g;s/unsigned long/uint32_t/g" %TMP_HEADER%
if not %ERRORLEVEL% == 0 (
    @echo "Failed patching type names"
    exit 1
)

copy %TMP_HEADER% %DST%
if not %ERRORLEVEL% == 0 (
    @echo "Failed copying file"
    exit 1
)

del %TMP_TXT%
del %TMP_HEADER%
