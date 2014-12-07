#!/usr/bin/env python3
# -*- Coding: UTF-8 -*-

# ---------------------------------------------------------------------------
# Open Asset Import Library (ASSIMP)
# ---------------------------------------------------------------------------
#
# Copyright (c) 2006-2010, ASSIMP Development Team
#
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms, 
# with or without modification, are permitted provided that the following 
# conditions are met:
# 
# * Redistributions of source code must retain the above
#   copyright notice, this list of conditions and the
#   following disclaimer.
# 
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other
#   materials provided with the distribution.
# 
# * Neither the name of the ASSIMP team, nor the names of its
#   contributors may be used to endorse or promote products
#   derived from this software without specific prior
#   written permission of the ASSIMP Development Team.
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
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------------------------------------------------------------------------

"""
Run the regression test suite using the settings from settings.py.

"""

import sys
import os
import subprocess
import zipfile
import collections

import settings
import utils

usage = """run [-i=...] [-e=...]

(lists of file extensions are comma delimited, i.e. `3ds,lwo,x`)
-i,--include: List of file extensions to test. If omitted,
         all file extensions are tested except those in `exclude`.

-e,--exclude: Merged with settings.exclude_extensions to produce a
         list of all file extensions to ignore.
"""

# -------------------------------------------------------------------------------
EXPECTED_FAILURE_NOT_MET, DATABASE_LENGTH_MISMATCH, \
DATABASE_VALUE_MISMATCH, IMPORT_FAILURE, \
FILE_NOT_READABLE, COMPARE_SUCCESS = range(6)

messages = collections.defaultdict(lambda: "<unknown", {
        EXPECTED_FAILURE_NOT_MET:
"""Unexpected success during import\n\
\tReturn code was 0""",

        DATABASE_LENGTH_MISMATCH:
"""Database mismatch: lengths don't match\n\
\tExpected: {0} Actual: {1}""",

        DATABASE_VALUE_MISMATCH:
"""Database mismatch: """,

        IMPORT_FAILURE:
"""Unexpected failure during import\n\
\tReturn code was {0}""",

        FILE_NOT_READABLE:
"""Unexpected failure reading file""",

        COMPARE_SUCCESS:
"""Results match archived reference dump in database\n\
\tNumber of bytes compared: {0}"""
})

outfilename_output = "run_regression_suite_output.txt"
outfilename_failur = "run_regression_suite_failures.csv"
# -------------------------------------------------------------------------------
class results:
    
    """ Handle formatting of results"""

    def __init__(self, zipin):
        """Init, given a ZIPed database """
        self.failures = []
        self.success = []
        self.zipin = zipin
    
    
    def fail(self, failfile, filename_expect, pp, msg, *args):
        """
        Report failure of a sub-test
        
        File f failed a test for pp config pp, failure notice is msg, 
        *args is format()ting args for msg
        
        """
        print("[FAILURE] " + messages[msg].format(*args))
        self.failures.append((failfile, filename_expect, pp))


    def ok(self, f, pp, msg, *args):
        """
        Report success of a sub-test

        File f passed the test, msg is a happy success note, 
        *args is format()ing args for msg.

        """
        print("[SUCCESS] " + messages[msg].format(*args)) 
        self.success.append(f)


    def report_results(self):
        """Write results to ../results/run_regression_suite_failures.txt"""

        print("\n" + ('='*60) + "\n" + "SUCCESS: {0}\nFAILURE: {1}\nPercentage good: {2}".format(
            len(self.success), len(self.failures), len(self.success)/(len(self.success)+len(self.failures))  ) + 
              "\n" + ('='*60) + "\n")

        with open(os.path.join('..', 'results',outfilename_failur), "wt") as f:
            f.write("ORIGINAL FILE;EXPECTED DUMP\n")
            f.writelines(map(
                lambda x: x[0] + ' ' + x[2] + ";" + x[1] + "\n", self.failures))
        
        if self.failures:
           print("\nSee " + settings.results + "\\" + outfilename_failur 
                 + " for more details\n\n") 
    
# -------------------------------------------------------------------------------
def mkoutputdir_andgetpath(fullpath, myhash, app):
    outfile = os.path.join(settings.results, "tmp", os.path.split(fullpath)[1] + "_" + myhash)
    try:
        os.mkdir(outfile)
    except OSError:
        pass
    
    outfile = os.path.join(outfile, app)
    return outfile


# -------------------------------------------------------------------------------
def process_dir(d, outfile_results, file_filter, zipin, result):
    shellparams = {'stdout':outfile_results, 'stderr':outfile_results, 'shell':False}

    print("Processing directory " + d)
    for f in os.listdir(d):
        fullpath = os.path.join(d, f)
        if os.path.isdir(fullpath) and not f == ".svn":
            process_dir(fullpath, outfile_results, file_filter, zipin, result)
            continue

        if f in settings.files_to_ignore:
            print("Ignoring " + f)
            continue

        # todo: Fix for multi dot extensions like .skeleton.xml
        if not file_filter(fullpath):
            continue

        for pppreset in settings.pp_configs_to_test:
            filehash = utils.hashing(fullpath, pppreset)
            failure = False
            try:
                input_expected = zipin.open(filehash, "r").read()
                # empty dump files indicate 'expected import failure'
                if not len(input_expected):
                   failure = True
            except KeyError:
                print("Didn't find "+fullpath+" (Hash is "+filehash+") in database")
                continue

            print("-"*60 + "\n  " + os.path.realpath(fullpath) + " pp: " + pppreset) 
            
            outfile_actual = mkoutputdir_andgetpath(fullpath, filehash, "ACTUAL")
            outfile_expect = mkoutputdir_andgetpath(fullpath, filehash, "EXPECT")
            outfile_results.write("assimp dump    "+"-"*80+"\n")
            outfile_results.flush()

            command = [utils.assimp_bin_path,"dump",fullpath,outfile_actual,"-b","-s","-l"]+pppreset.split()
            r = subprocess.call(command, **shellparams)

            if r and not failure:
                result.fail(fullpath, outfile_expect, pppreset, IMPORT_FAILURE, r)
                continue
            elif failure and not r:
                result.fail(fullpath, outfile_expect, pppreset, EXPECTED_FAILURE_NOT_MET)
                continue
            
            with open(outfile_expect, "wb") as s:
                s.write(input_expected) 

            try:    
                with open(outfile_actual, "rb") as s:
                    input_actual = s.read() 
            except IOError:
                continue
                
            if len(input_expected) != len(input_actual):
                result.fail(fullpath, outfile_expect, pppreset, DATABASE_LENGTH_MISMATCH,
                        len(input_expected), len(input_actual))
                continue

            outfile_results.write("assimp cmpdump "+"-"*80+"\n")
            outfile_results.flush()

            command = [utils.assimp_bin_path,'cmpdump',outfile_actual,outfile_expect]
            if subprocess.call(command, **shellparams) != 0:
                result.fail(fullpath, outfile_expect, pppreset, DATABASE_VALUE_MISMATCH)
                continue 
            
            result.ok(fullpath, pppreset, COMPARE_SUCCESS, 
                len(input_expected))     

# -------------------------------------------------------------------------------
def del_folder_with_contents(folder):
    for root, dirs, files in os.walk(folder, topdown=False):
        for name in files:
            os.remove(os.path.join(root, name))
        for name in dirs:
            os.rmdir(os.path.join(root, name))

# -------------------------------------------------------------------------------
def run_test():
    utils.find_assimp_or_die()
    def clean(f):
        f = f.strip("* \'")
        return "."+f if f[:1] != '.' else f

    ext_list = None
    for m in sys.argv[1:]:
        if m[:10]=="--exclude=":
            settings.exclude_extensions += map(clean, m[10:].split(","))
        elif m[:3]=="-e=":
            settings.exclude_extensions += map(clean, m[3:].split(","))
        elif m[:10]=="--include=":
            ext_list = m[10:].split(",")
        elif m[:3]=="-i=":
            ext_list = m[3:].split(",")
        elif m == "--help" or m == "-h":
            print(usage)
            sys.exit(0)

    if ext_list is None:
        (ext_list, err) = subprocess.Popen([utils.assimp_bin_path, "listext"],
            stdout=subprocess.PIPE).communicate()
        ext_list = str(ext_list).lower().split(";")
    # todo: Fix for multi dot extensions like .skeleton.xml
    ext_list = list(filter(lambda f: not f in settings.exclude_extensions, map(clean, ext_list)))
    file_filter = lambda x: os.path.splitext(x)[1].lower() in ext_list

    tmp_target_path = os.path.join(settings.results, "tmp")
    try:
        os.mkdir(tmp_target_path)
    except OSError as oerr:
        # clear contents if tmp folder exists already
       del_folder_with_contents(tmp_target_path)

    try:
        zipin = zipfile.ZipFile(settings.database_name + ".zip",
            "r", zipfile.ZIP_STORED)
    except IOError:
        print("Regression database ", settings.database_name,
              ".zip was not found")
        return

    res = results(zipin)
    with open(os.path.join(settings.results, outfilename_output), "wt") as outfile:
        for tp in settings.model_directories:
            process_dir(tp, outfile, file_filter, zipin, res)

    res.report_results()

# -------------------------------------------------------------------------------
if __name__ == "__main__":
    run_test()

# vim: ai ts=4 sts=4 et sw=4
