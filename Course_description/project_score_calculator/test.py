"""
## =========================================================================== ## 

MIT License

Copyright (c) 2020 Roman Parak

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## =========================================================================== ## 

Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak
File Name: test.py

## =========================================================================== ## 
"""

# System
import sys

# Numpy
import numpy as np

# DateTime
import datetime

def penalty_eqn(s_m, Dt):
    """
    Description:
    Simple function for calculating the penalty for late submission of a project.

    Args:
    :in  (1): maximum possible score
    :in  (2): difference between the date of deadline and the date of assignment of the project (in hours)
    :out (1): rounded result of the calculation
    """ 

    # difference between the date of deadline and the date of assignment
    delta_p = s_m/10 
    # main equation of penalty for late submission
    p_s     = abs((Dt/24)*np.exp(0.5)) + delta_p
    
    return round(s_m - p_s) 

def main():
    # Initialize input data
    PROJECT_TYPE     = 1 # 1 -> Seminar Paper; 2 -> Project no. 1; 3 -> Project no. 3
    PROJECT_TYPE_str = ['Seminar Paper', 'Project 1', 'Project 2']
    CALUCLATOR_TEST  = False
    
    # Maximum possible score
    s_max = [20, 30, 40]

    if PROJECT_TYPE == 0:
        print('[WARN] Unexpected input error!')
        return

    print('[INFO] Possible score calculator for the project:', PROJECT_TYPE_str[PROJECT_TYPE-1])

    # Assignment date [Seminar Paper, Project 1, Project 2]
    s_dt   = [datetime.datetime(2021, 2, 15, 0), datetime.datetime(2021, 2, 22, 0), datetime.datetime(2021, 3, 29, 0)]
    # Deadline [Seminar Paper, Project 1, Project 2]
    d_dt   = [datetime.datetime(2021, 3, 30, 0), datetime.datetime(2021, 3, 23, 0), datetime.datetime(2021, 5, 4, 0)]
    # Auxiliary data for the calculation test (six hour later)
    aux_dt = [datetime.datetime(2021, 3, 30, 6), datetime.datetime(2021, 3, 23, 6), datetime.datetime(2021, 5, 4, 6)]
    
    # Current time
    c_dt = datetime.datetime.now()

    if CALUCLATOR_TEST == True:
        # Time calculation {TEST}
        Delta_t = (d_dt[PROJECT_TYPE-1] - aux_dt[PROJECT_TYPE-1])
        # Status of the project {TEST}
        project_status_t = (aux_dt[PROJECT_TYPE-1] - s_dt[PROJECT_TYPE-1])
    else:
        # Time calculation {REAL}
        Delta_t = (d_dt[PROJECT_TYPE-1] - c_dt)
        # Status of the project {REAL}
        project_status_t = (c_dt - s_dt[PROJECT_TYPE-1])

    if project_status_t.total_seconds() > 0:
        print('[INFO] The project is running.')
    else:
        print('[INFO] The project has not started.')
        return

    # Transform datetime parameter to seconds
    Delta_t_sec = Delta_t.total_seconds()
    # Transofrm seconds to hours
    Delta_t_hr  = divmod(Delta_t_sec, 3600)[0]
    
    # The main condition of the penalty for the project
    if Delta_t_hr >= 0:
        # Success
        print('[INFO] Everything is fine, the submission of the project is without a penalty score.')
        s = s_max[PROJECT_TYPE-1]
    else:
        # Problem
        print('[INFO] Oops, a penalty for late submission of a project.')
        print('[INFO] Number of hours:', str(Delta_t_hr))

        # Calculation of the equation for the penalty
        s = penalty_eqn(s_max[PROJECT_TYPE-1], Delta_t_hr)

    # Publish the result
    print('[INFO] The maximum possible score for the project:', str(s))

if __name__ == '__main__':
    sys.exit(main())
