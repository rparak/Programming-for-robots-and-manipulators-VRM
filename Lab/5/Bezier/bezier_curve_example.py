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
File Name: bezier_curve_example.py
## =========================================================================== ## 
"""

# System (Default Lib.)
import sys
# Numpy (Array computing Lib.) [pip3 install numpy]
import numpy as np
# Mtaplotlib (Visualization Lib.) [pip3 install matplotlib]
import matplotlib.pyplot as plt


class bezier_ctrl(object):
    """
    Description:
        A Bézier curve is a parametric curve used in computer graphics and related fields.

        The class shows several types of Bézier curves (Linear, Quadratic, Cubic).
    """
    def __init__(self, p_0, p_1, p_2, p_3, step):
        # << PUBLIC >> #
        # Time t ∈ [0, 1]
        self.t = np.linspace(0.0, 1.0, step)
        # Points
        self.p = [p_0, p_1, p_2, p_3]
        # Remove empty (None) parts of the array.
        self.p = [x for x in self.p if x is not None]
        # << PRIVATE >> #
        # Display (Plot) variable.
        self.__plt = plt

    @staticmethod
    def __linear_curve(p_0, p_1, t):
        """
        Description:
            Given two control points p_{0} and p_{1} we define the linear Bezier curve to be the curve parametrized by:

            p(t) = (1 - t)*p_{0} + t*p_{1}, t ∈ [0, 1]

        Args:
            (1 - 2) p_0, p_0 [Float Array]: Multiple points to create a curve.
            (3) t [Float Array]: Time variable.
        Returns:
            (1 - 2) x, y [Float Array]: Results of curve values.

        Examples:
            self.__linear_curve([1.0, 1.0], [2.0, 2.0])
        """

        x = (1 - t) * p_0[0] + t * p_1[0]
        y = (1 - t) * p_0[1] + t * p_1[1]

        return x, y

    @staticmethod
    def __quadratic_curve(p_0, p_1, p_2, t):
        """
        Description:
            Given three control points p_{0}, p_{1} and p_{2} we define the quadratic Bezier curve (degree 2 Bezier curve)
            to be the curve parametrized by:

            p(t) = ((1 - t)^2)*p_{0} + 2*t*(1 - t)*p_{1} + (t^2)*p_{2}, t ∈ [0, 1]

        Args:
            (1 - 2) p_0, p_0, p_2 [Float Array]: Multiple points to create a curve.
            (3) t [Float Array]: Time variable.
        Returns:
            (1 - 2) x, y [Float Array]: Results of curve values.

        Examples:
            self.__quadratic_curve([1.0, 1.0], [2.0, 2.0], [3.0, 2.0])
        """

        x = (1 - t)**2 * p_0[0] + 2 * t * (1 - t) * p_1[0] + t**2 * p_2[0]
        y = (1 - t)**2 * p_0[1] + 2 * t * (1 - t) * p_1[1] + t**2 * p_2[1]

        return x, y   

    @staticmethod
    def __cubic_curve(p_0, p_1, p_2, p_3, t):
        """
        Description:
            Given four control points p_{0}, p_{1}, p_{2} and p_{3} we define the cubic Bezier curve (degree 3 Bezier curve) to
            be the curve parametrized by:

            p(t) = ((1 - t)^3)*p_{0} + 3*t*((1 - t)^2)*p_{1} + (3*t^2)*(1 - t)*p_{2} + (t^3) * p_{3}, t ∈ [0, 1]

        Args:
            (1 - 2) p_0, p_0, p_2 [Float Array]: Multiple points to create a curve.
            (3) t [Float Array]: Time variable.
        Returns:
            (1 - 2) parameter{1}, parameter{2} [Float Array]: Results of curve values.

        Examples:
            self.__cubic_curve([1.0, 1.0], [2.0, 2.0], [3.0, 2.0], [4.0, 1.0])
        """

        x = ((1 - t)**3) * (p_0[0]) + (3 * t * (1 - t)**2) * (p_1[0]) + 3 * (t**2) * (1 - t) * p_2[0] + (t**3) * p_3[0]
        y = ((1 - t)**3) * (p_0[1]) + (3 * t * (1 - t)**2) * (p_1[1]) + 3 * (t**2) * (1 - t) * p_2[1] + (t**3) * p_3[1]

        return x, y

    def __switch_dCtrl(self, s_index):
        """
        Description:
            Function to obtain a string with the number of points used in the calculation of the curve. (Figure Legend -> Label Name)

        Args:
            (1) s_index [INT]: Number of points for calculation.
        Returns:
            (1) param 1 [String]: The resulting string for the label.
        """

        # Switch Variable
        switch_var={
                2: r'Points: $p_{0}, p_{1}$',
                3: r'Points: $p_{0}, p_{1}, p_{2}$',
                4: r'Points: $p_{0}, p_{1}, p_{2}, p_{3}$',
        }

        # Return Result (Get the string with number of points for the Legend Label)
        return switch_var.get(s_index, "Wrong Input!")

    def __two_points(self):
        """
        Description:
            Function to create a multiple linear Bézier curve from two points.
        """

        x, y = self.__linear_curve(self.p[0], self.p[1], self.t)

        # Display the Linear Bézier Curve p(t) -> x, y
        self.__plt.plot(x, y, 'r--', label=r'Linear Bezier Curve: [$p_{0}$, $p_{1}$]' , linewidth=2.5)

    def __three_points(self):
        """
        Description:
            Function to create multiple linear Bézier curves and a quadratic Bézier curve from three points.
        """

        for i in range(len(self.p) - 1):
            x, y = self.__linear_curve(self.p[i], self.p[i + 1], self.t)
            
            if i == (len(self.p) - 1) - 1:
                self.__plt.plot(x, y, 'r--', label=r'Linear Bezier Curve: [$p_{0}$, $p_{1}$]; [$p_{1}$, $p_{2}$]' , linewidth=2.5)
            else:
                self.__plt.plot(x, y, 'r--', linewidth=2.5)

        x, y = self.__quadratic_curve(self.p[0], self.p[1], self.p[2], self.t)

        self.__plt.plot(x, y, 'g--', label=r'Quadratic Bezier Curve: [$p_{0}$, $p_{1}$, $p_{2}$]', linewidth=2.5)

    def __four_points(self):
        """
        Description:
            Function to create multiple linear / quadratic Bézier curves and a cubic four-point Bézier curve.
        """

        for i in range(len(self.p) - 1):
            x, y = self.__linear_curve(self.p[i], self.p[i + 1], self.t)
            
            if i == (len(self.p) - 1) - 1:
                self.__plt.plot(x, y, 'r--', label=r'Linear Bezier Curve: [$p_{0}$, $p_{1}$]; [$p_{1}$, $p_{2}$]; [$p_{2}$, $p_{3}$]' , linewidth=2.5)
            else:
                self.__plt.plot(x, y, 'r--', linewidth=2.5)

        for i in range(len(self.p) - 2):
            x, y = self.__quadratic_curve(self.p[i], self.p[i + 1], self.p[i + 2], self.t)

            if i == (len(self.p) - 2) - 1:
                self.__plt.plot(x, y, 'g--', label=r'Quadratic Bezier Curve: [$p_{0}$, $p_{1}$, $p_{2}$]; [$p_{1}$, $p_{2}$, $p_{3}$]', linewidth=2.5)
            else:
                self.__plt.plot(x, y, 'g--', linewidth=2.5)

        x, y = self.__cubic_curve(self.p[0], self.p[1], self.p[2], self.p[3], self.t)

        self.__plt.plot(x, y, 'b--', label=r'Cubic Bezier Curve: [$p_{0}$, $p_{1}$, $p_{2}$, $p_{3}$]', linewidth=2.5)

    def __display_aux_result(self):
        """
        Description:
            Function for displaying points and text labeling.
        """

        for i in range(len(self.p)):
            self.__plt.text(self.p[i][0] + 0.01, self.p[i][1] + 0.01, '$p_{' + str(i) + '}$ = [' + str(self.p[i][0]) + ', ' + str(self.p[i][1]) + ']', fontsize=20)

            if i != len(self.p) - 1:
                self.__plt.plot(self.p[i][0], self.p[i][1], marker = 'o', ms = 15, mfc = [1,1,1], markeredgecolor = [0,0,0], mew = 5)
            else:
                self.__plt.plot(self.p[i][0], self.p[i][1], label=self.__switch_dCtrl(len(self.p)), marker = 'o', ms = 15, mfc = [1,1,1], markeredgecolor = [0,0,0], mew = 5)

    def display_result(self):
        """
        Description:
            Function for calculating and displaying the results of Bézier curves.
        """

        try:
            assert len(self.p) > 1

            # Select a calculation method based on the number of points in the array (p).
            if len(self.p) == 2:
                self.__two_points()
            elif len(self.p) == 3:
                self.__three_points()
            elif len(self.p) == 4:
                self.__four_points()

            self.__display_aux_result()

            # Set additional features for successful display of the Bézier curves.
            self.__plt.grid()
            self.__plt.xlabel('x axis [Unit]', fontsize = 20, fontweight ='normal')
            self.__plt.ylabel('y axis [Unit]', fontsize = 20, fontweight ='normal')
            self.__plt.title('Bezier Curve', fontsize = 50, fontweight ='normal')
            self.__plt.legend(loc=0,fontsize=20)

            # Display a figure. Wait for the user to close the window.
            self.__plt.show()

        except AssertionError as error:
            print('[INFO] Insufficient number of entry points.')
            print('[INFO] The minimum number of entry points is 2.')

def main():
    # Initialization of the Class (Control Manipulator)
    # Input:
    #   (1 - 4) Points [Float Array]
    #   (5) Time Step  [INT]
    # Example:
    #   x = bezier_ctrl([1.0, 1.0], [1.25, 2.0], None, None, 100)

    # Try the calculation of the Bézier curve:
    # Select one of these options: 2 - Linear Curve, 3 - Quadratic Curve, 4 - Cubic Curve
    test = 4

    if test == 2:
        # Linear Curve
        bezier = bezier_ctrl([1.0, 1.0], [1.25, 2.0], None, None, 100)
    elif test == 3:
        # Quadratic Curve
        bezier = bezier_ctrl([1.0, 1.0], [1.25, 2.0], [1.75, 2.0], None, 100)
    elif test == 4:
        # Cubic Curve
        bezier = bezier_ctrl([1.0, 1.0], [1.25, 2.0], [1.75, 2.0], [2.0, 1.0], 100)

    # Display the result of the calculation -> figure with the resulting Bézier curves
    bezier.display_result()

if __name__ == '__main__':
    sys.exit(main())