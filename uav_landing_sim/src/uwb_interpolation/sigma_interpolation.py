 # 
 # This file is part of the uav_landing_sim distribution (https://github.com/dimianx/uav_landing_sim).
 # Copyright (c) 2025 Dmitry Anikin.
 # 
 # This program is free software: you can redistribute it and/or modify  
 # it under the terms of the GNU General Public License as published by  
 # the Free Software Foundation, version 3.
 #
 # This program is distributed in the hope that it will be useful, but 
 # WITHOUT ANY WARRANTY; without even the implied warranty of 
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 # General Public License for more details.
 #
 # You should have received a copy of the GNU General Public License 
 # along with this program. If not, see <http://www.gnu.org/licenses/>.
 #

from scipy.interpolate import InterpolatedUnivariateSpline

class SigmaSplineInterpolator():

    def __init__(self):
        self.SIGMA = {            
            0.50 : 0.018,
            1.00 : 0.024,
            1.50 : 0.024,
            2.00 : 0.023,
            2.50 : 0.021,
            3.00 : 0.021,
            3.50 : 0.025,
            4.00 : 0.019,
            4.50 : 0.015,
            5.00 : 0.022,
            5.50 : 0.025,
            6.00 : 0.019,
            6.50 : 0.014,
            7.00 : 0.014,
            7.50 : 0.018,
            8.00 : 0.020,
            8.50 : 0.021,
            9.00 : 0.026,
            9.50 : 0.022,
            10.00 : 0.024,
            11.00 : 0.020,
            12.00 : 0.014,
            13.00 : 0.010,
            14.00 : 0.015,
            15.00 : 0.016,
            16.00 : 0.016,
            17.00 : 0.018,
            18.00 : 0.015,
            19.00 : 0.015,
            20.00 : 0.017
        }

        self.interpolator = InterpolatedUnivariateSpline(list(self.SIGMA.keys()), list(self.SIGMA.values()))

    def interpolate(self, distance):
        return self.interpolator(distance)
