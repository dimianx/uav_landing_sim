from scipy.interpolate import InterpolatedUnivariateSpline

class RMSESplineInterpolator():

    def __init__(self):
        self.RMSE = {
            0.50 : 0.027,
            1.00 : 0.089,
            1.50 : 0.081,
            2.00 : 0.095,
            2.50 : 0.147,
            3.00 : 0.056,
            3.50 : 0.124,
            4.00 : 0.075,
            4.50 : 0.091,
            5.00 : 0.136,
            5.50 : 0.158,
            6.00 : 0.144,
            6.50 : 0.106,
            7.00 : 0.072,
            7.50 : 0.135,
            8.00 : 0.162,
            8.50 : 0.158,
            9.00 : 0.163,
            9.50 : 0.174,
            10.00 : 0.187,
            11.00 : 0.174,
            12.00 : 0.108,
            13.00 : 0.112,
            14.00 : 0.078,
            15.00 : 0.198,
            16.00 : 0.214,
            17.00 : 0.165,
            18.00 : 0.202,
            19.00 : 0.194,
            20.00 : 0.156
        }

        self.interpolator = InterpolatedUnivariateSpline(list(self.RMSE.keys()), list(self.RMSE.values()))

    def interpolate(self, distance):
        return self.interpolator(distance)
