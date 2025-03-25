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

import cv2
import numpy as np

def adjust_brightness(image, bright):
     image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
     image1 = np.array(image1, dtype = np.float64)
     image1[:,:,2] = image1[:,:,2] * bright
     image1[:,:,2][image1[:,:,2]>255]  = 255
     image1 = np.array(image1, dtype = np.uint8)
     image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    
     return image1

def add_streaks(image, noise_value = 30, length = 25, angle = 0, w = 1, beta = 0.8):
     noise = np.random.uniform(0, 256, image.shape[0:2])
     noise[np.where(noise < (256 - noise_value))] = 0
 
     k = np.array([[0, 0.1, 0],
                  [0.1, 8, 0.1],
                  [0, 0.1, 0]])
 
     noise = cv2.filter2D(noise, -1, k)
     
     trans = cv2.getRotationMatrix2D((length/2, length/2), angle-45, 1-length/100.0)  
     dig = np.diag(np.ones(length))
     k = cv2.warpAffine(dig, trans, (length, length))
     k = cv2.GaussianBlur(k,(w,w),0) 
     blurred = cv2.filter2D(noise, -1, k)
     cv2.normalize(blurred, blurred, 0, 255, cv2.NORM_MINMAX)
     streaks = np.array(blurred, dtype=np.uint8)

     streaks = np.expand_dims(streaks,2)
 
     streaks_result = image.copy()
     streaks = np.array(streaks,dtype=np.float32)
     streaks_result[:,:,0] = streaks_result[:,:,0] * (255-streaks[:,:,0])/255.0 + beta * streaks[:,:,0]
     streaks_result[:,:,1] = streaks_result[:,:,1] * (255-streaks[:,:,0])/255 + beta * streaks[:,:,0] 
     streaks_result[:,:,2] = streaks_result[:,:,2] * (255-streaks[:,:,0])/255 + beta * streaks[:,:,0]

     return streaks_result

def add_haze(image, haze_coef):
     overlay = image.copy()
     output = image.copy()
     cv2.rectangle(overlay, (0,0), (overlay.shape[1], overlay.shape[0]), (250, 250, 250), -1)
     cv2.addWeighted(overlay, haze_coef, output, 1 - haze_coef, 0, output)
     img = output.copy()
     return img

def get_extinction_coeff(visibility, wavelength, r):
    if visibility < 6:
        size_dist = 0.585 * visibility ** (1/3)
    elif 6 < visibility < 15:
        size_dist = 1.3
    else:
        size_dist = 1.6

    return np.exp((-3.91/visibility * (wavelength / 0.55) ** -size_dist) * r / 1000)