import cv2
import math


# rescale to -PI..+PI
def scalePI(v):  
    d = v
    while (d < 0):    
        d = d + 2*math.pi    
    while (d >= 2*math.pi):  
        d = d - 2*math.pi    
    if (d >= math.pi):
        return (-2*math.pi+d)
    else:
        if (d < -math.pi):
            return (2*math.pi+d)
        else:
            return d

# computes minimum distance between x radiant (current-value) and w radiant (set-value)
def distancePI(x, w):
    # cases:
    # w=330 degree, x=350 degree => -20 degree
    # w=350 degree, x=10  degree => -20 degree
    # w=10  degree, x=350 degree =>  20 degree
    # w=0   degree, x=190 degree => 170 degree
    # w=190 degree, x=0   degree => -170 degree
    d = scalePI(w - x)
    if (d < -math.pi):
        d = d + 2*math.pi
    else:
        if (d > math.pi):
            d = d - 2*math.pi
    return d;


# alpha blend two images into one
def alphaBlend(background, alpha1, foreground, alpha2):
    background = cv2.applyColorMap(background, cv2.COLORMAP_JET)
    background = cv2.cvtColor(background, cv2.COLOR_RGB2RGBA)
    foreground = cv2.applyColorMap(foreground, cv2.COLORMAP_WINTER)                
    foreground = cv2.cvtColor(foreground, cv2.COLOR_RGB2RGBA)        
    background[:,:,3] = alpha1
    foreground[:,:,3] = alpha2
    # normalize alpha channels from 0-255 to 0-1
    alpha_background = background[:,:,3] / 255.0
    alpha_foreground = foreground[:,:,3] / 255.0
    # set adjusted colors
    for color in range(0, 3):
        background[:,:,color] = alpha_foreground * foreground[:,:,color] + \
            alpha_background * background[:,:,color] * (1 - alpha_foreground)
    # set adjusted alpha and denormalize back to 0-255
    background[:,:,3] = (1 - (1 - alpha_foreground) * (1 - alpha_background)) * 255
    return background

