#!/usr/bin/env python
#tiff_img = 'aruco/markers/aruco_green15_15.png'
    
    
import asyncio
import cv2
import functools
from concurrent.futures import ProcessPoolExecutor
executor = ProcessPoolExecutor(1)

cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")

def load_img(image):
    print(image)
    #im = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    #_, inv = cv2.threshold(im, 150, 255, cv2.THRESH_BINARY_INV)
    #cv2.GaussianBlur(inv, (3, 3), 0)
    while 1:
        ret, frame = cam.read()
        if not ret:
            break
        cv2.imshow('Async test', frame)
        #cv2.waitKey(0)
        if cv2.pollKey() == 27:
            break
    cv2.destroyAllWindows()
    return




async def test():
    tiff_img = 'aruco/markers/aruco_green15_15.png'
    await loop.run_in_executor(
        executor, 
        functools.partial(load_img, image=tiff_img)
    )

async def numbers():
    for number in range(50):
        await asyncio.sleep(0.1)
        print(number)
    return

if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    single = asyncio.gather(test(), numbers())
    loop.run_until_complete(single)