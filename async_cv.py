#!/usr/bin/env python
#tiff_img = 'aruco/markers/aruco_green15_15.png'
    
    
import asyncio
import cv2
import functools
from concurrent.futures import ProcessPoolExecutor
#from multiprocessing import Queue



loop = asyncio.get_event_loop()
executor = ProcessPoolExecutor(1)
q = asyncio.Queue()

class GloomLurker(object):
    def __init__(self):
        pass
        
    def load_img(self):
        #print(image)
        cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")
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
            try:
                token = q.get_nowait()
                print(f"got token {token}")
                q.task_done()
                break
            except Exception as e:
                pass
                #print(f"q broked: {e}")
            #if Farthat:
            #    break
        cv2.destroyAllWindows()
        return

    async def test(self):
        #tiff_img = 'aruco/markers/aruco_green15_15.png'
        await loop.run_in_executor(
            executor, 
            functools.partial(self.load_img)
        )

    async def numbers(self):
        for number in range(50):
            await asyncio.sleep(0.1)
            await q.put("what the fuck")
            print(number)
        print("farting in hat")    
        await q.put("Farthat = True")
        return
        
    def go(self):
        single = asyncio.gather(self.test(), self.numbers())
        loop.run_until_complete(single)

if __name__ == '__main__':
    GL = GloomLurker()
    GL.go()
