import cv2, queue as Queue, threading, time

is_frame = True
# bufferless VideoCapture
Terminated = False

class VideoCaptureQ:

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2016)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1520)
        self.cap.set(cv2.CAP_PROP_FPS,60)
        self.q = Queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while not Terminated:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    global is_frame
                    is_frame = False
                    self.cap.release()
                    break
                if not self.q.empty():
                    try:
                        self.q.get_nowait()   # discard previous (unprocessed) frame
                    except Queue.Empty:
                        pass
                self.q.put(frame)
            except:
                break
        #On thread exit
        self.cap.release()

    def read(self):
        return self.q.get()
        
import time

cap = VideoCaptureQ(0)

while True:

    t1 = time.time()

    if is_frame == False:
        print('no more frames left')
        break

    try:
        ori_frame = cap.read()
        print(ori_frame.shape)
        cv2.imshow("frame", ori_frame)
        key=cv2.waitKey(33)
        if key == ord('q'):
            break
        
        # do your stuff
    except Exception as e:
        print(e)
        break
    t2 = time.time()
    print(f'FPS: {1/(t2-t1)}')
    
Terminated = True
cap.cap.release()
cap.t.join()
cap.q.join()
