import cv2
import numpy as np
from urllib.request import urlopen

stream_url = 'http://192.168.4.1:129/stream'

with urlopen(stream_url) as stream:
    data = b""
    while True:
        chunk = stream.read(1024)
        if not chunk:
            print("No more data from stream")
            break
        data += chunk

        # print("Data snippet:", data[:100])
        a = data.find(b'\xff\xd8')  # jpeg start
        b = data.find(b'\xff\xd9')  # jpeg end
        if (a != -1 and b != -1 and b > a) or len(data) > 8000:
            jpg = data[a:b+2]
            data = data[b+2:]
            
            img_array = np.frombuffer(jpg, dtype=np.uint8)
            if img_array.size == 0:
                continue
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img is None:
                print("Warning: imdecode returned None, skipping frame")
                continue
            
            cv2.imshow('Stream', img)
            if cv2.waitKey(1) == 27: 
                break

cv2.destroyAllWindows()
