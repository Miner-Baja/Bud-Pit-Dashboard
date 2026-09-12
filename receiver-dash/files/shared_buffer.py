from collections import deque

buffer_size = 500 #number of most recent packets to keep
rolling_buffer = deque(maxlen=buffer_size)
