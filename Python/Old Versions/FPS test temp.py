import time


start = time.perf_counter()
end = start

while(1):
    start = time.perf_counter()
    print (1/(start-end))
    end = start
