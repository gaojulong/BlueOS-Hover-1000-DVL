import time
from fakedev import DvlDriver

if __name__ == '__main__':
    driver = DvlDriver()
    driver.start()
    while True:
        time.sleep(1)
