#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from BLEHardwarePayload import BLEHardwarePayload
import time


def main():
    ble = BLEHardwarePayload(log=True, port='/dev/ttyACM0')

    while(1):
        ble.scan('IDLE')


if __name__ == '__main__':
    main()