#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from BLEHardwareTarget import BLEHardwareTarget
import time


def main():
    ble = BLEHardwareTarget(log=True, port='/dev/ttyACM1')

    ble.advertise('TARGET-1234', 'IDLE', False, power=8)

    ble.configure_connection()


    while(1):    
        write = input("Would you like to write to a characteristic? (y/n): ")

        if write == 'y':
            ble.set_capture_state_characteristic('IDLE')
        else:
            pass

        stop = input('Would you like to exit the program? (y/n): ')

        if stop == 'y':
            break


if __name__ == '__main__':
    main()