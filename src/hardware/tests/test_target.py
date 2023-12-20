#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import unittest
from mock import Mock
from uuid import getnode as get_mac_address
import paho.mqtt.client as mqtt
import sys
sys.path.append('..')
from target.Target import Target


class TestTarget(unittest.TestCase):
    mock = Mock()
    target_identifier = 'TARGET'

    def test_set_uid(self):
        """
        Ensure that the system is using the correct MAC address and correct identifier
        """
        target = Target('test', self.mock, self.mock, self.target_identifier)

        current_mac = str(get_mac_address())
        
        # Assert that the device MAC address is being used in the UID
        self.assertTrue(current_mac in target.get_uid())

        # Assert that the correct identifier is being used in the UID
        self.assertTrue(self.target_identifier in target.get_uid())

        return


if __name__ == 'main':
    unittest.main()