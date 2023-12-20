#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  




class BLEPayload:
    def set_uid(self, uid: str) -> None:
        raise NotImplementedError("Interface Method not Implemented")

    def get_discovered_targets(self) -> list:
        raise NotImplementedError("Interface Method not Implemented")
    
    def reset_discoverd_targets(self):
        raise NotImplementedError("Interface Method not Implemented")

    def scan(self, intent: str):
        """
        Scan for advertisements being broadcasted by a device
        """
        raise NotImplementedError("Interface Method not Implemented")

    def get_connected_devices(self):
        """
        Get the currently connected targets
        """
        raise NotImplementedError("Interface Method not Implemented")

    def set_notification_callbacks(self, capture_cb, in_range_cb):
        pass

    def set_client(self, server_client):
        pass

    def get_type_in_range(self) -> 'str | None':
        raise NotImplementedError("Interface Method not Implemented")