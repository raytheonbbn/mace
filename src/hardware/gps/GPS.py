#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from gps3.agps3threaded import AGPS3mechanism

class GPS:
    def __init__(self):

        # Invalid value indicating an 'n/a' reading from the gps
        self.INVALID = 15000

        # Class Variables
        self.latitude = self.INVALID       # Current latitude of the gps
        self.longitude = self.INVALID      # Current longitude of the gps
        self.altitude = self.INVALID       # Current altitude of the gps
            
        self.gps_thread = AGPS3mechanism()  # Instantiate AGPS3 Mechanisms
        
        self.gps_thread.stream_data() 
        self.gps_thread.run_thread()


    def set_target_uid(self, target_uid):
        """
        Used by SimGPS
        """
        pass


    def set_client(self, client):
        """
        Used by SimGPS
        """
        pass


    def read_gps(self):
        """
        Responsible for reading the current gps measurements and populating the 
        object class values accordingly
        """
        self.set_latitude()
        self.set_longitude()
        self.set_altitude()

        return


    def get_latitude(self):
        """
        Getter used to retrieve the saved latitude
        """
        return self.latitude


    def set_latitude(self):
        """
        Setter used to get the current gps latitude reading and set the class latitude variable
        """
        latitude = self.gps_thread.data_stream.lat

        if latitude == 'n/a':
            latitude = self.INVALID

        self.latitude = latitude

        return


    def stop_client(self):
        pass


    def get_longitude(self):
        """
        Getter used to retrieve the saved longitude
        """
        return self.longitude


    def set_longitude(self):
        """
        Setter used to get the current gps longitude reading and set the class longitude variable
        """
        longitude = self.gps_thread.data_stream.lon

        if longitude == 'n/a':
            longitude = self.INVALID

        self.longitude = longitude

        return


    def get_altitude(self):
        """
        Getter used to retrieve the saved altitude
        """
        return self.altitude


    def set_altitude(self):
        """
        Setter used to get the current gps altitude reading and set the class altitude variable
        """
        altitude = self.gps_thread.data_stream.alt

        if altitude == 'n/a':
            altitude = self.INVALID

        self.altitude = altitude

        return
