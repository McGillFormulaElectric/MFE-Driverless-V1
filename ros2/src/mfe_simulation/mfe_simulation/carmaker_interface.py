import socket
import logging
import numpy as np

from rclpy.node import Node

class Quantity():
    FLOAT = 1.0
    INT = 1

    def __init__(self, name, q_type, command=False):
        self.name = name
        self.type = q_type
        self.command = command
        self.data = None

        if command == True:
            self.read_msg = self.name + "\r"
        else:
            self.read_msg = "expr {$Qu(" + self.name + ")}\r"

class CarMaker():
    status_dic = {-1: "Preprocessing", -2: "Idle", -3: "Postprocessing", -4: "Model Check",
                  -5: "Driver Adaption", -6: "FATAL ERROR", -7: "Waiting for License",
                  -8: "Simulation paused", -10: "Starting application", -11: "Simulink Initialization"}

    def __init__(self, ip, port, log_level=logging.INFO):
        self.logger = logging.getLogger("pycarmaker")
        self.logger.setLevel(log_level)

        self.ip = ip
        self.port = port

        self.socket = None
        self.quantities = []

        self.logger.debug("pycarmaker init completed")

    def connect(self):
        # Open the TCP / IP port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the CM TCP / IP port 16660 on localhost
        self.socket.connect((self.ip, self.port))
        self.logger.info("TCP socket connected")

    def subscribe(self, quantity):
        self.quantities.append(quantity)

        if quantity.command == True:
            self.logger.info("Subscribe for command " + quantity.name + ": OK")
            return

        # Create message to subscribe to quantities with all quantities
        # previous subscribed
        msg = ""
        for q in self.quantities:
            if msg == "":
                msg = q.name
            else:
                msg += " " + q.name
        msg = "QuantSubscribe {" + msg + "}\r"
        self.logger.debug(msg)

        if (self.socket == None):
            self.logger.error("Not connected")
            return

        self.socket.send(msg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        self.logger.info("Subscribe for quantity " +
                         quantity.name + ": " + str(rsp))
        # TODO Handle error

    def read(self):
        # By IPG recommendation, read one quantity at a time.
        for quantity in self.quantities:
            self.socket.send(quantity.read_msg.encode())
            str_rx = self.socket.recv(300).decode()
            rx_list = str_rx.split("\r\n\r\n")
            self.logger.debug(rx_list)

            if (len(rx_list) != 2):
                self.logger.error("Wrong read")
                return

            if type(quantity.type) == type(Quantity.FLOAT):
                quantity.data = float(rx_list[0][1:])
            elif type(quantity.type) == type(Quantity.INT):
                quantity.data = int(rx_list[0][1:])
            else:
                self.logger.error("Unknown type")

    def DVA_write(self, quantity, value, duration=-1, mode="Abs"):
        """ set the value of a variable using DVAWrite <Name> <Value> <Duration> <Mode> ...
        Parameters
        ----------
        quant : Quantity
            Quantity to set.
        value : Float
            New quantity value
        duration : Float
            Duration in milliseconds
        mode : string
            One of Abs, Off, Fac, AbsRamp, ...; default Absolute Value
        """

        msg = "DVAWrite " + quantity.name + " " + \
            str(value)+" "+str(duration)+" "+mode+"\r"
        self.socket.send(msg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        self.logger.info("Write quantity " +
                         quantity.name + ": " + str(rsp))

    def DVA_release(self):
        """ Call this method when you are done using DVA """
        self.send("DVAReleaseQuants\r")

    def send(self, msg):
        """ send the giving message to CarMaker
        Paramters
        ---------
        msg : string
            a string contains the message ending with \ r
        """
        self.socket.send(msg.encode())
        return self.socket.recv(200)

class VDS:
    def __init__(self, ip="localhost", port=2210, log_level=logging.INFO):
        self.logger = logging.getLogger("pycarmaker")
        self.logger.setLevel(log_level)
        self.ip = ip
        self.port = port
        self.socket = None
        self.cameras = []
        self.connected = False

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
        data = self.socket.recv(64)
        if(data.decode().find("*IPGMovie") != -1):
            self.logger.info("IPG Movie is Connected...")
            self.connected = True

    def read(self):
        """
        Read the streamed images.

        Returns
        -------
        img : numpy array
            a numpy array representing the image

        """
        if not self.connected:
            self.logger.error("Connect first by calling .connect()")
            return
        # Get Image header and fill data
        data = self.socket.recv(64)
        splitdata = data.decode().split(" ")
        imgtype = splitdata[2]
        img_size = splitdata[4]
        data_len = int(splitdata[5])
        imag_h = int(img_size.split('x')[1])
        image_w = int(img_size.split('x')[0])
        lastdata = b''
        size = 0
        while(size != data_len):
            data = self.socket.recv(1024)
            try:
                strdata = data.decode()
                if strdata[0] == '*' and strdata[1] == 'V':
                    splitdata = data.decode().split(" ")
                    imgtype = splitdata[2]
                    img_size = splitdata[4]
                    data_len = int(splitdata[5])
                    imag_h = int(img_size.split('x')[1])
                    image_w = int(img_size.split('x')[0])
                    lastdata = b''
                    size = 0
                    continue
            except:
                pass
            lastdata += data
            size = np.frombuffer(lastdata, dtype=np.uint8).size
        datalist = np.frombuffer(lastdata, dtype=np.uint8)
        if(imgtype == "rgb"):
            img = datalist.reshape((imag_h, image_w, 3))
        elif(imgtype == "grey"):
            img = datalist.reshape((imag_h, image_w))
        else:
            self.logger.error("rgb and gray are supported for now")

        return img
    
class CameraRSDR(VDS): #Video data stream
    def __init__(self, ip="localhost", port=2210, log_level=logging.INFO):
        super()._init__(ip, port, log_level)

    def read(self):
        """
        Read the streamed images.

        Returns
        -------
        img : numpy array
            a numpy array representing the image

        """
        if not self.connected:
            self.logger.error("Connect first by calling .connect()")
            return
        # Get Image header and fill data
        data = self.socket.recv(64)
        splitdata = data.decode().split(" ")
        imgtype = splitdata[2]
        img_size = splitdata[4]
        data_len = int(splitdata[5])
        imag_h = int(img_size.split('x')[1])
        image_w = int(img_size.split('x')[0])
        lastdata = b''
        size = 0
        while(size != data_len):
            data = self.socket.recv(1024)
            try:
                strdata = data.decode()
                if strdata[0] == '*' and strdata[1] == 'V':
                    splitdata = data.decode().split(" ")
                    imgtype = splitdata[2]
                    img_size = splitdata[4]
                    data_len = int(splitdata[5])
                    imag_h = int(img_size.split('x')[1])
                    image_w = int(img_size.split('x')[0])
                    lastdata = b''
                    size = 0
                    continue
            except:
                pass
            lastdata += data
            size = np.frombuffer(lastdata, dtype=np.uint8).size
        datalist = np.frombuffer(lastdata, dtype=np.uint8)
        if(imgtype == "rgb"):
            img = datalist.reshape((imag_h, image_w, 3))
        elif(imgtype == "grey"):
            img = datalist.reshape((imag_h, image_w))
        else:
            self.logger.error("rgb and gray are supported for now")

        return img
    
class LidarRSDR(VDS): # Point cloud data stream
    def __init__(self, ip="localhost", port=2210, log_level=logging.INFO):
        super().__init__(ip, port, log_level)

    def read(self):
        """
        Read the streamed 3D point cloud data from CarMaker.

        Returns
        -------
        points : numpy.ndarray
        Nx3 array of XYZ coordinates (float32)
        """
        if not self.connected:
            self.logger.error("Connect first by calling .connect()")
            return None

        try:
            # Read the 64-byte header from the LiDAR stream
            header = self.socket.recv(64)
            splitdata = header.decode().split(" ")

            if len(splitdata) < 6:
                    self.logger.error(f"Unexpected header format: {header}")
                    return None

            datatype = splitdata[2]  # Should be something like "xyz"
            data_len = int(splitdata[5])  # Total number of bytes expected

            if datatype != "xyz":
                self.logger.error(f"Unsupported LiDAR datatype: {datatype}")
                return None

            # Read the full point cloud binary payload
            raw_data = b''
            while len(raw_data) < data_len:
                chunk = self.socket.recv(min(4096, data_len - len(raw_data)))
                raw_data += chunk

            # Convert binary data to float32 and reshape to Nx3 points
            num_floats = data_len // 4  # float32 = 4 bytes
            if num_floats % 3 != 0:
                self.logger.error("Received data length is not divisible by 3 floats (x, y, z)")
                return None

            points = np.frombuffer(raw_data, dtype=np.float32).reshape((-1, 3))
            return points

        except Exception as e:
            self.logger.error(f"Error while reading LiDAR data: {e}")
        
        return None
