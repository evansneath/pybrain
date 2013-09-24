__author__ = 'Frank Sehnke, sehnke@in.tum.de'

#############################################################################################################
# UDP Connection classes                                                                                    #
#                                                                                                           #
# UDPServer waits till at least one client is connected.                                                    #
# It then sends a list to the connected clients (can also be a list of scipy arrays!)                       #
# There can connect several clients to the server but the same data is sent to all clients.                 #
# Options for the constructor are the server IP and the starting port (2 adjacent ports will be used)       #
#                                                                                                           #
# UDPClient trys to connect to a UDPServer till the connection is established.                              #
# The client then recives data from the server and parses it into an list of the original shape.            #
# Options for the cunstructor are server-, client IP and the starting port (2 adjacent ports will be used)  #
#                                                                                                           #
# Requirements: sockets and scipy.                                                                          #
# Example: FlexCubeEnvironment and FlexCubeRenderer (env sends data to renderer for OpenGL output)          #
#                                                                                                           #
#############################################################################################################

import math
import socket

# The server class
class UDPServer(object):
    def __init__(self, ip='127.0.0.1', port='21560', buf='16384', verbose=False):
        self.verbose = verbose

        #Socket settings
        self.host = ip
        self.inPort = eval(port) + 1
        self.outPort = eval(port)
        self.buf = eval(buf) #16384
        self.addr = (self.host, self.inPort)

        #Create socket and bind to address
        self.UDPInSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.UDPInSock.bind(self.addr)

        #Client lists
        self.clients = 0
        self.cIP = []
        self.addrList = []
        self.UDPOutSockList = []
        if self.verbose:
            print 'UDP server listening on port %d' % (self.inPort)

        return

    def addClient(self, cIP):
        # Adding a client to the list
        self.cIP.append(cIP)
        self.addrList.append((cIP, self.outPort))
        self.UDPOutSockList.append(socket.socket(socket.AF_INET,
                socket.SOCK_DGRAM))
        self.clients += 1

        if self.verbose:
            print 'UDP client connected at %s' % (cIP)

        return

    def listen(self):
        # Listen for clients
        if self.clients < 1:
            self.UDPInSock.settimeout(10)
            try:
                cIP = self.UDPInSock.recv(self.buf)
                self.addClient(cIP)
            except:
                pass
        else:
            # At least one client has to send a sign of life during 2 seconds
            self.UDPInSock.settimeout(2)
            try:
                cIP = self.UDPInSock.recv(self.buf)
                newClient = True
                for i in self.cIP:
                    if cIP == i:
                        newClient = False
                        break
                #Adding new client
                if newClient:
                    self.addClient(cIP)
            except:
                if self.verbose:
                    print 'All UDP clients disconnected'

                self.clients = 0
                self.cIP = []
                self.addrList = []
                self.UDPOutSockList = []

                if self.verbose:
                    print 'UPD server listening on port %d' % (self.inPort)

        return

    # Sending the actual data too all clients
    def send(self, arrayList):
        data = repr(arrayList)
        data_len = len(data)

        # Message format:
        # [<more_data_flag (1 byte)>, <application_data (x bytes)>]

        for i, sock in enumerate(self.UDPOutSockList):
            # Get the max socket packet size
            max_size = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)

            # Calculate the number of packets necessary for the data size
            num_packets = int(math.ceil(float(data_len)/(float(max_size)-1)))

            for msg_num in range(num_packets):
                # This single byte value prepends each message. The client
                # looks for this when deciding whether there is more data
                # that was sent
                more_data = 1 if msg_num < num_packets - 1 else 0

                # Parse the data to send in this packet
                cur_pos = msg_num * (max_size - 1)
                packet = repr(more_data) + data[cur_pos:cur_pos+max_size-1]

                # Ship it off!
                sock.sendto(packet, self.addrList[i])

        return


class UDPClient(object):
    # The client class
    def __init__(self, servIP='127.0.0.1', ownIP='127.0.0.1', port="21560",
            buf='16384', verbose=False):
        self.verbose = verbose

        #UDP Sttings
        self.host = servIP
        self.inPort = eval(port)
        self.outPort = eval(port) + 1
        self.inAddr = (ownIP, self.inPort)
        self.outAddr = (self.host, self.outPort)
        self.ownIP = ownIP
        self.buf = eval(buf) #16384

        # Create sockets
        self.createSockets()

        self.listen_attempts = 0
        self.timeout = 0.5 # [s]

        return

    # Listen for data from server
    def listen(self):
        data_obj = None

        # Send alive signal (own IP adress)
        self.UDPOutSock.sendto(self.ownIP, self.outAddr)

        # if there is no data from Server for 10 seconds server is down
        self.UDPInSock.settimeout(self.timeout)

        self.listen_attempts += 1

        data = ''

        # Get all data associated with packets sent over. Each packet has
        # the following format:
        # [<more_data_flag (1 byte)>, <application_data (x bytes)>]

        # If the more_data_flag is set, this means that the application
        # data was sent in more than one packet and must be reconstructed
        while True:
            # Receive data over UDP socket connection
            try:
                packet = self.UDPInSock.recv(self.buf)
            except:
                # Display this message
                if self.verbose and (self.listen_attempts * self.timeout) % 10 == 0:
                    print 'No connection to UDP server'
                raise

            # Extract all application data
            data += packet[1:]

            # There are no more packets associated with this data blob
            if not int(packet[0]):
                break

        try:
            data_obj = eval(data)
        except:
            if self.verbose:
                print ('Unsupported data format received from UDP server at %s'
                    % (self.outAddr))
            raise

        return data_obj

    def createSockets(self):
        # Creating the sockets
        self.UDPOutSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.UDPOutSock.sendto(self.ownIP, self.outAddr)
        self.UDPInSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.UDPInSock.bind(self.inAddr)

        return


if __name__ == '__main__':
    pass
