import datetime
import socket
import struct
import sys
import threading
import time

from . import crc
from . import logger
from . import event
from . import state
from . import error
from . import video_stream
from . utils import *
from . protocol import *
from . import dispatcher


class VideoFrame(object):
    def __init__(self, seq_id):
        self.seq_id = seq_id
        self.packets = dict()
        self.last_sub_id = None
        self.frame_t = None
        self.complete = False

    def store_packet(self, sub_id, payload):
        if sub_id in self.packets:
            raise IndexError('Repeated sub_id: %d' % sub_id)
        sub_last = False
        if sub_id >= 128:
            sub_last = True
            sub_id -= 128
        if sub_last:
            self.last_sub_id = sub_id
        self.packets[sub_id] = payload
        self.frame_t = time.time()
        self.complete = (self.last_sub_id is not None and len(self.packets) == self.last_sub_id+1)

    def get_frame(self, pad_missing_packets=False):
        if len(self.packets) <= 0:
            raise ValueError('Empty frame')
        elif self.last_sub_id is None:
            raise ValueError('Incomplete frame without last packet')

        mid_packet_len = 0
        for sub_id in self.packets:
            if sub_id < self.last_sub_id:
                if len(self.packets[sub_id]) > mid_packet_len:
                    mid_packet_len = len(self.packets[sub_id])

        if isinstance(self.packets[self.last_sub_id], str):
            frame = ''
            for sub_id in range(self.last_sub_id+1):
                if sub_id in self.packets:
                    frame += self.packets[sub_id]
                elif pad_missing_packets:
                    frame += chr(0)*mid_packet_len
        else: # bytes
            frame = b''
            for sub_id in range(self.last_sub_id+1):
                if sub_id in self.packets:
                    frame += self.packets[sub_id]
                elif pad_missing_packets:
                    frame += bytes(bytearray(mid_packet_len))
        return frame


class Tello(object):
    EVENT_CONNECTED = event.Event('connected')
    EVENT_WIFI = event.Event('wifi')
    EVENT_LIGHT = event.Event('light')
    EVENT_FLIGHT_DATA = event.Event('flight_data')
    EVENT_LOG = event.Event('log')
    EVENT_TIME = event.Event('time')

    # data = (payload_bytes, consec_incr_seq_id, frame_secs)
    EVENT_VIDEO_FRAME = event.Event('video_frame')

    # data = bytes([seq_id_byte, sub_id_byte, payload_bytes])
    EVENT_VIDEO_PACKET = event.Event('video_packet')
    EVENT_DISCONNECTED = event.Event('disconnected')

    # internal events
    __EVENT_CONN_REQ = event.Event('conn_req')
    __EVENT_CONN_ACK = event.Event('conn_ack')
    __EVENT_TIMEOUT = event.Event('timeout')
    __EVENT_QUIT_REQ = event.Event('quit_req')

    STATE_DISCONNECTED = state.State('disconnected')
    STATE_CONNECTING = state.State('connecting')
    STATE_CONNECTED = state.State('connected')
    STATE_QUIT = state.State('quit')

    LOG_ERROR = logger.LOG_ERROR
    LOG_WARN = logger.LOG_WARN
    LOG_INFO = logger.LOG_INFO
    LOG_DEBUG = logger.LOG_DEBUG
    LOG_ALL = logger.LOG_ALL

    def __init__(self,
                 local_cmd_client_port=9000,
                 local_vid_server_port=6038,
                 tello_ip='192.168.10.1',
                 tello_cmd_server_port=8889,
                 log=None):
        self.tello_addr = (tello_ip, tello_cmd_server_port)
        self.debug = False
        self.pkt_seq_num = 0x01e4
        self.local_cmd_client_port = local_cmd_client_port
        self.local_vid_server_port = local_vid_server_port
        self.udpsize = 2048  # observed max packet size: 1460
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.fast_mode = False
        self.sock = None
        self.state = self.STATE_DISCONNECTED
        self.lock = threading.Lock()
        self.connected = threading.Event()
        self.video_enabled = False
        self.log = log or logger.Logger('Tello')
        self.exposure = 0
        self.video_encoder_rate = VIDRATE_AUTO
        self.video_req_sps_hz = 4.0
        self.video_stream = None

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.local_cmd_client_port))
        self.sock.settimeout(2.0)

        dispatcher.connect(self.__state_machine, dispatcher.signal.All)
        self.recv_thread = threading.Thread(target=self.__recv_thread)
        self.video_thread = threading.Thread(target=self.__video_thread)
        self.recv_thread.start()
        self.video_thread.start()

    def set_loglevel(self, level):
        """
        Set_loglevel controls the output messages. Valid levels are
        LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG and LOG_ALL.
        """
        self.log.set_level(level)

    def get_video_stream(self):
        """
        Get_video_stream is used to prepare buffer object which receive video data from the drone.
        """
        newly_created = False
        self.lock.acquire()
        self.log.info('get video stream')
        try:
            if self.video_stream is None:
                self.video_stream = video_stream.VideoStream(self)
                newly_created = True
            res = self.video_stream
        finally:
            self.lock.release()
        if newly_created:
            self.__send_exposure()
            self.__send_video_encoder_rate()
            self.start_video()

        return res

    def connect(self):
        """Connect is used to send the initial connection request to the drone."""
        self.__publish(event=self.__EVENT_CONN_REQ)

    def wait_for_connection(self, timeout=None):
        """Wait_for_connection will block until the connection is established."""
        if not self.connected.wait(timeout):
            raise error.TelloError('timeout')

    def __send_conn_req(self):
        port_bytes = struct.pack('<H', self.local_vid_server_port)
        if isinstance(port_bytes, str):
            buf = 'conn_req:'+port_bytes
        else:
            buf = b'conn_req:'+port_bytes
        self.log.info('send connection request (cmd="%sx%02xx%02x")' %
                      (str(buf[:-2]), byte(port_bytes[0]), byte(port_bytes[1])))
        return self.send_packet(Packet(buf))

    def subscribe(self, signal, handler):
        """Subscribe a event such as EVENT_CONNECTED, EVENT_FLIGHT_DATA, EVENT_VIDEO_FRAME and so on."""
        dispatcher.connect(handler, signal)

    def __publish(self, event, data=None, **args):
        args.update({'data': data})
        if 'signal' in args:
            del args['signal']
        if 'sender' in args:
            del args['sender']
        self.log.debug('publish signal=%s, args=%s' % (event, args))
        dispatcher.send(event, sender=self, **args)

    def takeoff(self):
        """Takeoff tells the drones to liftoff and start flying."""
        self.log.info('takeoff (cmd=0x%02x seq=0x%04x)' %
                      (TAKEOFF_CMD, self.pkt_seq_num))
        pkt = Packet(TAKEOFF_CMD)
        pkt.fixup()
        return self.send_packet(pkt)

    def throw_takeoff(self):
        """Throw_takeoff starts motors and expects to be thrown to takeoff."""
        self.log.info('throw_takeoff (cmd=0x%02x seq=0x%04x)' %
                      (THROW_TAKEOFF_CMD, self.pkt_seq_num))
        pkt = Packet(THROW_TAKEOFF_CMD)
        pkt.fixup()
        return self.send_packet(pkt)

    def land(self, stop_landing=False):
        """Land tells the drone to come in for landing."""
        self.log.info('land (cmd=0x%02x seq=0x%04x)' %
                      (LAND_CMD, self.pkt_seq_num))
        pkt = Packet(LAND_CMD)
        pkt.add_byte(int(stop_landing))
        pkt.fixup()
        return self.send_packet(pkt)

    def palm_land(self, stop_landing=False):
        """Palm_land tells the drone to cut motors near flat surface below."""
        self.log.info('palm_land (cmd=0x%02x seq=0x%04x)' %
                      (PALM_LAND_CMD, self.pkt_seq_num))
        pkt = Packet(PALM_LAND_CMD)
        pkt.add_byte(int(stop_landing))
        pkt.fixup()
        return self.send_packet(pkt)

    def flattrim(self):
        """FlatTrim re-calibrates IMU to be horizontal."""
        self.log.info('flattrim (cmd=0x%02x seq=0x%04x)' %
                      (FLATTRIM_CMD, self.pkt_seq_num))
        pkt = Packet(FLATTRIM_CMD)
        pkt.fixup()
        return self.send_packet(pkt)

    def quit(self):
        """Quit stops the internal threads."""
        self.log.info('quit')
        self.__publish(event=self.__EVENT_QUIT_REQ)
        self.recv_thread.join()
        self.video_thread.join()

    def __send_time_command(self):
        self.log.info('send_time (cmd=0x%02x seq=0x%04x)' %
                      (TIME_CMD, self.pkt_seq_num))
        pkt = Packet(TIME_CMD, 0x50)
        pkt.add_byte(0)
        pkt.add_time()
        pkt.fixup()
        return self.send_packet(pkt)

    def send_req_video_sps_pps(self):
        """Manually request drone to send an I-frame info (SPS/PPS) for video stream."""
        pkt = Packet(VIDEO_REQ_SPS_PPS_CMD, 0x60)
        pkt.fixup()
        return self.send_packet(pkt)

    def start_video(self):
        """Start_video tells the drone to send start info (SPS/PPS) for video stream."""
        self.log.info('start video (cmd=0x%02x seq=0x%04x)' %
                      (VIDEO_REQ_SPS_PPS_CMD, self.pkt_seq_num))
        self.video_enabled = True
        self.__send_exposure()
        self.__send_video_encoder_rate()
        return self.send_req_video_sps_pps()

    def set_exposure(self, level):
        """Set_exposure sets the drone camera exposure level. Valid levels are 0, 1, and 2."""
        if level < 0 or 2 < level:
            raise error.TelloError('Invalid exposure level')
        self.log.info('set exposure (cmd=0x%02x seq=0x%04x)' %
                      (EXPOSURE_CMD, self.pkt_seq_num))
        self.exposure = level
        return self.__send_exposure()

    def __send_exposure(self):
        pkt = Packet(EXPOSURE_CMD, 0x48)
        pkt.add_byte(self.exposure)
        pkt.fixup()
        return self.send_packet(pkt)

    def set_video_encoder_rate(self, rate):
        """Set_video_encoder_rate sets the drone video encoder rate."""
        self.log.info('set video encoder rate (cmd=0x%02x seq=%04x)' %
                      (VIDEO_ENCODER_RATE_CMD, self.pkt_seq_num))
        self.video_encoder_rate = rate
        return self.__send_video_encoder_rate()

    def set_video_req_sps_hz(self, hz):
        """Internally sends a SPS/PPS request at desired rate; <0: disable."""
        if hz < 0:
            hz = 0.
        self.video_req_sps_hz = hz

    def __send_video_encoder_rate(self):
        pkt = Packet(VIDEO_ENCODER_RATE_CMD, 0x68)
        pkt.add_byte(self.video_encoder_rate)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip(self, flip_dir=FLIP_FRONT):
        """flip tells the drone to perform a flip given a protocol.FLIP_XYZ direction"""
        self.log.info('flip (cmd=0x%02x seq=0x%04x)' %
                      (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        if flip_dir < 0 or flip_dir >= FLIP_MAX_INT:
            flip_dir = FLIP_FRONT
        pkt.add_byte(flip_dir)
        pkt.fixup()
        return self.send_packet(pkt)

    def __fix_range(self, val, min=-1.0, max=1.0):
        if val < min:
            val = min
        elif val > max:
            val = max
        return val

    def set_vspeed(self, vspeed):
        """
        Set_vspeed controls the vertical up and down motion of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value means upward)
        """
        if self.left_y != self.__fix_range(vspeed):
            self.log.info('set_vspeed(val=%4.2f)' % vspeed)
        self.left_y = self.__fix_range(vspeed)

    def set_yaw(self, yaw):
        """
        Set_yaw controls the left and right rotation of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone turn to the right)
        """
        if self.left_x != self.__fix_range(yaw):
            self.log.info('set_yaw(val=%4.2f)' % yaw)
        self.left_x = self.__fix_range(yaw)

    def set_pitch(self, pitch):
        """
        Set_pitch controls the forward and backward tilt of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone move forward)
        """
        if self.right_y != self.__fix_range(pitch):
            self.log.info('set_pitch(val=%4.2f)' % pitch)
        self.right_y = self.__fix_range(pitch)

    def set_roll(self, roll):
        """
        Set_roll controls the the side to side tilt of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone move to the right)
        """
        if self.right_x != self.__fix_range(roll):
            self.log.info('set_roll(val=%4.2f)' % roll)
        self.right_x = self.__fix_range(roll)

    def set_fast_mode(self, enabled):
        self.fast_mode = enabled

    def reset_cmd_vel(self):
        self.left_x = 0.
        self.left_y = 0.
        self.right_x = 0.
        self.right_y = 0.
        self.fast_mode = False

    def __send_stick_command(self):
        pkt = Packet(STICK_CMD, 0x60)

        axis1 = int(1024 + 660.0 * self.right_x) & 0x7ff
        axis2 = int(1024 + 660.0 * self.right_y) & 0x7ff
        axis3 = int(1024 + 660.0 * self.left_y) & 0x7ff
        axis4 = int(1024 + 660.0 * self.left_x) & 0x7ff
        axis5 = int(self.fast_mode) & 0x01
        self.log.debug("stick command: fast=%d yaw=%4d vrt=%4d pit=%4d rol=%4d" %
                       (axis5, axis4, axis3, axis2, axis1))

        '''
        11 bits (-1024 ~ +1023) x 4 axis = 44 bits
        fast_mode takes 1 bit
        44+1 bits will be packed in to 6 bytes (48 bits)

         axis5      axis4      axis3      axis2      axis1
             |          |          |          |          |
                 4         3         2         1         0
        98765432109876543210987654321098765432109876543210
         |       |       |       |       |       |       |
             byte5   byte4   byte3   byte2   byte1   byte0
        '''
        packed = axis1 | (axis2 << 11) | (
            axis3 << 22) | (axis4 << 33) | (axis5 << 44)
        packed_bytes = struct.pack('<Q', packed)
        pkt.add_byte(byte(packed_bytes[0]))
        pkt.add_byte(byte(packed_bytes[1]))
        pkt.add_byte(byte(packed_bytes[2]))
        pkt.add_byte(byte(packed_bytes[3]))
        pkt.add_byte(byte(packed_bytes[4]))
        pkt.add_byte(byte(packed_bytes[5]))
        pkt.add_time()
        pkt.fixup()
        self.log.debug("stick command: %s" %
                       byte_to_hexstring(pkt.get_buffer()))
        return self.send_packet(pkt)

    def send_packet(self, pkt):
        """Send_packet is used to send a command packet to the drone."""
        try:
            cmd = pkt.get_buffer()
            self.sock.sendto(cmd, self.tello_addr)
            self.log.debug("send_packet: %s" % byte_to_hexstring(cmd))
        except socket.error as err:
            if self.state == self.STATE_CONNECTED:
                self.log.error("send_packet: %s" % str(err))
            else:
                self.log.info("send_packet: %s" % str(err))
            return False

        return True

    def __process_packet(self, data):
        if isinstance(data, str):
            data = bytearray([x for x in data])

        if str(data[0:9]) == 'conn_ack:' or data[0:9] == b'conn_ack:':
            self.log.info('connected. (vid_port=x%2xx%2x)' %
                          (data[9], data[10]))
            self.log.debug('    %s' % byte_to_hexstring(data))
            if self.video_enabled:
                self.__send_exposure()
                self.__send_video_encoder_rate()
                self.send_req_video_sps_pps()
            self.__publish(self.__EVENT_CONN_ACK, data)

            return True

        if data[0] != START_OF_PACKET:
            self.log.info('start of packet != %02x (%02x) (ignored)' %
                          (START_OF_PACKET, data[0]))
            self.log.info('    %s' % byte_to_hexstring(data))
            self.log.info('    %s' % str(map(chr, data))[1:-1])
            return False

        pkt = Packet(data)
        cmd = int16(data[5], data[6])
        if cmd == LOG_MSG:
            self.log.debug("recv: log: %s" % byte_to_hexstring(data[9:]))
            self.__publish(event=self.EVENT_LOG, data=data[9:])
        elif cmd == WIFI_MSG:
            self.log.debug("recv: wifi: %s" % byte_to_hexstring(data[9:]))
            self.__publish(event=self.EVENT_WIFI, data=data[9:])
        elif cmd == LIGHT_MSG:
            self.log.debug("recv: light: %s" % byte_to_hexstring(data[9:]))
            self.__publish(event=self.EVENT_LIGHT, data=data[9:])
        elif cmd == FLIGHT_MSG:
            flight_data = FlightData(data[9:])
            self.log.debug("recv: flight data: %s" % str(flight_data))
            self.__publish(event=self.EVENT_FLIGHT_DATA, data=flight_data)
        elif cmd == TIME_CMD:
            self.log.debug("recv: time data: %s" % byte_to_hexstring(data))
            self.__publish(event=self.EVENT_TIME, data=data[7:9])
        elif cmd in (TAKEOFF_CMD, THROW_TAKEOFF_CMD, LAND_CMD, PALM_LAND_CMD, FLATTRIM_CMD, VIDEO_REQ_SPS_PPS_CMD, VIDEO_ENCODER_RATE_CMD):
            self.log.info("recv: ack: cmd=0x%02x seq=0x%04x %s" %
                          (int16(data[5], data[6]), int16(data[7], data[8]), byte_to_hexstring(data)))
        else:
            self.log.info('unknown packet: %s' % byte_to_hexstring(data))
            return False

        return True

    def __state_machine(self, event, sender, data, **args):
        self.lock.acquire()
        cur_state = self.state
        event_connected = False
        event_disconnected = False
        self.log.debug('event %s in state %s' % (str(event), str(self.state)))

        if self.state == self.STATE_DISCONNECTED:
            if event == self.__EVENT_CONN_REQ:
                self.__send_conn_req()
                self.state = self.STATE_CONNECTING
            elif event == self.__EVENT_QUIT_REQ:
                self.state = self.STATE_QUIT
                event_disconnected = True
                self.video_enabled = False

        elif self.state == self.STATE_CONNECTING:
            if event == self.__EVENT_CONN_ACK:
                self.state = self.STATE_CONNECTED
                event_connected = True
                # send time
                self.__send_time_command()
            elif event == self.__EVENT_TIMEOUT:
                self.__send_conn_req()
            elif event == self.__EVENT_QUIT_REQ:
                self.state = self.STATE_QUIT

        elif self.state == self.STATE_CONNECTED:
            if event == self.__EVENT_TIMEOUT:
                self.__send_conn_req()
                self.state = self.STATE_CONNECTING
                event_disconnected = True
                self.video_enabled = False
            elif event == self.__EVENT_QUIT_REQ:
                self.state = self.STATE_QUIT
                event_disconnected = True
                self.video_enabled = False

        elif self.state == self.STATE_QUIT:
            pass

        if cur_state != self.state:
            self.log.info('state transit %s -> %s' % (cur_state, self.state))
        self.lock.release()

        if event_connected:
            self.__publish(event=self.EVENT_CONNECTED, **args)
            self.connected.set()
        if event_disconnected:
            self.__publish(event=self.EVENT_DISCONNECTED, **args)
            self.connected.clear()

    def __recv_thread(self):
        sock = self.sock

        while self.state != self.STATE_QUIT:

            if self.state == self.STATE_CONNECTED:
                self.__send_stick_command()  # ignore errors

            try:
                data, server = sock.recvfrom(self.udpsize)
                self.log.debug("recv: %s" % byte_to_hexstring(data))
                self.__process_packet(data)
            except socket.timeout as ex:
                if self.state == self.STATE_CONNECTED:
                    self.log.error('recv: timeout')
                self.__publish(event=self.__EVENT_TIMEOUT)
            except Exception as ex:
                self.log.error('recv: %s' % str(ex))
                show_exception(ex)

        self.log.info('exit from the recv thread.')

    def __video_thread(self, pub_partial_packets=True, pad_missing_packets=True):
        self.log.info('start video thread')
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', self.local_vid_server_port))
        sock.settimeout(5.0)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                        self.udpsize)  # observed max packet size: 1460
        self.log.debug('video receive buffer size = %d bytes' %
                       sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))

        prev_frame = None
        curr_frame = None
        last_req_sps_t = None
        seq_block_count = 0
        while self.state != self.STATE_QUIT:
            if not self.video_enabled:
                time.sleep(0.01)
                continue
            try:
                # receive and publish packet
                data, server = sock.recvfrom(self.udpsize)
                now = time.time()
                self.__publish(event=self.EVENT_VIDEO_PACKET, data=data)

                # parse packet
                seq_id = byte(data[0])
                sub_id = byte(data[1])
                packet = data[2:]
                sub_last = (sub_id >= 128)  # MSB asserted

                # notify packet
                self.log.debug("packet recv: %3u %3u %s, %4d bytes" %
                               (seq_id, sub_id-128 if sub_last else sub_id,
                                'L' if sub_last else ' ',
                                len(data)))

                # associate packet to frame
                drop_frame = None
                if curr_frame is None: # First ever packet
                    curr_frame = VideoFrame(seq_id)
                    curr_frame.store_packet(sub_id, packet)
                elif seq_id == curr_frame.seq_id: # current frame
                    curr_frame.store_packet(sub_id, packet)
                elif seq_id > curr_frame.seq_id: # next frame
                    seq_id_diff = seq_id - curr_frame.seq_id
                    if seq_id_diff == 1:
                        drop_frame = prev_frame
                        prev_frame = curr_frame
                    else: # > 1
                        drop_frame = curr_frame
                        prev_frame = None
                    curr_frame = VideoFrame(seq_id)
                    curr_frame.store_packet(sub_id, packet)
                elif curr_frame.seq_id >= 254 and seq_id <= 1: # next frame with wrap-around
                    seq_block_count += 1
                    seq_id_diff = 256 + seq_id - curr_frame.seq_id
                    if seq_id_diff == 1:
                        drop_frame = prev_frame
                        prev_frame = curr_frame
                    else: # > 1
                        drop_frame = curr_frame
                        prev_frame = None
                    curr_frame = VideoFrame(seq_id)
                    curr_frame.store_packet(sub_id, packet)
                elif prev_frame is not None and prev_frame.seq_id == seq_id: # prev frame
                    prev_frame.store_packet(sub_id, packet)
                else: # before prev frame
                    pass # drop packet

                # publish frames when available
                if pub_partial_packets and drop_frame is not None and drop_frame.last_sub_id is not None:
                    frame = drop_frame.get_frame(pad_missing_packets)
                    frame_seq_id = drop_frame.seq_id
                    frame_contig_seq_id = seq_block_count*256+frame_seq_id
                    if frame_seq_id > curr_frame.seq_id:
                        frame_contig_seq_id -= 1 # before seq_id wrap-around
                    frame_t = drop_frame.frame_t

                    self.__publish(event=self.EVENT_VIDEO_FRAME,
                                   data=(frame, frame_contig_seq_id, frame_t))
                    self.log.debug("frame recv: %3u, (%5u), %6d bytes (outdated)" %
                                   (frame_seq_id, frame_contig_seq_id, len(frame)))

                if prev_frame is not None and (prev_frame.complete or (pub_partial_packets and curr_frame.complete and prev_frame.last_sub_id is not None)):
                    frame = prev_frame.get_frame(pad_missing_packets)
                    frame_seq_id = prev_frame.seq_id
                    frame_contig_seq_id = seq_block_count*256+frame_seq_id
                    if frame_seq_id > curr_frame.seq_id:
                        frame_contig_seq_id -= 1 # before seq_id wrap-around
                    frame_t = prev_frame.frame_t

                    self.__publish(event=self.EVENT_VIDEO_FRAME,
                                   data=(frame, frame_contig_seq_id, frame_t))
                    self.log.debug("frame recv: %3u, (%5u), %6d bytes (outdated)" %
                                   (frame_seq_id, frame_contig_seq_id, len(frame)))

                    prev_frame = None
                
                if curr_frame is not None and curr_frame.complete:
                    frame = curr_frame.get_frame(pad_missing_packets)
                    frame_seq_id = curr_frame.seq_id
                    frame_contig_seq_id = seq_block_count*256+frame_seq_id
                    frame_t = curr_frame.frame_t

                    self.__publish(event=self.EVENT_VIDEO_FRAME,
                                   data=(frame, frame_contig_seq_id, frame_t))
                    self.log.debug("frame recv: %3u, (%5u), %6d bytes (outdated)" %
                                   (frame_seq_id, frame_contig_seq_id, len(frame)))

                    curr_frame = None
                    prev_frame = None # do not publish prev frame out of order

                # Regularly request SPS/PPS data to repair broken video stream
                if last_req_sps_t is None:
                    last_req_sps_t = now
                dur = now - last_req_sps_t
                if self.video_req_sps_hz > 0 and dur > 1.0/self.video_req_sps_hz:
                    last_req_sps_t = now
                    self.send_req_video_sps_pps()

            except socket.timeout as ex:
                self.log.error('video recv: timeout')
                data = None
            except Exception as ex:
                self.log.error('video recv: %s' % str(ex))
                show_exception(ex)

        self.log.info('exit from the video thread.')
