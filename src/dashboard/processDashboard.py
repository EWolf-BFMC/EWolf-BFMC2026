# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import queue
import psutil
import json
import inspect
import eventlet
import os
import time

from flask import Flask, request
from flask_socketio import SocketIO
from flask_cors import CORS
from enum import Enum

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.templates.workerprocess import WorkerProcess
from src.utils.messages.allMessages import Semaphores
from src.statemachine.stateMachine import StateMachine
from src.dashboard.components.calibration import Calibration
from src.dashboard.components.ip_manger import IpManager

import src.utils.messages.allMessages as allMessages


class processDashboard(WorkerProcess):
    """
    This process handles the dashboard interactions.
    Fixed to support supersecurepassword validation.
    """
    
    def __init__(self, queueList, logging, ready_event=None, debugging = False):
        self.running = True
        self.queueList = queueList
        self.logger = logging
        self.debugging = debugging
        
        IpManager.replace_ip_in_file()
        self.stateMachine = StateMachine.get_instance()

        self.messages = {}
        self.sendMessages = {}
        self.messagesAndVals = {}

        self.memoryUsage = 0
        self.cpuCoreUsage = 0
        self.cpuTemperature = 0

        self.heartbeat_last_sent = time.time()
        self.heartbeat_retries = 0
        self.heartbeat_max_retries = 3
        self.heartbeat_time_between_heartbeats = 20 
        self.heartbeat_time_between_retries = 5 
        self.heartbeat_received = False

        self.sessionActive = False
        self.activeUser = None
        self.serialConnected = False

        self.table_state_file = self._get_table_state_path()

        self.app = Flask(__name__)
        # Fixed SocketIO initialization for compatibility
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading', 
                                 ping_timeout=60, ping_interval=5, allow_eio3=True, 
                                 allow_unsafe_werkzeug=True, logger=False, engineio_logger=False)
        CORS(self.app, supports_credentials=True)

        self.calibration = Calibration(self.queueList, self.socketio)

        self._initialize_messages()
        self._setup_websocket_handlers()
        self._start_background_tasks()

        super(processDashboard, self).__init__(self.queueList, ready_event)

    def _get_table_state_path(self):
        base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        return os.path.join(base_path, 'src', 'utils', 'table_state.json')

    def _initialize_messages(self):
        self.get_name_and_vals()
        self.messagesAndVals.pop("mainCamera", None)
        self.messagesAndVals.pop("Semaphores", None)
        self.subscribe()

    def _setup_websocket_handlers(self):
        self.socketio.on_event('message', self.handle_message)
        self.socketio.on_event('save', self.handle_save_table_state)
        self.socketio.on_event('load', self.handle_load_table_state)

    def _start_background_tasks(self):
        psutil.cpu_percent(interval=1, percpu=False)
        eventlet.spawn(self.update_hardware_data)
        eventlet.spawn(self.send_continuous_messages)
        eventlet.spawn(self.send_hardware_data_to_frontend)
        eventlet.spawn(self.send_heartbeat)
        eventlet.spawn(self.stream_console_logs)

    def stream_console_logs(self):
        log_queue = self.queueList.get("Log")
        if not log_queue: return
        while self.running:
            try:
                while not log_queue.empty():
                    msg = log_queue.get_nowait()
                    self.socketio.emit('console_log', {'data': msg})
                    eventlet.sleep(0)
                eventlet.sleep(0.1)
            except Exception: eventlet.sleep(1)

    def stop(self):
        super(processDashboard, self).stop()
        self.running = False

    def run(self):
        if self.ready_event: self.ready_event.set()
        self.socketio.run(self.app, host='0.0.0.0', port=5005)

    def subscribe(self):
        for name, enum in self.messagesAndVals.items():
            if enum["owner"] != "Dashboard":
                subscriber = messageHandlerSubscriber(self.queueList, enum["enum"], "lastOnly", True)
                self.messages[name] = {"obj": subscriber}
            else:
                sender = messageHandlerSender(self.queueList, enum["enum"])
                self.sendMessages[str(name)] = {"obj": sender}
        subscriber = messageHandlerSubscriber(self.queueList, Semaphores, "fifo", True)
        self.messages["Semaphores"] = {"obj": subscriber}

    def get_name_and_vals(self):
        classes = inspect.getmembers(allMessages, inspect.isclass)
        for name, cls in classes:
            if name != "Enum" and issubclass(cls, Enum):
                self.messagesAndVals[name] = {"enum": cls, "owner": cls.Owner.value}

    def send_message_to_brain(self, dataName, dataDict):
        if dataName in self.sendMessages:
            self.sendMessages[dataName]["obj"].send(dataDict.get("Value"))

    def handle_message(self, data):
        """Processes incoming data and validates 'supersecurepassword'."""
        if self.debugging:
            self.logger.info("Received message: " + str(data))

        try:
            # Use dict if already parsed, otherwise parse JSON
            dataDict = data if isinstance(data, dict) else json.loads(data)
            dataName = dataDict.get("Name")
            socketId = request.sid

            if dataName == "SessionAccess":
                if dataDict.get("Value") == "supersecurepassword":
                    self.handle_single_user_session(socketId)
                else:
                    self.socketio.emit('session_access', {'data': False}, room=socketId)
                return

            if self.sessionActive and self.activeUser != socketId:
                return

            if dataName == "Heartbeat":
                self.handle_heartbeat()
            elif dataName == "SessionEnd":
                self.handle_session_end(socketId)
            elif dataName == "DrivingMode":
                self.handle_driving_mode(dataDict)
            elif dataName == "Calibration":
                self.handle_calibration(dataDict, socketId)
            elif dataName == "GetCurrentSerialConnectionState":
                self.handle_get_current_serial_connection_state(socketId)
            else:
                self.send_message_to_brain(dataName, dataDict)

            self.socketio.emit('response', {'data': 'OK'}, room=socketId)
        except Exception as e:
            if self.debugging: self.logger.error(f"Error: {e}")

    def handle_heartbeat(self):
        self.heartbeat_retries = 0
        self.heartbeat_last_sent = time.time()
        self.heartbeat_received = True

    def handle_driving_mode(self, dataDict):
        self.stateMachine.request_mode(f"dashboard_{dataDict['Value']}_button")

    def handle_calibration(self, dataDict, socketId):
        self.calibration.handle_calibration_signal(dataDict, socketId)

    def handle_get_current_serial_connection_state(self, socketId):
        self.socketio.emit('current_serial_connection_state', {'data': self.serialConnected}, room=socketId)

    def handle_single_user_session(self, socketId):
        if not self.sessionActive:
            self.sessionActive = True
            self.activeUser = socketId
            print(f"Session access granted to {socketId}")
            self.socketio.emit('session_access', {'data': True}, room=socketId)
            self.send_message_to_brain("RequestSteerLimits", {"Value": True})
        elif self.activeUser == socketId:
            self.socketio.emit('session_access', {'data': True}, room=socketId)
        else:
            self.socketio.emit('session_access', {'data': False}, room=socketId)

    def handle_session_end(self, socketId):
        if self.sessionActive and self.activeUser == socketId:
            self.sessionActive = False
            self.activeUser = None

    def handle_save_table_state(self, data):
        try:
            dataDict = json.loads(data)
            os.makedirs(os.path.dirname(self.table_state_file), exist_ok=True)
            with open(self.table_state_file, 'w') as json_file:
                json.dump(dataDict, json_file, indent=4)
            self.socketio.emit('response', {'data': 'Saved'})
        except Exception: pass

    def handle_load_table_state(self, data):
        try:
            with open(self.table_state_file, 'r') as json_file:
                dataDict = json.load(json_file)
            self.socketio.emit('loadBack', {'data': dataDict})
        except Exception: pass

    def update_hardware_data(self):
        try:
            self.cpuCoreUsage = psutil.cpu_percent(interval=None, percpu=False)
            self.memoryUsage = psutil.virtual_memory().percent
            self.cpuTemperature = round(psutil.sensors_temperatures()['cpu_thermal'][0].current)
        except Exception: pass
        eventlet.spawn_after(1, self.update_hardware_data)

    def send_heartbeat(self):
        if not self.running: return
        if not self.heartbeat_received and self.sessionActive:
            self.heartbeat_retries += 1
            if self.heartbeat_retries < self.heartbeat_max_retries:
                self.socketio.emit('heartbeat', {'data': 'Heartbeat'})
            else:
                self.sessionActive = False
                self.activeUser = None
                self.heartbeat_retries = 0
            eventlet.spawn_after(self.heartbeat_time_between_retries, self.send_heartbeat)
        else:
            self.heartbeat_received = False
            eventlet.spawn_after(self.heartbeat_time_between_heartbeats, self.send_heartbeat)

    def send_continuous_messages(self):
        if not self.running: return
        for msg, subscriber in self.messages.items():
            resp = subscriber["obj"].receive()
            if resp is not None:
                if msg == "SerialConnectionState": self.serialConnected = resp
                self.socketio.emit(msg, {"value": resp})
        eventlet.spawn_after(0.1, self.send_continuous_messages)

    def send_hardware_data_to_frontend(self):
        if not self.running: return
        self.socketio.emit('memory_channel', {'data': self.memoryUsage})
        self.socketio.emit('cpu_channel', {'data': {'usage': self.cpuCoreUsage, 'temp': self.cpuTemperature}})
        eventlet.spawn_after(1.0, self.send_hardware_data_to_frontend)