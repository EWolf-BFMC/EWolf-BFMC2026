# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import inspect
from multiprocessing import Pipe
from typing import List, Union

class messageHandlerSubscriber: 
    """
    Enhanced Subscriber Class to handle multiple message types.
    Updated for EWolf-BFMC2026 to support concurrent monitoring of 
    Control and State messages.
    """
        
    def __init__(self, queuesList, message, deliveryMode="fifo", subscribe=False):
        self._queuesList = queuesList
        # Store messages as a list even if it's just one
        self._message = message if isinstance(message, list) else [message]
        self._deliveryMode = str.lower(deliveryMode)
        self._pipeRecv, self._pipeSend = Pipe(duplex=False)
        
        # Buffer to store messages sorted by type
        self._buffer = {} 
        
        frame = inspect.currentframe().f_back
        if 'self' in frame.f_locals:
            self._receiver = frame.f_locals['self'].__class__.__name__
        else:
            self._receiver = frame.f_globals.get('__name__', None)
        
        if subscribe == True:
            self.subscribe()

        if self._deliveryMode not in ["fifo", "lastonly"]:
            self._deliveryMode = "fifo"

    def subscribe(self):
        """Subscribes to all messages in the list."""
        for msg in self._message:
            self._queuesList["Config"].put({
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": msg.Owner.value,
                "msgID": msg.msgID.value,
                "To": {"receiver": self._receiver, "pipe": self._pipeSend},
            })

    def unsubscribe(self):
        """Unsubscribes from all messages in the list."""
        for msg in self._message:
            self._queuesList["Config"].put({
                "Subscribe/Unsubscribe": "unsubscribe",
                "Owner": msg.Owner.value,
                "msgID": msg.msgID.value,
                "To": {"receiver": self._receiver}
            })

    def get_message(self, message_type):
        """
        Custom helper for threadLaneFollower. 
        Filters the incoming pipe for a specific message type.
        """
        # 1. Pull everything from the pipe into our local buffer
        while self._pipeRecv.poll():
            msg_data = self._pipeRecv.recv()
            # The key is the class name (e.g., 'StanleyControl')
            m_type = type(msg_data["value"]).__name__
            self._buffer[m_type] = msg_data

        # 2. Check if we have the requested type in the buffer
        target_name = message_type.__name__ if hasattr(message_type, '__name__') else str(message_type)
        if target_name in self._buffer:
            return self._buffer.pop(target_name)
        return None

    def receive(self):
        """Standard receive method."""
        if not self._pipeRecv.poll(): return None
        return self.receive_with_block()
        
    def receive_with_block(self):
        """Processes the pipe data and returns the value."""
        message = self._pipeRecv.recv()
        if self._deliveryMode == "lastonly":
            while (self._pipeRecv.poll()):
                message = self._pipeRecv.recv()
        return message["value"]

    def is_data_in_pipe(self):
        return self._pipeRecv.poll()

    def __del__(self): 
        self._pipeRecv.close()
        self._pipeSend.close()
