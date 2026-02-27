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

# ==============================================================================
# PROCESS DESCRIPTION:
# THIS PROCESS MANAGES THE CAMERA HARDWARE AND THE ENTIRE VISION PIPELINE.
# IT INTEGRATES PERCEPTION (LANE) AND OBJECT DETECTION (SIGNS) TO MINIMIZE
# LATENCY BY SHARING THE RAW CV2 MAT IMAGE VIA SHARED RAM.
# 
# THREADS:
#   - threadCamera: Captures frames and stores them in shared_container['frame'].
#   - threadLane: Processes the shared frame for Stanley Control (e_y, theta_e).
#   - threadSigns: Processes the shared frame with YOLO for Traffic Signs.
#
# SHARED RESOURCES:
#   - shared_container: Dictionary {'frame': np_array} for zero-latency transfer.
# ==============================================================================

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from cv2 import meanShift
from src.templates.workerprocess import WorkerProcess
from src.hardware.camera.threads.threadCamera import threadCamera
from src.perception.Perception.threads.threadLane import threadLane
from src.hardware.camera.threads.threadSigns import threadSigns
from src.statemachine.stateMachine import StateMachine
from src.statemachine.systemMode import SystemMode
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StateChange


class processCamera(WorkerProcess):
    """This process handle camera.\n
    Args:
            queueList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
            logging (logging object): Made for debugging.
            debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        # Internal container to share the OpenCV frame between threads without Gateway overhead
        self.shared_container = {'frame': None}
        self.stateChangeSubscriber = messageHandlerSubscriber(self.queuesList, StateChange, "lastOnly", True)

        super(processCamera, self).__init__(self.queuesList, ready_event)

    # ================================ STATE CHANGE HANDLER ========================================
    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value["camera"]["process"]

            if modeDict["enabled"] == True:
                self.resume_threads()
            elif modeDict["enabled"] == False:
                self.pause_threads()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        # 1. Hardware Thread: Captures the frame and puts it in self.shared_container
        camTh = threadCamera(
            self.queuesList, self.logging, self.debugging, self.shared_container
        )
        self.threads.append(camTh)

        # 2. Perception Thread: Calculates Stanley errors (Lateral/Heading)
        laneTh = threadLane(
            self.queuesList, self.logging, self.debugging, self.shared_container
        )
        self.threads.append(laneTh)

        # 3. Object Detection Thread: Uses YOLO for signs and obstacles
        signTh = threadSigns(
            self.queuesList, self.logging, self.debugging, self.shared_container
        )
        self.threads.append(signTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processCamera.py
if __name__ == "__main__":
    from multiprocessing import Queue, Event
    import time
    import logging
    import cv2
    import base64
    import numpy as np

    allProcesses = list()

    debugg = True

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    logger = logging.getLogger()

    process = processCamera(queueList, logger, debugg)

    process.daemon = True
    process.start()

    time.sleep(4)
    if debugg:
        logger.warning("getting")
    img = {"msgValue": 1}
    while not isinstance(img["msgValue"], str):
        img = queueList["General"].get()
    
    msg_value = img["msgValue"]
    if isinstance(msg_value, str):
        image_data = base64.b64decode(msg_value)
    else:
        raise ValueError("Expected string for base64 decoding")
    img = np.frombuffer(image_data, dtype=np.uint8)
    image = cv2.imdecode(img, cv2.IMREAD_COLOR)
    if debugg:
        logger.warning("got")
    cv2.imwrite("test.jpg", image)
    process.stop()
