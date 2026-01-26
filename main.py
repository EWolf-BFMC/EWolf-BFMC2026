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
#
# To start the project: 
#
#       chmod +x setup.sh
#       ./setup.sh
#       cd src/dashboard/frontend
#       npm start
#       answer "y" to the popup
#       close the frontend (CTRL + C)
#       cd ../..
#
# ===================================== GENERAL IMPORTS ==================================

# ===================================== GENERAL IMPORTS ==================================
import sys
import time
import os
import psutil
from multiprocessing import Queue, Event
import logging
import logging.handlers

# Pin to CPU cores 0–3 for better performance allocation
available_cores = list(range(psutil.cpu_count()))
psutil.Process(os.getpid()).cpu_affinity(available_cores)

sys.path.append(".")
from src.utils.bigPrintMessages import BigPrint
from src.utils.outputWriters import QueueWriter, MultiWriter

# ===================================== PROCESS IMPORTS ==================================
from src.gateway.processGateway import processGateway
from src.dashboard.processDashboard import processDashboard
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StateChange
from src.statemachine.stateMachine import StateMachine
from src.statemachine.systemMode import SystemMode

# ------ New component imports starts here ------#
from src.data.Artificial_Vision.processArtificial_Vision import processArtificial_Vision
# ------ New component imports ends here ------#

# ===================================== SHUTDOWN HELPER ====================================
def shutdown_process(process, timeout=1):
    """Gracefully shutdown a process to avoid memory leaks."""
    process.join(timeout)
    if process.is_alive():
        process.terminate()
        process.join(timeout)
        if process.is_alive():
            process.kill()
    print(f"The process {process} stopped")

# ===================================== MAIN EXECUTION ==================================
if __name__ == '__main__':
    import multiprocessing
    # Forcing 'spawn' is critical for Windows and Python 3.14 stability.
    multiprocessing.set_start_method('spawn', force=True) 

    print(BigPrint.PLEASE_WAIT.value)
    
    # Initialize basic communication queues
    queueList = {
        "Critical": Queue(), "Warning": Queue(), "General": Queue(),
        "Config": Queue(), "Log": Queue(),
    }
    
    logging = logging.getLogger()
    
    # Setup shared system state
    stateChangeSubscriber = messageHandlerSubscriber(queueList, StateChange, "lastOnly", True)
    StateMachine.initialize_shared_state(queueList)

    # Start Gateway: The central message hub
    pGateway = processGateway(queueList, logging)
    pGateway.start()

   # --- STARTING THE DASHBOARD SAFELY ---
    dashboard_ready = Event()
    
    # We pass the class and the arguments
    # This avoid Windows to try to "picklear" thr Flask object
    pDashboard = processDashboard(
        queueList, 
        logging, 
        dashboard_ready, 
        debugging=False
    )
    pDashboard.daemon = True
    pDashboard.start()

    # --- STARTING ARTIFICIAL VISION ---
    Artificial_Vision_ready = Event()
    pVision = processArtificial_Vision(
        queueList, 
        logging, 
        Artificial_Vision_ready, 
        debugging=False
    )
    pVision.daemon = True
    pVision.start()
    # --- STARTING SERIAL HANDLER ---
    pSerial = processSerialHandler(queueList, logging, debugging=False)
    pSerial.daemon = True
    pSerial.start()

    # --- STAYING ALIVE ---
    blocker = Event()
    try:
        # Wait for signals from the Dashboard and Vision logic
        dashboard_ready.wait()
        Artificial_Vision_ready.wait()

        StateMachine.initialize_starting_mode()

        time.sleep(2)
        print(BigPrint.C4_BOMB.value)
        print(BigPrint.PRESS_CTRL_C.value)

        while True:
            # Stay alive and listen for state changes (e.g., HighwayZone detection)
            message = stateChangeSubscriber.receive()
            blocker.wait(0.1)

    except KeyboardInterrupt:
        print("\nShutting down system safely...\n")
        pVision.stop()
        pDashboard.stop()
        pSerial.stop()
        pGateway.stop()