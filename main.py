# Location: /home/ewolf/EWolf-BFMC2026/main.py
import multiprocessing
import sys
import time
import os
import psutil
from multiprocessing import Queue, Event
import logging
from src.utils.bigPrintMessages import BigPrint
from src.gateway.processGateway import processGateway
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StateChange
from src.statemachine.stateMachine import StateMachine
from src.data.Artificial_Vision.processArtificial_Vision import processArtificial_Vision
from src.data.Controller.processController import processController

if __name__ == '__main__':
    # Force 'fork' start method for stability on Linux/Raspberry Pi [cite: 86]
    multiprocessing.set_start_method('fork', force=True)

    print(BigPrint.PLEASE_WAIT.value)
    
    # Corrected dictionary structure for inter-process communication 
    queueList = {
        "Critical": Queue(), 
        "Warning": Queue(), 
        "General": Queue(),
        "Config": Queue(), 
        "Log": Queue()
    }
    
    logging = logging.getLogger()
    
    # Initialize state machine and subscriber
    stateChangeSubscriber = messageHandlerSubscriber(queueList, StateChange, "lastOnly", True)
    StateMachine.initialize_shared_state(queueList)

    # Start system processes
    pGateway = processGateway(queueList, logging)
    pGateway.start()

    # Dashboard is commented out to avoid Flask pickling errors [cite: 104]
    # pDashboard.start() 

    # Vision Process readiness event
    Artificial_Vision_ready = Event()
    pVision = processArtificial_Vision(queueList, logging, Artificial_Vision_ready)
    pVision.daemon = True
    pVision.start()

    # Controller Process readiness event
    Controller_ready = Event()
    pController = processController(queueList, logging, Controller_ready)
    pController.daemon = True
    pController.start()

    # Serial Handler for hardware connection
    pSerial = processSerialHandler(queueList, logging)
    pSerial.daemon = True
    pSerial.start()

    # Maintenance loop
    blocker = Event()
    try:
        Artificial_Vision_ready.wait()
        Controller_ready.wait() 
        StateMachine.initialize_starting_mode()
        print(BigPrint.C4_BOMB.value)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[EWolf] Safety Shutdown initiated...")
        pController.stop(); pVision.stop(); pSerial.stop(); pGateway.stop()