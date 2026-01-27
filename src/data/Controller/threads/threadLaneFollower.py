import numpy as np
from src.templates.threadwithstop import ThreadWithStop
# Agregamos 'State' a las importaciones
from src.utils.messages.allMessages import StanleyControl, SpeedMotor, SteerMotor, State

class threadLaneFollower(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        # Stanley Controller Parameters
        self.k = 0.55 
        self.ks = 0.1 
        self.max_steer = 0.436
        
        # Variable interna para rastrear el modo de conducción
        self.current_mode = "Manual" 
        
        self.subscribe()
        super(threadLaneFollower, self).__init__()

    def subscribe(self):
        """Suscribirse a la visión y al estado del sistema"""
        self.messageHandlerSubscriber.subscribe_to_message(StanleyControl)
        self.messageHandlerSubscriber.subscribe_to_message(State)

    def thread_work(self):
        """Loop principal con filtro de modo automático"""
        while not self.is_stopped():
            # 1. ACTUALIZAR MODO (Viene del Dashboard/StateMachine)
            state_msg = self.messageHandlerSubscriber.get_message(State)
            if state_msg:
                self.current_mode = state_msg['value']

            # 2. SOLO ACTUAR SI ESTAMOS EN MODO AUTOMÁTICO
            if self.current_mode == "Automated":
                # Recibir datos de la visión de Ángel
                vision_message = self.messageHandlerSubscriber.get_message(StanleyControl)
                
                if vision_message:
                    data = vision_message['Value']
                    e_y = data['e_y']
                    theta_e = data['theta_e']
                    v = data['speed']
                    
                    # Cálculo de Stanley: 
                    # $$\delta = \theta_e + \arctan\left(\frac{k \cdot e_y}{v + k_s}\right)$$
                    steering_adj = np.arctan2(self.k * e_y, v + self.ks)
                    steering_angle = theta_e + steering_adj
                    
                    # Saturación
                    steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
                    
                    # Enviar comandos al hardware
                    self.send_commands(v, steering_angle)
            else:
                # Si no es automático, nos quedamos quietos para no chocar con el control manual
                # Opcional: Podrías limpiar la cola de mensajes aquí
                pass

    def send_commands(self, speed, steer):
        """Envío de comandos de velocidad y giro"""
        self.messageHandlerSender.send_message(SpeedMotor, {"value": speed})
        self.messageHandlerSender.send_message(SteerMotor, {"value": steer})
        
        if self.debugging:
            self.logging.info(f"Modo: {self.current_mode} | V: {speed:.2f} | Delta: {steer:.2f}")

    def state_change_handler(self):
        pass