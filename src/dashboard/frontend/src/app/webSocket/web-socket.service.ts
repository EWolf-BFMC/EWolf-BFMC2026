// Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { Observable, Subject } from 'rxjs';

@Injectable({ providedIn: 'root' })
export class WebSocketService {

  private socket: Socket;

  private connectionStatusSubject =
    new Subject<'connected' | 'disconnected' | 'error'>();

  connectionStatus$ = this.connectionStatusSubject.asObservable();

  constructor() {
    this.socket = io('http://ewolf.local:5005', {
      reconnection: true,
      reconnectionAttempts: Infinity,
      reconnectionDelay: 1000,
      timeout: 20000
    });

    this.socket.on('connect', () =>
      this.connectionStatusSubject.next('connected')
    );

    this.socket.on('disconnect', () =>
      this.connectionStatusSubject.next('disconnected')
    );

    this.socket.on('connect_error', () =>
      this.connectionStatusSubject.next('error')
    );
  }

  /* ================= CORE ================= */

  private fromEvent<T>(event: string): Observable<T> {
    return new Observable(observer => {
      this.socket.on(event, data => observer.next(data));
    });
  }

  sendMessageToFlask(message: any) {
    this.socket.emit('message', message);
  }

  SaveTable(message: any) {
    this.socket.emit('save', message);
  }

  LoadTable(message: any) {
    this.socket.emit('load', message);
  }

  disconnectSocket() {
    this.socket.disconnect();
  }

  reconnect() {
    this.socket.connect();
  }

  isConnected(): boolean {
    return this.socket.connected;
  }

  /* ========== COMPATIBILITY LAYER ========== */

  receiveSessionAccess() { return this.fromEvent<any>('session_access'); }
  receiveHeartbeat() { return this.fromEvent<any>('heartbeat'); }
  receiveHeartbeatDisconnect() { return this.fromEvent<any>('heartbeat_disconnect'); }
  receiveCurrentSerialConnectionState() { return this.fromEvent<any>('current_serial_connection_state'); }
  receiveMemoryUsage() { return this.fromEvent<any>('memory_channel'); }
  receiveCpuUsage() { return this.fromEvent<any>('cpu_channel'); }
  receiveResourceMonitor() { return this.fromEvent<any>('ResourceMonitor'); }
  receiveBatteryLevel() { return this.fromEvent<any>('BatteryLvl'); }
  receiveInstantConsumption() { return this.fromEvent<any>('InstantConsumption'); }
  receiveEnableButton() { return this.fromEvent<any>('EnableButton'); }
  receiveCamera() { return this.fromEvent<any>('serialCamera'); }
  receiveLocation() { return this.fromEvent<any>('Location'); }
  receiveSemaphores() { return this.fromEvent<any>('Semaphores'); }
  receiveCurrentSpeed() { return this.fromEvent<any>('CurrentSpeed'); }
  receiveCurrentSteer() { return this.fromEvent<any>('CurrentSteer'); }
  receiveSerialConnectionState() { return this.fromEvent<any>('SerialConnectionState'); }
  receiveWarningSignal() { return this.fromEvent<any>('WarningSignal'); }
  receiveStateChange() { return this.fromEvent<any>('StateChange'); }
  receiveSteerLimits() { return this.fromEvent<any>('SteeringLimits'); }
  receiveCalibrationData() { return this.fromEvent<any>('Calibration'); }
  receiveConsoleLog() { return this.fromEvent<any>('console_log'); }
  receiveLoadTable() { return this.fromEvent<any>('loadBack'); }

  receiveUnhandledEvents() {
    return this.fromEvent<{ channel: string; data: any }>('__any__');
  }
}

