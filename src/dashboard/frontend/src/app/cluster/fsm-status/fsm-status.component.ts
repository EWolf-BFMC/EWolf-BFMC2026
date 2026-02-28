import { Component } from '@angular/core';
import { Subscription } from 'rxjs';
import { CommonModule } from '@angular/common';
import { WebSocketService } from '../../webSocket/web-socket.service';

@Component({
  selector: 'app-fsm-status',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './fsm-status.component.html',
  styleUrl: './fsm-status.component.css'
})
export class FsmStatusComponent {
  public fsmState: string = 'IDLE';
  public sign: string = 'NONE';
  public obstacleZone: string = 'CLEAR';

  private fsmSubscription: Subscription | undefined;

  constructor(private webSocketService: WebSocketService) {}

  ngOnInit() {
    this.fsmSubscription = this.webSocketService.receiveFsmStatus().subscribe(
      (message) => {
        this.fsmState = message.value?.state ?? 'IDLE';
        this.sign = message.value?.sign ?? 'NONE';
        this.obstacleZone = message.value?.obstacle_zone ?? 'CLEAR';
      },
      (error) => {
        console.error('Error receiving FSM status:', error);
      }
    );
  }

  ngOnDestroy() {
    if (this.fsmSubscription) {
      this.fsmSubscription.unsubscribe();
    }
  }

  // BehaviorState enum names from allStates.py:
  // IDLE, LANE_FOLLOWING, HIGHWAY_DRIVING, DECELERATING,
  // STOP_ACTION, EMERGENCY_BRAKE, PARKING_MANEUVER, ROUNDABOUT, INTERSECTION
  getStateColor(): string {
    switch (this.fsmState) {
      case 'LANE_FOLLOWING':
      case 'HIGHWAY_DRIVING':
        return '#2ecc71';          // green  — active driving
      case 'EMERGENCY_BRAKE':
        return '#e74c3c';          // red    — immediate stop
      case 'DECELERATING':
      case 'STOP_ACTION':
        return '#e67e22';          // orange — slowing / halted
      case 'PARKING_MANEUVER':
      case 'INTERSECTION':
      case 'ROUNDABOUT':
        return '#3498db';          // blue   — planned maneuver
      default:                     // IDLE
        return '#7f8c8d';          // grey
    }
  }

  // ObstacleZone enum names from allStates.py: CLEAR, WARNING, DANGER
  getZoneColor(): string {
    switch (this.obstacleZone) {
      case 'DANGER':  return '#e74c3c';  // red
      case 'WARNING': return '#e67e22';  // orange
      default:        return '#2ecc71';  // green — CLEAR
    }
  }

  // SignType enum names from allStates.py:
  // TRAFFIC_LIGHT, STOP, PARKING, CROSSWALK, PRIORITY,
  // HIGHWAY_ENTRY, HIGHWAY_EXIT, ONE_WAY, ROUNDABOUT, NO_ENTRY
  getSignColor(): string {
    switch (this.sign) {
      case 'STOP':
      case 'NO_ENTRY':
        return '#e74c3c';   // red   — prohibition / mandatory stop
      case 'TRAFFIC_LIGHT':
      case 'CROSSWALK':
      case 'PARKING':
        return '#e67e22';   // orange — caution
      case 'PRIORITY':
      case 'HIGHWAY_ENTRY':
      case 'HIGHWAY_EXIT':
      case 'ONE_WAY':
      case 'ROUNDABOUT':
        return '#3498db';   // blue  — informational
      case 'NONE':
      default:
        return '#7f8c8d';   // grey
    }
  }
}
