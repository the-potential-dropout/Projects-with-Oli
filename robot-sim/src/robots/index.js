// Every robot type the simulator knows about. Adding a new robot (a rover, a
// bigger arm...) means writing a class with the same shape as Hexapod
// (spawn, update(dt, cmd, game), damage, center, servoFrame, attachments)
// and listing it here.
import { Hexapod } from './hexapod.js';

export const ROBOTS = [
  {
    id: 'hexapod',
    name: 'Hexapod',
    blurb: '6 legs, 18 servos, tripod gait. Swappable top turret and front sensor.',
    available: true,
    create: (opts) => new Hexapod(opts),
  },
  {
    id: 'rover',
    name: 'Rover',
    blurb: '6-wheel rocker-bogie rover. Coming soon.',
    available: false,
  },
  {
    id: 'big-arm',
    name: 'Big Robotic Arm',
    blurb: '6-axis desktop arm on a fixed base. Coming soon.',
    available: false,
  },
];

export const robotById = (id) => ROBOTS.find((r) => r.id === id);
