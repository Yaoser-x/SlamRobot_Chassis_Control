/* Exercise the actual embedded page script with a deterministic DOM and clock. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const sketch = fs.readFileSync(path.join(__dirname, '..', 'F407_ESP12F.ino'), 'utf8');
const html = sketch.match(/HTML_PAGE\[\] PROGMEM = R"raw\(\r?\n([\s\S]*?)\r?\n\)raw";/)[1];
const source = html.match(/<script>([\s\S]*?)<\/script>/)[1];
let now = 1000;
const messages = [];
const timers = [];
const nodes = new Map();
const windowEvents = {};
const documentEvents = {};
function node(id) {
  if (!nodes.has(id)) nodes.set(id, {
    textContent: '', style: {}, dataset: {}, events: {}, value: '60',
    classList: {add() {}, remove() {}, toggle() {}},
    addEventListener(name, callback) { this.events[name] = callback; },
    getBoundingClientRect() { return {left: 0, top: 0, width: 200, height: 200}; },
    setPointerCapture() {},
  });
  return nodes.get(id);
}
class Socket {
  constructor() { this.readyState = 1; }
  send(message) { messages.push({...JSON.parse(message), at: now}); }
  close() { this.readyState = 3; }
}
const document = {hidden: false, getElementById: node,
  addEventListener(name, callback) { documentEvents[name] = callback; }};
const context = vm.createContext({document, window: {
  addEventListener(name, callback) { windowEvents[name] = callback; }},
  location: {hostname: 'robot.invalid'}, WebSocket: Socket, Date: {now: () => now},
  setTimeout() {}, setInterval(callback, period) { timers.push({callback, period, next: now + period}); },
});
vm.runInContext(source, context);
function advance(ms) {
  const end = now + ms;
  while (true) {
    const next = Math.min(...timers.map(timer => timer.next));
    if (next > end) break;
    now = next;
    for (const timer of timers) if (timer.next === next) { timer.next += timer.period; timer.callback(); }
  }
  now = end;
}
function press() { node('joy').events.pointerdown({pointerId: 1, clientX: 100, clientY: 26}); }
function velocities() { return messages.filter(message => message.cmd === 'vel' && message.lx !== 0); }
context.setRole('owner');
press();
advance(10000);
const held = velocities();
assert(held.length >= 200, 'fixed held joystick must keep submitting fresh velocity');
for (let index = 1; index < held.length; ++index) {
  assert(held[index].at - held[index - 1].at <= 50, 'held velocity must refresh below the 500 ms deadman');
  assert.equal(held[index].lx, held[0].lx, 'a fixed gesture preserves its requested speed');
}
node('joy').events.pointerup();
assert.equal(messages.at(-1).cmd, 'stop');
let count = velocities().length;
advance(600);
assert.equal(velocities().length, count, 'release cannot keep the old movement alive');
press(); windowEvents.blur(); count = velocities().length; advance(600);
assert.equal(velocities().length, count, 'window blur cancels movement');
press(); document.hidden = true; documentEvents.visibilitychange();
count = velocities().length; advance(600);
assert.equal(velocities().length, count, 'hidden page cancels movement');
document.hidden = false;
press(); context.setRole('readonly'); context.setRole('owner');
count = velocities().length; advance(600);
assert.equal(velocities().length, count, 'owner recovery needs a new gesture');
console.log('PASS: embedded ESP held joystick, release, blur, hidden page and owner recovery');
