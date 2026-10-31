import assert from 'node:assert/strict';
import { test } from 'node:test';
import { parseCSV, getIMUReadings, getMAGReadings } from '../src/csvParser.ts';

const columns = '$COL,IMU,time,wx,wy,wz,ax,ay,az,temperature';
const imu = '$IMU,10,-16.296,6.042,20.019,0.00683,0.32812,0.63574,33.96';
const mag = '$MAG,10,0.259,0.073,-0.438,33.0';
const quaternionColumns = `${columns},q_w,q_x,q_y,q_z`;

test('legacy recordings retain sensor values, metadata, statistics and no device quaternion', () => {
  const dataset = parseCSV([
    '$FLYS,1', '$VAR,FIRMWARE_VER,legacy', '$VAR,DEVICE_ID,device',
    '$VAR,SESSION_ID,session', columns, '$DATA', mag, imu,
    '$BARO,10,97936.72,32.44', '$HUM,10,45,33',
    '$TIME,10,123,2400', '$VBAT,10,4.1', imu.replace(',10,', ',11,')
  ].join('\r\n'));
  assert.equal(dataset.firmwareVersion, 'legacy');
  assert.equal(dataset.deviceId, 'device');
  assert.equal(dataset.sessionId, 'session');
  assert.equal(dataset.imuCount, 2);
  assert.equal(dataset.magCount, 1);
  assert.equal(dataset.baroCount, 1);
  assert.equal(dataset.readings.length, 5);
  assert.equal(dataset.startTime, 10);
  assert.equal(dataset.endTime, 11);
  assert.equal(dataset.duration, 1);
  assert.equal(dataset.imuRate, 2);
  assert.deepEqual(getIMUReadings(dataset)[0], {
    type: 'IMU', timestamp: 10, wx: -16.296, wy: 6.042, wz: 20.019,
    ax: 0.00683, ay: 0.32812, az: 0.63574, temperature: 33.96
  });
  assert.deepEqual(getMAGReadings(dataset)[0], {
    type: 'MAG', timestamp: 10, x: 0.259, y: 0.073, z: -0.438, temperature: 33
  });
});

test('new recordings preserve timestamped firmware quaternions without normalization or sign changes', () => {
  const dataset = parseCSV([
    quaternionColumns, '$DATA', mag, `${imu},-0.6396,0.0840,0.0224,0.7637`
  ].join('\n'));
  const reading = getIMUReadings(dataset)[0];
  assert.equal(reading.timestamp, 10);
  assert.deepEqual(reading.deviceQuaternion, { w: -0.6396, x: 0.084, y: 0.0224, z: 0.7637 });
  assert.equal(reading.temperature, 33.96);
  assert.equal(reading.wx, -16.296);
  assert.equal(dataset.magCount, 1);
});

test('quaternion component order follows the IMU column declaration', () => {
  const dataset = parseCSV([
    `${columns},extra,q_z,q_y,q_w,q_x`, '$DATA', `${imu},123,0.4,0.3,0.8,0.2`
  ].join('\n'));
  assert.deepEqual(getIMUReadings(dataset)[0].deviceQuaternion, { w: 0.8, x: 0.2, y: 0.3, z: 0.4 });
});

test('headerless IMU layouts support both legacy and appended quaternion records', () => {
  const dataset = parseCSV(['$DATA', imu, `${imu},1,0,0,0`].join('\n'));
  const readings = getIMUReadings(dataset);
  assert.equal(readings[0].deviceQuaternion, undefined);
  assert.deepEqual(readings[1].deviceQuaternion, { w: 1, x: 0, y: 0, z: 0 });
});

test('unrelated extra columns are not mistaken for quaternions', () => {
  const dataset = parseCSV([`${columns},status`, '$DATA', `${imu},123`].join('\n'));
  assert.equal(getIMUReadings(dataset)[0].deviceQuaternion, undefined);
});

test('partial quaternion declarations report the header line', () => {
  assert.throws(() => parseCSV(`${columns},q_w,q_x\n$DATA\n${imu},1,0`),
    /Incomplete device quaternion columns at CSV line 1/);
});

for (const suffix of ['', ',1,0,0', ',1,,0,0', ',NaN,0,0,0', ',Infinity,0,0,0', ',1bad,0,0,0', ',0,0,0,0']) {
  test(`invalid or missing declared quaternion is reported: ${JSON.stringify(suffix)}`, () => {
    assert.throws(() => parseCSV(`${quaternionColumns}\n$DATA\n${imu}${suffix}`),
      /Invalid device quaternion at CSV line 3/);
  });
}
