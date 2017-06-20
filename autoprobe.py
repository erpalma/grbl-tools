#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import json
import re
import sys

from argparse import ArgumentParser
from datetime import datetime
from itertools import product
from numpy import linspace, where
from serial import Serial
from time import sleep, time


class Probe():

    def __init__(self, device, input_gcode, grid_spacing, feed_rate, overscan, min_z, max_z):
        self.ser = None
        self.device = device
        self.input_gcode = input_gcode
        self.grid_spacing = grid_spacing
        self.feed_rate = feed_rate
        self.overscan = overscan
        self.min_z = min_z
        self.max_z = max_z
        self.ser_timeout = 120
        self.fine_feed_probe = 1
        self.coarse_feed_probe = 40
        self.x_coords_re = re.compile(r'X\s*(-?[0-9]+(?:\.[0-9]+)?)')
        self.y_coords_re = re.compile(r'Y\s*(-?[0-9]+(?:\.[0-9]+)?)')
        self.mpos_re = re.compile(r'\|MPos:(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+)')
        self.probe_re = re.compile(r'\[PRB:(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+):([0-1])\]')

    def init_grbl(self):
        # open serial port and wait for welcome msg
        self.ser = Serial(self.device, 115200, timeout=self.ser_timeout)
        while self.ser.readline().strip() != "Grbl 1.1f ['$' for help]":
            pass
        self.ser.reset_input_buffer()
        # set millimeter mode
        self.send('G21')
        # set adbsolute coords
        self.send('G90')
        # reset work coords
        self.send('G92X0Y0Z0')
        # set local relative offset
        self.start_mpos = self.get_abs_pos()

    def send(self, data, newline=True):
        # open serial only on first send
        if self.ser is None:
            self.init_grbl()
        # wait for machine to be idle (not moving)
        while True:
            self.ser.write('?')
            if '<Idle|' in self.ser.readline():  # and 'WCO' in mpos:
                break
            sleep(.25)
        # send data and wait for answer
        self.ser.write(data + ('\n' if newline else ''))
        resp = self.ser.readline().strip()
        # parse and return responses
        if resp == 'ok':
            return True
        elif 'error:' in resp or 'ALARM:' in resp:
            raise Exception(resp)
        elif resp.startswith('['):
            out = [resp]
            while True:
                resp = self.ser.readline().strip()
                if resp.startswith('['):
                    out.append(resp)
                elif resp == 'ok':
                    return '\n'.join(out)
        return resp

    def get_rel_coord(self, coords):
        resp = {}
        for coord in 'xyz':
            if coord in coords:
                resp[coord] = - self.start_mpos[coord] + coords[coord]
        return resp

    def get_abs_pos(self):
        # wait for machine to be idle
        while True:
            mpos = self.send('?', newline=False)
            if '<Idle|' in mpos:
                break
            sleep(.25)
        mpos = map(float, self.mpos_re.findall(mpos)[0])
        return {'x': mpos[0], 'y': mpos[1], 'z': mpos[2]}

    def get_pos(self):
        # get current position in relative coords
        return self.get_rel_coord(self.get_abs_pos())

    def probe(self, min_z, feed_rate, retract=None, zero_coords=False):
        resp = self.send('G38.3 Z{:.5f} F{:.0f}'.format(min_z, feed_rate))
        resp = self.probe_re.findall(resp)[0]
        probe_point, probe_success = map(float, resp[:3]), bool(resp[-1])
        # zero out work coords
        if probe_success and zero_coords:
            # zero out work offset
            self.send('G92Z{:.5f}'.format(self.get_abs_pos()['z'] - probe_success))
            # go to effective zero since probe might have stopped after
            # the probe touchdown (due to deceleration)
            self.send('G01Z0F1')
            # set new local relative offset
            self.start_mpos = self.get_abs_pos()
        if retract is not None:
            self.send('G0Z{:.5f}'.format(retract))
        probe_point = {'x': probe_point[0], 'y': probe_point[1], 'z': 0. if zero_coords else probe_point[2]}
        return self.get_rel_coord(probe_point), probe_success

    def probe_origin(self):
        sys.stdout.write('\n[I] Zeroing Z in origin using coarse mode (F{:.0f})... '.format(self.coarse_feed_probe))
        sys.stdout.flush()

        # raise Z axis a bit to avoid potential alarm
        self.send('G0Z1')
        if not self.probe(-50, self.coarse_feed_probe, zero_coords=True)[1]:
            print('\n\n[E] Probe error!')
            sys.exit(1)
        self.send('G1Z.1F10')
        sys.stdout.write('Done.\n[I] Zeroing Z in origin using fine mode (F{:.0f})... '.format(self.fine_feed_probe))
        sys.stdout.flush()
        if not self.probe(-.4, self.fine_feed_probe, zero_coords=True)[1]:
            print('\n\n[E] Probe error!')
            sys.exit(1)
        print('Done.')

    def return_home(self):
        print('\n[I] Returning home. X0 Y0 Z0.5')
        self.send('G0Z5')
        self.send('G0X0Y0')
        self.send('G0Z.5')
        self.send('G1Z.2F10')

    def get_workspace_size(self):
        # get all X and Y coords in the gcode file
        X = map(float, self.x_coords_re.findall(self.input_gcode))
        Y = map(float, self.y_coords_re.findall(self.input_gcode))
        # find boundaries
        return min(X), max(X), min(Y), max(Y)

    def get_probe_coords(self):
        minx, maxx, miny, maxy = self.get_workspace_size()
        print('\n[I] Gcode area (WxH): {:.2f}mm x {:.2f}mm'.format(abs(maxx - minx), abs(maxy - miny)))
        if self.overscan != 0:
            minx, maxx = minx - self.overscan, maxx + self.overscan
            miny, maxy = miny - self.overscan, maxy + self.overscan
            print('[I] Probe area with overscan (WxH): {:.2f}mm x {:.2f}mm'.format(abs(maxx - minx), abs(maxy - miny)))

        x_steps = int(round(abs(maxx - minx) / self.grid_spacing)) + 1
        x_spacing = abs(maxx - minx) / (x_steps - 1)
        X = linspace(minx, maxx, x_steps)

        y_steps = int(round(abs(maxy - miny) / self.grid_spacing)) + 1
        y_spacing = abs(maxy - miny) / (y_steps - 1)
        Y = linspace(miny, maxy, y_steps)

        coords = list(product(X, Y))

        # sort probing coords in zig-zag to minimize path length
        sorted_coords = []
        for x in sorted(X):
            tmp = [point for point in coords if point[0] == x]
            sorted_coords.append(sorted(tmp, key=lambda point: point[1], reverse=len(sorted_coords) % 2 == 1))
        sorted_coords = [item for sublist in sorted_coords for item in sublist]
        self.probe_coords = sorted_coords
        self.X, self.Y = X, Y

        print('[I] Probing {:d} points, {:.5f}mm x-grid, {:.5f}mm y-grid:'.format(
            len(sorted_coords), x_spacing, y_spacing))
        # return the probing grid
        return sorted_coords

    def probe_grid(self):
        # probe the surface using the calculated grid
        self.probe_result = []
        for x, y in self.probe_coords:
            sys.stdout.write('[I] Probing x: {:.1f} y: {:.1f} '.format(x, y))
            sys.stdout.flush()

            # skip probing point X0 Y0 if exists
            if x == y == 0.:
                probe_point, probe_success = {'z': 0.}, True
            else:
                # raising probe Z to max_z
                self.send('G0Z{:.5f}'.format(self.max_z))
                # moving to next probe point
                self.send('G0X{:.5f}Y{:.5f}'.format(x, y))
                # do probe
                probe_point, probe_success = self.probe(self.min_z, self.feed_rate, retract=self.max_z)

            if not probe_success:
                print('\n[E] Unable to probe point!')
                self.return_home()
                sys.exit(1)

            now = datetime.fromtimestamp(int(time())).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
            result = {
                "sent": True,
                "done": True,
                "x": float(x),
                "y": float(y),
                "z": float(probe_point['z']),
                "ts": now,
                "xindx": int(where(self.X == x)[0][0]),
                "yindx": int(where(self.Y == y)[0][0]),
            }
            self.probe_result.append(result)
            print('z: {:.5f}'.format(result['z']))
        print('')

    def get_json(self):
        # return a json string with the probe result
        return json.dumps(self.probe_result)


def parse_args():
    # parse command line arguments
    parser = ArgumentParser(description='pcb surface autoprober')

    parser.add_argument('-i', metavar='INPUT_GCODE', dest='input_gcode',
                        help='input gcode for automatic surface probing', required=True)
    parser.add_argument('-o', dest='output_json', help='output JSON file containing probe points', required=True)
    parser.add_argument('-g', '--grid', metavar='mm', type=float, dest='grid_spacing',
                        help='probe grid spacing (mm)', required=True)

    parser.add_argument('-d', '--device', metavar='tty', dest='device', default='/dev/ttyUSB0', help='GRBL device')
    parser.add_argument('-f', '--feed', metavar='mm/min', type=int, dest='feed_rate',
                        default=5, help='probing feed rate on Z axis (default 5 mm/min)')
    parser.add_argument('--maxz', metavar='mm', type=float, dest='max_z',
                        default=.5, help='start probing at this Z axis value (default 0.5 mm)')
    parser.add_argument('--minz', metavar='mm', type=float, dest='min_z',
                        default=-.5, help='stop probing if Z axis reaches this value (default -0.5 mm)')
    parser.add_argument('--overscan', metavar='mm', type=float, default=1.0, dest='overscan',
                        help='probe grid overscan. the probe grid will be this value larger on every edge (mm)')

    args = parser.parse_args()

    assert args.max_z > args.min_z
    assert args.feed_rate > 0
    assert args.grid_spacing > 0

    return args


if __name__ == '__main__':
    args = parse_args()

    try:
        with open(args.input_gcode, 'rb') as input_f:
            input_gcode = input_f.read()
    except IOError:
        print('[E] Unable to open input file.')
        sys.exit(1)
    try:
        with open(args.output_json, 'ab') as output_f:
            pass
    except IOError:
        print('[E] Unable to write to output file.')
        sys.exit(1)

    prober = Probe(args.device, input_gcode, args.grid_spacing,
                   args.feed_rate, args.overscan, args.min_z, args.max_z)
    prober.get_probe_coords()
    if raw_input('[?] Do you want to probe the surface? [y/N] ') != 'y':
        sys.exit(0)
    prober.probe_origin()
    try:
        prober.probe_grid()
        with open(args.output_json, 'wb') as output_f:
            output_f.write(prober.get_json())
        print('\n[I] All done.')
    except KeyboardInterrupt:
        prober.get_pos()
    prober.return_home()
