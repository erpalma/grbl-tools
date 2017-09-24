#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import numpy as np
import os
import re
import sys

from argparse import ArgumentParser
from datetime import datetime
from itertools import product
from scipy.interpolate import griddata
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
        self.z_max_travel = 40
        self.x_coords_re = re.compile(r'X\s*(-?[0-9]+(?:\.[0-9]+)?)')
        self.y_coords_re = re.compile(r'Y\s*(-?[0-9]+(?:\.[0-9]+)?)')
        self.mpos_re = re.compile(r'\|MPos:(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+)')
        self.probe_re = re.compile(r'\[PRB:(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+):([0-1])\]')

    def init_grbl(self):
        # open serial port and wait for welcome msg
        self.ser = Serial(self.device, 115200, timeout=self.ser_timeout)
        data = ''
        while "Grbl 1.1f ['$' for help]" != data:
            data = self.ser.readline().strip()
        self.ser.timeout = 1
        if '''[MSG:'$H'|'$X' to unlock]''' in self.ser.readline().strip():
            self.send('$X', wait_for_idle=False)
        self.ser.reset_input_buffer()
        self.ser.timeout = self.ser_timeout
        # set millimeter mode
        self.send('G21')
        # set adbsolute coords
        self.send('G90')
        # reset work coords
        self.send('G92X0Y0Z0')
        # set local relative offset
        self.zero_wpos = self.get_abs_pos()

    def send(self, data, newline=True, wait_for_idle=True):
        # open serial only on first send
        if self.ser is None:
            self.init_grbl()
        # wait for machine to be idle (not moving)
        if wait_for_idle:
            while True:
                self.ser.write('?')
                if '<Idle|' in self.ser.readline():
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
                resp[coord] = - self.zero_wpos[coord] + coords[coord]
        return resp

    def get_abs_pos(self):
        # wait for machine to be idle
        while True:
            mpos = self.send('?', newline=False)
            if '<Idle|' in mpos:
                break
            sleep(.25)
        mpos = tuple(map(float, self.mpos_re.findall(mpos)[0]))
        return {'x': mpos[0], 'y': mpos[1], 'z': mpos[2]}

    def get_pos(self):
        # get current position in relative coords
        return self.get_rel_coord(self.get_abs_pos())

    def probe(self, min_z, feed_rate, retract=None, zero_coords=False):
        assert(min_z < 0)
        assert(retract is None or retract >= 0)
        resp = self.send('G38.3 Z{:.5f} F{:.0f}'.format(min_z, feed_rate))
        resp = self.probe_re.findall(resp)[0]
        probe_point, probe_success = tuple(map(float, resp[:3])), bool(resp[-1])
        # zero out work coords
        if probe_success and zero_coords:
            # zero out work offset
            self.send('G92Z{:.5f}'.format(self.get_abs_pos()['z'] - probe_point[2]))
            # go to effective zero since probe might have stopped after
            # the probe touchdown (due to deceleration)
            self.send('G01Z0F1')
            # set new local relative offset
            self.zero_wpos = self.get_abs_pos()
        if retract is not None:
            self.send('G0Z{:.5f}'.format(retract))
        probe_point = {'x': probe_point[0], 'y': probe_point[1], 'z': 0. if zero_coords else probe_point[2]}
        return self.get_rel_coord(probe_point), probe_success

    def probe_origin(self):
        sys.stdout.write('\n[I] Zeroing Z in origin using coarse mode (F{:.0f})... '.format(self.coarse_feed_probe))
        sys.stdout.flush()

        # raise Z axis a bit to avoid potential alarm
        self.send('G0Z1')
        if not self.probe(-self.z_max_travel, self.coarse_feed_probe, zero_coords=True)[1]:
            print('\n\n[E] Probe error!')
            sys.exit(1)
        self.send('G1Z.1F1')
        sys.stdout.write('Done.\n[I] Zeroing Z in origin using fine mode (F{:.0f})... '.format(self.fine_feed_probe))
        sys.stdout.flush()
        if not self.probe(-.4, self.fine_feed_probe, zero_coords=True)[1]:
            print('\n\n[E] Probe error!')
            sys.exit(1)
        print('Done.')

    def return_home(self):
        print('\n[I] Returning home. X0 Y0 Z0.2')
        self.send('G0Z5')
        self.send('G0X0Y0')
        self.send('G0Z.5')
        self.send('G1Z.2F10')

    def get_workspace_size(self):
        # get all X and Y coords in the gcode file
        X = np.asarray(self.x_coords_re.findall(self.input_gcode), np.double)
        Y = np.asarray(self.y_coords_re.findall(self.input_gcode), np.double)
        # find boundaries
        return min(X), max(X), min(Y), max(Y)

    def get_probe_coords(self):
        minx, maxx, miny, maxy = self.get_workspace_size()
        print('\n[I] Gcode area (WxH): {:.2f}mm x {:.2f}mm'.format(abs(maxx - minx), abs(maxy - miny)))
        if self.overscan != 0:
            minx, maxx = minx - self.overscan, maxx + self.overscan
            miny, maxy = miny - self.overscan, maxy + self.overscan
            print('[I] Probe area with overscan (WxH): {:.2f}mm x {:.2f}mm'.format(abs(maxx - minx), abs(maxy - miny)))

        x_steps = max(2, int(round(abs(maxx - minx) / self.grid_spacing)) + 1)
        x_spacing = abs(maxx - minx) / (x_steps - 1)
        X = np.linspace(minx, maxx, x_steps)

        y_steps = max(2, int(round(abs(maxy - miny) / self.grid_spacing)) + 1)
        y_spacing = abs(maxy - miny) / (y_steps - 1)
        Y = np.linspace(miny, maxy, y_steps)

        coords = tuple(product(X, Y))

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
        for i, (x, y) in enumerate(self.probe_coords):
            sys.stdout.write('[{:03d}] Probing x: {:.1f} y: {:.1f} '.format(i + 1, x, y))
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
                "xindx": int(np.where(self.X == x)[0][0]),
                "yindx": int(np.where(self.Y == y)[0][0]),
            }
            self.probe_result.append(result)
            print('z: {:.5f}'.format(result['z']))
        print('')

    def get_json(self):
        # return a json string with the probe result
        return json.dumps(self.probe_result)


def correct_gcode(input_gcode, probe_json):
    probe_json = json.loads(probe_json)

    X = np.asarray([point['x'] for point in probe_json], np.double)
    Y = np.asarray([point['y'] for point in probe_json], np.double)
    points = np.vstack((X, Y)).T
    values = np.asarray([point['z'] for point in probe_json], np.double)

    regexps = {
        'x': re.compile(r'x\s*(-?[0-9]+\.[0-9]+)', re.IGNORECASE),
        'y': re.compile(r'y\s*(-?[0-9]+\.[0-9]+)', re.IGNORECASE),
        'z': re.compile(r'z\s*(-?[0-9]+\.[0-9]+)', re.IGNORECASE),
    }

    # split input gcode by line, filtering empty lines
    input_gcode = list(filter(lambda x: x, map(lambda x: x.strip(), input_gcode.split('\n'))))
    result = []
    cur_coords = [0] * 3
    for i, line in enumerate(input_gcode):
        # skip comments
        if line.startswith(';') or line.startswith('('):
            continue

        cur_line = ''
        # update current gcode coordinates
        for j, coord in enumerate(('x', 'y', 'z')):
            match = regexps[coord].search(line)
            if match:
                cur_coords[j] = float(match.group(1))
                # keep track of which coordinate we have found in this gcode line
                cur_line += coord

        # if this gcode line contains a Z coord, correct it
        if 'z' in cur_line:
            result.append((i, 'sub', cur_coords[:]))
        # no Z coord in this line, let's add it
        elif 'x' in cur_line or 'y' in cur_line:
            result.append((i, 'append', cur_coords[:]))
    # points that we need to adjust (x,y,z)
    gcode_points = np.vstack(zip(*[item[2] for item in result])).T

    # calculate new Z value for each point in gcode_points using both linear and nearest interpolation
    newZval_lin = griddata(points, values, gcode_points[:, :2], method='linear') + gcode_points[:, 2]
    newZval_near = griddata(points, values, gcode_points[:, :2], method='nearest') + gcode_points[:, 2]
    for i, newZval in enumerate(newZval_lin):
        j, action = result[i][:2]
        # if the new Z value is nan, than the point is probably outside the probing grid
        # we use the nearest point as an approximation
        if np.isnan(newZval):
            newZval = newZval_near[i]
        # replace or add the new Z value
        if action == 'sub':
            input_gcode[j] = regexps['z'].sub('Z{:.5f}'.format(newZval), input_gcode[j])
        else:
            input_gcode[j] += ' Z{:.5f}'.format(newZval)

    return '\n'.join(input_gcode).encode('ascii')


def parse_args():
    # parse command line arguments
    parser = ArgumentParser(description='pcb surface autoprober')
    subparsers = parser.add_subparsers(title='actions')

    probe_parsers = subparsers.add_parser('probe', help='probe the surface and generate JSON report')
    probe_parsers.set_defaults(which='probe')
    probe_parsers.add_argument('-i', metavar='INPUT_GCODE', dest='input_gcode',
                               help='input gcode for automatic surface probing', required=True)
    probe_parsers.add_argument('-l', dest='output',
                               help='output JSON file containing probe points', required=True)
    probe_parsers.add_argument('-g', '--grid', metavar='mm', type=float, dest='grid_spacing',
                               help='probe grid spacing (mm)', required=True)
    probe_parsers.add_argument('-d', '--device', metavar='serial_device', dest='device',
                               default='/dev/ttyUSB0', help='GRBL device')
    probe_parsers.add_argument('-f', '--feed', metavar='mm/min', type=int, dest='feed_rate',
                               default=5, help='probing feed rate on Z axis (default 5 mm/min)')
    probe_parsers.add_argument('--maxz', metavar='mm', type=float, dest='max_z',
                               default=.5, help='start probing at this Z axis value (default 0.5 mm)')
    probe_parsers.add_argument('--minz', metavar='mm', type=float, dest='min_z',
                               default=-.5, help='stop probing if Z axis reaches this value (default -0.5 mm)')
    probe_parsers.add_argument('--overscan', metavar='mm', type=float, default=1.0, dest='overscan',
                               help='probe grid overscan. the probe grid will be this value larger on every edge (mm)')

    correct_parsers = subparsers.add_parser('correct', help='correct the input gcode with the probing result')
    correct_parsers.set_defaults(which='correct')
    correct_parsers.add_argument(metavar='INPUT_GCODE', dest='input_gcode',
                                 help='input gcode file to be corrected', nargs='+')
    # correct_parsers.add_argument('-o', metavar='OUTPUT_GCODE', dest='output',
    #                              help='corrected output gcode file (default to lvl_<input_gcode_name>)')
    correct_parsers.add_argument('-l', dest='input_json',
                                 help='input JSON file containing probe points', required=True)

    args = parser.parse_args()

    if args.which == 'probe':
        assert args.max_z > args.min_z
        assert args.feed_rate > 0
        assert args.grid_spacing > 0

    return args


if __name__ == '__main__':
    args = parse_args()

    if args.which in ['probe', 'correct']:
        try:
            with open(args.input_gcode, 'rb') as input_f:
                input_gcode = input_f.read().decode('utf-8')
        except IOError:
            print('[E] Unable to open input file.')
            sys.exit(1)

        if not args.output:
            dirname = os.path.dirname(args.input_gcode)
            filename = os.path.basename(args.input_gcode)
            args.output = os.path.join(dirname, 'lvl_{}'.format(filename))
        try:
            with open(args.output, 'ab') as output_f:
                pass
        except IOError:
            print('[E] Unable to write to output file.')
            sys.exit(1)

    if args.which == 'probe':
        try:
            with open(args.input_gcode, 'rb') as input_f:
                input_gcode = input_f.read().decode('utf-8')
        except IOError:
            print('[E] Unable to open input file.')
            sys.exit(1)

        try:
            with open(args.output, 'ab') as output_f:
                pass
        except IOError:
            print('[E] Unable to write to output file.')
            sys.exit(1)

        prober = Probe(args.device, input_gcode, args.grid_spacing,
                       args.feed_rate, args.overscan, args.min_z, args.max_z)
        prober.get_probe_coords()
        # python 2/3 compatibility
        _input = getattr(__builtins__, 'raw_input', input)
        if _input('[?] Do you want to probe the surface? [y/N] ') != 'y':
            sys.exit(0)
        prober.probe_origin()
        try:
            prober.probe_grid()
            with open(args.output, 'wb') as output_f:
                output_f.write(prober.get_json())
            print('\n[I] All done.')
        except KeyboardInterrupt:
            prober.get_pos()
        prober.return_home()

    elif args.which == 'correct':
        try:
            with open(args.input_json, 'rb') as input_f:
                input_json = input_f.read().decode('utf-8')
        except IOError:
            print('[E] Unable to open JSON file.')
            sys.exit(1)

        for fname in args.input_gcode:
            try:
                with open(fname, 'rb') as input_f:
                    input_gcode = input_f.read().decode('utf-8')
            except IOError:
                print('[E] Unable to open input file.')
                sys.exit(1)

            output_gcode = correct_gcode(input_gcode, input_json)
            dirname = os.path.dirname(fname)
            filename = os.path.basename(fname)
            try:
                output = os.path.join(dirname, 'lvl_{}'.format(filename))
                with open(output, 'wb') as output_f:
                    output_f.write(output_gcode)
            except IOError:
                print('[E] Unable to write to output file.')
                sys.exit(1)

        print('[I] All done.')
