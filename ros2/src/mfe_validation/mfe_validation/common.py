"""Helpers shared by the validation nodes."""
import csv


def read_track(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def car_start(rows):
    """(x, y, yaw) of the car_start row of a track CSV, or None."""
    found = None
    for r in rows:
        if r['tag'] == 'car_start':
            found = (float(r['x']), float(r['y']), float(r['direction']))
    return found
