#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from os import times
import sys

'''
Helper script to parse and analyze tcpdump logs from MQTT messages
'''


def parseLine(line: str):
    try:
        parts = line.split()
        timestamp = parts[0]
        times = [float(a) for a in timestamp.split(':')]

        return {
            'seconds': times[0]*3600 + times[1]*60 + times[2],
            'origin': parts[2],
            'size': int(parts[-1])
        }
    except:
        return None


def analyze_stream(values):
    min_time = values[0]['seconds']
    max_time = values[-1]['seconds']
    time_span = max_time - min_time

    total_data = sum(map(lambda x: x['size'], values))

    count = len(values)
    data_rate = total_data / time_span
    frequency = count / time_span
    avg_size = total_data / count

    return {
        'origin': values[0]['origin'],
        'time_span': time_span,
        'total_data': total_data,
        'count': count,
        'data_rate': data_rate,
        'freq': frequency,
        'avg_size': avg_size
    }


file_name = sys.argv[1]

lines = []

with open(file_name) as f:
    lines = f.readlines()

data = filter(lambda a: a is not None, map(parseLine, lines))

streams = {}
for d in data:
    if d['origin'] in streams:
        streams[d['origin']].append(d)
    else:
        streams[d['origin']] = [d]

analytics = list(map(analyze_stream, streams.values()))

print(analytics)

totals = {
    'total_data': sum(map(lambda x: x['total_data'], analytics)),
    'total_count': sum(map(lambda x: x['count'], analytics)),
    'data_rate': sum(map(lambda x: x['data_rate'], analytics)),
    'freq': sum(map(lambda x: x['freq'], analytics))
}

print(totals)
