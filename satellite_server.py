import atexit
import json
from time import gmtime, strftime

import numpy as np
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.interval import IntervalTrigger
from flask import Flask

import satellite_api

app = Flask(__name__, static_url_path='/static')

@app.route("/")
def index():
    return app.send_static_file('dash.html')

@app.route("/data/", methods=['GET'])
def get_data():

    data = []

    for channel in range(0,11):
        data.append(list(satellite.data[channel]))

    data = json.dumps(
        {'ax': list(data[0]),
         'ay': list(data[1]),
         'az': list(data[2]),
         'gx': list(data[3]),
         'gy': list(data[4]),
         'gz': list(data[5]),
         'temp': list(data[6]),
         'hum': list(data[7]),
         'light': list(data[8]),
         'rad_bool': list(data[9]),
         'rad': list(data[10])
         },
        sort_keys=False,
        indent=4,
        separators=(',', ': '))

    return data

# Updates EMG data in realtime
def data_realtime_update():
    satellite.read_packs()


scheduler = BackgroundScheduler()
scheduler.start()
scheduler.add_job(
    func=data_realtime_update,
    trigger=IntervalTrigger(seconds=0.5),
    replace_existing=True)
# Shut down the scheduler when exiting the app
atexit.register(lambda: scheduler.shutdown())

if __name__ == '__main__':
    try:
        satellite = satellite_api.Satellite('/dev/cu.usbmodem14101', numread=1, plotting=True, packsize=54)
        connection = satellite.establish_connection()
    except Exception as e:
        print('Connection to arduino failed: {}'.format(e))
    print('Flask server started')
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False, threaded=True)