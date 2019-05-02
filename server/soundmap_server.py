#!flask/bin/python

import os
import re
import Time
from flask import Flask, request
from werkzeug.utils import secure_filename

UPLOAD_FOLDER = '../../../soundMap/uploads'
LATLON_FOLDER = '../../../soundMap/latlon'
LATLON_DEST = '../../../soundMap/latlonDest'
OTHER_LATLON = '../../../soundMap/otherLatlon'

ALLOWED_EXTENSIONS = set(['txt', 'wav', 'mp3', '3gp'])
INNACTIVE_THR = 300000  # ms --> Currently 15mins

users = {}

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['LATLON_FOLDER'] = LATLON_FOLDER
app.config['LATLON_DEST'] = LATLON_DEST
app.config['OTHER_LATLON'] = OTHER_LATLON


# Check for appropriate filetype
def allowed(filename):
    return '.' in filename and \
        filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


# Index #######################################################################
@app.route('/')
def index():
    print("The purpose of this URL is to serve the soundMap app")
    return "The purpose of this URL is to serve the soundMap app"


# /location ###################################################################
@app.route('/location', methods=['GET'])
def getTargetLocation():
    if request.method == 'GET':

        # Raw Request ---------------------------------------------------------
        print "\nHeaders"
        print request.headers

        # Parse Request -------------------------------------------------------
        if 'lat' not in request.headers or 'lng' not in request.headers:
            print("Please include 'lat' and 'lng' in your headers")
            return "Please include 'lat' and 'lng' in your headers"

        lat = request.headers['lat']
        lng = request.headers['lng']
        print("Lat: {} Lng: {}").format(lat, lng)

        if 'username' not in request.headers:
            print("Please include 'username' in your headers")
            return "Please include 'username' in your headers"

        uname = request.headers['username']
        print("Username: {}").format(uname)

        # Resolve Next Point --------------------------------------------------
        latlonPath = os.path.join(app.config['LATLON_FOLDER'], uname) + '/'
        print latlonPath

        if not os.path.exists(latlonPath):
            os.makedirs(latlonPath)

        latlonDestPath = os.path.join(app.config['LATLON_DEST'], uname) + '/'
        print latlonDestPath

        # If the mapping process is not finished, return wait signal
        if os.listdir(latlonPath) == []:
            return "Wait"
            print "Waiting for latlon file"

        if not os.path.exists(latlonDestPath):
            os.makedirs(latlonDestPath)

        fileNm = os.listdir(latlonPath)[0]
        f = open(latlonPath + fileNm, 'r')
        latlon = f.read()
        f.close()

        os.rename(os.path.join(latlonPath, fileNm),
                  os.path.join(latlonDestPath, fileNm))

        # Return Target Location ----------------------------------------------
        # Expects something like "Target:45.37364,-72.474"
        return "Target:" + latlon


# /users ######################################################################
@app.route('/users', methods=['GET'])
def getUsers():
    if request.method == 'GET':

        # Raw Request ---------------------------------------------------------
        print "\nHeaders"
        print request.headers

        # Parse Request -------------------------------------------------------
        if 'username' not in request.headers:
            print("Please include 'username' in your headers")
            return "Please include 'username' in your headers"

        uname = request.headers['username']
        print("Username: {}").format(uname)

        # Cull inactive users
        for user in users:
            if (int(Time.time() * 1000) - user['timestamp']) > INNACTIVE_THR:
                users.pop(user['uname'], None)

        # Format Response -----------------------------------------------------
        res = ""
        for user in users:
            if user['uname'] != uname:
                res = res + user['uname'] + ':' + user['coords'] + ';'

        # Return Response -----------------------------------------------------
        print('Response:\n{}').format(res)
        return res


# /upload #####################################################################
@app.route('/upload', methods=['GET', 'POST'])
def upload():
    if request.method == 'POST':

        # Raw Request ---------------------------------------------------------
        print "\nHeaders"
        print request.headers
        print "\nForm Elements"
        print request.form
        print "\nFiles"
        print request.files

        # Parse Request -------------------------------------------------------
        # If the audio portion of the request is missing, return
        if 'audio' not in request.files:
            print("Please include an audio file tagged 'audio'")
            return "Please include an audio file tagged 'audio'"

        file = request.files['audio']
        if file.filename == '':
            print("No filename selected")
            return "No filename selected"

        if 'username' not in request.form:
            print("Please include an audio file tagged 'username'")
            return "Please include an audio file tagged 'username'"

        uname = request.form['username']
        print('User: {}').format(uname)

        if 'location' not in request.form:
            print("Please include formatted coordinates as 'location'")
            return "Please include formatted coordinates as 'location'"

        lat = re.split(r'[(,)]', request.form['location'])[1]
        lon = re.split(r'[(,)]', request.form['location'])[2]
        print('Lat: {} , Lng: {} ').format(lat, lon)

        # Update Users --------------------------------------------------------
        # If user is in the active user list, update location and timestamp
        if users[uname]:
            users[uname]['coords'] = (lat + ',' + lon)
            users[uname]['timestamp'] = int(Time.time() * 1000)
        # If the user is not in the active user list, create a new user
        else:
            user = {}
            user['uname'] = uname
            user['coords'] = (lat + ',' + lon)
            user['timestamp'] = int(Time.time() * 1000)
            users[uname] = user

        # Write a file with neighbours information
        othrLatlonPath = os.path.join(app.config['OTHER_LATLON'], uname) + '/'
        if not os.path.exists(othrLatlonPath):
            os.makedirs(othrLatlonPath)

        fNm = os.path.join(othrLatlonPath,str(int(time.time()))+"_neighbor.txt")
        f = open(fileNm, 'w')
        for i in users:
            if i['uname'] != uname:
                f.write(i['coords']+"\n")
        f.close()

        # Parse Uploaded File -------------------------------------------------
        if allowed(file.filename):
            filename = secure_filename(file.filename)

            # Changing the file name to include latlon and username
            if not os.path.exists(os.path.join(app.config['UPLOAD_FOLDER'],
                                               uname)):
                os.makedirs(os.path.join(app.config['UPLOAD_FOLDER'], uname))

            fn = uname + "_" + lat + "_" + lon + "_" + filename.split("_")[-1]
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], uname, fn))

            # Convert to Mp3
#            cmd = 'ffmpeg -i ' + UPLOAD_FOLDER + '/' + filename +
#            ' -c:a libmp3lame ' + UPLOAD_FOLDER + '/' + filename + '.mp3'
#            print cmd
#            subprocess.Popen(cmd)

            # Return Response -------------------------------------------------
            print("Successful upload")
            return "Successful upload"
        else:
            print("This file type is not allowed")
            return "This file type is not allowed"

    else:
        print("Not yet implemented. Did you mean to POST?")
        return "Not yet implemented. Did you mean to POST?"


if __name__ == '__main__':
    app.run(host='192.168.0.100', port='5000', debug=False)
