from __future__ import division
import sys
sys.path.append('../')

# import Flask functions
from flask import Flask, flash, redirect, render_template, request, session, \
				  abort, url_for, escape, jsonify
import os
import json
import time
import datetime

# Create Paths
device_file_path = os.path.dirname(os.path.abspath(__file__))
path, file = os.path.split(device_file_path)
device_file_path = os.path.join(path, 'config')
database_path = os.path.join(path, 'WebPage')

botIPS = ['192.168.1.105','192.168.1.103','192.168.1.108']

# Create Flask App
app = Flask(__name__)

# check if robots are Active or Inactive
def checkBotStatus(numRobot=9):
	# hostnames = ["192.168.1.10" + str(i) for i in range(1,numRobot+1)]
	hostnames = ["192.168.1.10" + str(i) for i in [3, 5, 8]]
	lut = {0:'Active', 256:'Inactive'}  
	botStatus = [os.system("ping -c 1 " + i) for i in hostnames]
	botStatus = [lut[i] for i in botStatus]
	return botStatus

# create file structure (start up files)
def createStartUPFiles(device_file_path=device_file_path):

	# create the gen_config json file if does not exist
	if os.path.exists(device_file_path + '/gen_config.json'):
		pass
	else:
		data = {"code":None}
		with open(device_file_path + '/gen_config.json', 'w') as f:
			json.dump(data, f)

	# create the cmd_vel json file if does not exist
	if os.path.exists(device_file_path + '/cmd_vel.json'):
		pass
	else:
		data = {"id":None, "vel": "0", "omega": "0"}
		with open(device_file_path + '/cmd_vel.json', 'w') as f:
			json.dump(data, f)

	# create the piston json file if does not exist
	if os.path.exists(device_file_path + '/piston.json'):
		pass
	else:
		data = {"status": "0", "ids": "[]", "start": "None"}
		with open(device_file_path + '/piston.json', 'w') as f:
			json.dump(data, f)

	# create the go to goal json file if does not exist
	if os.path.exists(device_file_path + '/g2g.json'):
		pass
	else:
		data = {"id": None, "start": "[]", "goal": "[]"}
		with open(device_file_path + '/g2g.json', 'w') as f:
			json.dump(data, f)

# intialize app
def init_app():
	createStartUPFiles(device_file_path)

# Route for Home page
@app.route('/')
def home():
	if not 'username' in session:
		flash('You are Logged Out')
		return render_template('home.html')
	else:
		return render_template('welcome.html', active_user=active_user)

# Route for the Welcome page
@app.route('/welcome')
def welcome():
	if not 'username' in session:
		flash('You are Logged Out')
		return render_template('home.html')
	else:
		# return render_template('welcome.html', active_user=active_user, \
								# botStatus=['192.168.1.109'], ips=botIPS)  
		return render_template('welcome.html', active_user=active_user, \
								botStatus=checkBotStatus(), ips=botIPS)  


# Route for controlling the speed of the robot
@app.route('/speed_control', methods=['GET','POST'])
def speed_control():
	if not 'username' in session:
		flash('You are Logged Out')
		return render_template('home.html')
	else:
		if request.method == 'POST':
			rid = request.form['rid']
			vel = request.form['vel']
			omega = request.form['omega']
			speed_control_config('id', rid)
			speed_control_config('vel', vel)
			speed_control_config('omega', omega)
			general_config('code', 'cmd_vel')
		return render_template('speed_control.html', active_user=active_user) 

# Route for controlling the piston on the robot
@app.route('/piston_control', methods=['GET','POST'])
def piston_control():
	if not 'username' in session:
		flash('You are Logged Out')
		return render_template('home.html')
	else:
		if request.method == 'POST':
			pstat = request.form['pstat']
			rid = request.form.getlist('ids')
			control_piston = "True"
			piston_control_config('status', pstat)
			piston_control_config('ids', map(int, rid))
			general_config('code', 'piston')
		return render_template('piston_control.html', active_user=active_user,\
								pis_status=['STOP', 'UP', 'DOWN']) 

# Route for Go to goal control on the robot
@app.route('/go_to_goal', methods=['GET','POST'])
def go_to_goal():
	if not 'username' in session:
		flash('You are Logged Out')
		return render_template('home.html')
	else:
		if request.method == 'POST':
			rid = request.form['rid']
			startx = request.form['startx']
			starty = request.form['starty']
			goalx = request.form['goalx']
			goaly = request.form['goaly']
			g2g_config('start', [float(startx), float(starty)])
			g2g_config('goal', [float(goalx), float(goaly)])
			g2g_config('id', int(rid))
			general_config('code', 'g2g')
		return render_template('go_to_goal.html', active_user=active_user) 

# Route for Signin Page
@app.route('/signin', methods=['GET', 'POST'])
def signin():
	global active_user
	error = None
	if request.method == 'POST':
		session['username'] = request.form['username']
		# check if user is authentic, Raise error if not
		if request.form['password'] != 'aarg' or request.form['username'] != 'aarg' :
			active_user = 'Invalid'
			error = 'Invalid Credentials. Please try again.'
		else:
			active_user = request.form['username']
			return redirect(url_for('welcome'))
	return render_template('signin.html', error=error)

# Route for SignOut Page
@app.route('/signout')
def signout():
	global active_user
	active_user = ""
	session.pop('username', None)
	return redirect(url_for('home'))

def speed_control_config(item, val):
	path = device_file_path + '/cmd_vel.json'
	with open(path, 'r') as file:
		data = json.load(file)
	data[item] = val
	with open(path, 'w') as file:
		json.dump(data, file)

def piston_control_config(item, val):
	path = device_file_path + '/piston.json'
	with open(path, 'r') as file:
		data = json.load(file)
	data[item] = val
	with open(path, 'w') as file:
		json.dump(data, file)

def g2g_config(item, val):
	path = device_file_path + '/g2g.json'
	with open(path, 'r') as file:
		data = json.load(file)
	data[item] = val
	with open(path, 'w') as file:
		json.dump(data, file)

def general_config(item, val):
	path = device_file_path + '/gen_config.json'
	with open(path, 'r') as file:
		data = json.load(file)
	data[item] = val
	with open(path, 'w') as file:
		json.dump(data, file)

def main():
	app.secret_key = os.urandom(12)
	app.run(debug=True)

if __name__ == '__main__':
	init_app()
	main()
