'''
Copyright 2022 Leonardo Cabral

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import importlib
import json
from Modules.TCL_Experiment import Experiment
from Modules.TCL_CommunicationProtocol import searchTeCoLabPort, readTemperatures, writePWMs
from Modules.TCL_CommandLineArguments import getParameters

args = getParameters()

settings_file = args.SettingsFile
settings = None

try:
	with open(f'Settings/{settings_file}.json', 'r', encoding='utf-8') as json_file:
		settings = json.loads(json_file.read())
except FileNotFoundError:
	print(f"The file {settings_file} doesn't exist.")
except Exception as e:
	print(f"An error occurred: {e}")

experiment_file_path = f'Experiments/{settings["experiment"]}.csv'
experiment_info_path = f'Experiments/{settings["experiment"]}.txt'
control_file_path = f'Controllers.{settings["controller"]}'
control_module = importlib.import_module(control_file_path)
parameters = settings['parameters']

## Search for a TeCoLab device
tecolab = searchTeCoLabPort()
if tecolab is False:
	print("No TeCoLab device found. Terminating program.")
	exit()

## Load the selected experiment
print("Loading experiment: " + settings["experiment"])
exp = Experiment(experiment_file_path)
expDescription = open(experiment_info_path, 'r', encoding='utf-8')
print(expDescription.read())
print('Experiment table:')
print(exp.expTable)

cont = control_module.Controller(**parameters)

## Execute experiment
exp.setInitialTime()

while exp.run_flag == True:
	if exp.iterationControl() is True:
		# Reads temperatures
		exp.setTemperatures(readTemperatures(tecolab))

		# Get control action
		exp.controlAction = cont.run_control(exp.getSetPoints(), exp.getTemperatures())

		# Adds experiment disturbances
		exp.applyDisturbances()

		# Applies to the board
		writePWMs(tecolab, exp.getDisturbedControlAction())

		# Logs the information
		exp.log()
