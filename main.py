import os
from flask import Flask, request

# command: python3 main.py
from commands.hello_spot import hello_spot_main
app = Flask("Spot Controller")

# Change this to the correct IP address
SPOT_IP_ADDRESS = "gouger.rlab.cs.brown.edu"

@app.route("/")
def return_valid_response():
	return "The spot controller server is working correctly."

@app.route("/hello_spot")
def invoke_hello_spot():
	print("Hello Spot Command is Triggered")
	os.system("arch -arm64 python3 commands/hello_spot.py "+SPOT_IP_ADDRESS)
	return "its done"

if __name__ == "__main__":
	app.run(host=os.getenv('IP', '0.0.0.0'), port=int(os.getenv('PORT', 8888)))
