# flask_server

To trigger the Python Spot SDK from triggering a button in VR, follow these steps:

- Add an endpoint in the Flask server like `app.route` and trigger a python Spot SDK command
- Invoke that endpoint from within a game object through the C# script associated with that object
- Connect to RLAB WiFi
- Run ESTOP:
  - Open rosetta terminal
  - source my_spot_env/bin/activate
  - cd ~/spot-sdk/python/examples/estop estop
  - export BOSDYN_CLIENT_USERNAME=user
  - export BOSDYN_CLIENT_PASSWORD=bigxxxxxxxxxxxba
  - arch -arm64 python3 estop_nogui.py

- In a new terminal, run the flask server
  - Depending on which Spot we are using, change the IP address on the server
  - source my_spot_env/bin/activate
  - export BOSDYN_CLIENT_USERNAME=user
  - export BOSDYN_CLIENT_PASSWORD=bigxxxxxxxxxxxba
  - arch -arm64 python3 main.py
  
 - Other notes
    - IMP: Undock Spot first if the code involves making Spot sit
    - If you use the tablet to control Spot, first restart Spot before running the Flask server to claim the Spot lease
