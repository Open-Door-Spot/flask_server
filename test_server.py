from flask import Flask, request, Response, send_file
import jsonpickle
import numpy as np
import cv2
from io import BytesIO
# from Flask import send_file
from PIL import Image
import os
import time

# Initialize the Flask application
app = Flask(__name__)

# route http posts to this method
# 
@app.route('/api/image', methods=['GET'])
def file_sender():

    return send_file("./sidebyside.jpg", mimetype='image/jpg')


@app.route('/api/params', methods=['POST'])
def get_params():
		pixel_x = request.form.get('pixel_x')
		pixel_y = request.form.get('pixel_y')
		print(pixel_x + ", " + pixel_y)
		return "we are chilling"

# start flask app
app.run(host="0.0.0.0", port=5030)