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
@app.route('/api/image', methods=['GET'])
def send_file():
    # with open('image.png') as f:
    #     img = Image.open(f)

    # img_bytes = BytesIO()
    # img.save(img_bytes,format='PNG')
    # img_bytes = img_bytes.getvalue()

    # wait_for_file(filename)
    # return send_file('example.pdf',
    #                  attachment_filename='example-document.pdf',
    #                  mimetype='application/pdf',
    #                  as_attachment=True,
    #                  cache_timeout=0)
    return send_file ('image.png', 
                    # mimetype="image/png", 
                    download_name="image.png")

@app.route('/api/test', methods=['POST'])
def test():
    r = request
    # convert string of image data to uint8
    nparr = np.fromstring(r.data, np.uint8)
    # decode image
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    window_name = 'image'
    cv2.imshow(window_name, img)
    # do some fancy processing here....

    # build a response dict to send back to client
    response = {'message': 'image received. size={}x{}'.format(img.shape[1], img.shape[0])
                }
    # encode response using jsonpickle
    response_pickled = jsonpickle.encode(response)

    return Response(response=response_pickled, status=200, mimetype="application/json")


# start flask app
app.run(host="0.0.0.0", port=5030)