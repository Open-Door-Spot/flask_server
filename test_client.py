from __future__ import print_function
import requests
import json
import cv2
import numpy as np
from PIL import Image
from io import BytesIO
addr = 'http://localhost:5030'
test_url = addr + '/api/image'

# prepare headers for http request
content_type = 'image/png'
# headers = {'content-type': content_type}

# img = cv2.imread('image.png')
# # encode image as jpeg
# _, img_encoded = cv2.imencode('image.png', img)
# # send http request with image and receive response
# response = requests.post(test_url, data=img_encoded.tobytes(), headers=headers)
# # decode response
# print(json.loads(response.text))

response = requests.get(test_url)
print(response.content)
print(type(response.content))
img = Image.open(BytesIO(response.content))
img.show()