# ####################################
# Generate QR Code
import pyqrcode

#qr = pyqrcode.create('hello')
#qr.png('abc.png', scale=8)

# ####################################
# Read QR Code
from pyzbar.pyzbar import decode
from PIL import Image

d = decode(Image.open('abc.png'))

print(d[0].data.decode('ascii'))