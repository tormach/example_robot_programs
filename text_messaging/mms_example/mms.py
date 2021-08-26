# imports
# re and datetime live on the base OS image, but twilio, dropbox, and cv2 must be installed every time a new docker container is started
# this may take a minute or two the first time you run this program after restarting your computer
import sh
import re
import datetime
from robot_command.rpl import *

# Setup required: Enter the account SID, auth token, and 'from' phone number from your Twilio account here:
account_sid = ""
auth_token = ""
from_phone_number = "+"

# Enter your 'to' (destination) phone number here:
to_phone_number = '+16088498381'

# Enter your Dropbox API Key here:
dropbox_api_key = ""

try:
    from twilio.rest import Client
except:
    notify("The first time this program is run it must import three packages (Twilio, Dropbox, OpenCV).  "
    "This may take a minute or two depending on your internet connection.  Subsequent program runs will not experience this delay")
    sh.sudo.pip3.install("twilio")
    from twilio.rest import Client

try:
    import dropbox
except:
    sh.sudo.pip3.install("dropbox")
    import dropbox

try:
    import cv2
except:
    sh.sudo.pip3.install("opencv-python")
    import cv2

def take_and_send_image(filename=None):
    if filename == None:
        filename = "robot_photo.jpg"
    take_snapshot(filename)
    url = upload_file_to_dropbox(filename)
    send_mms(url)

def take_snapshot(filename):
    videoCaptureObject = cv2.VideoCapture(0)
    result = True
    while (result):
        ret, frame = videoCaptureObject.read()
        cv2.imwrite(filename, frame)
        result = False
    videoCaptureObject.release()
    cv2.destroyAllWindows()


def upload_file_to_dropbox(filename):
    # FIXME - look for an intelligent path, not CWD
    # add timestamp
    newfilename = "/robot_images/" + str(datetime.datetime.today().strftime('%d-%m-%Y-%H-%M-%S')) + "_" + filename
    file_to_upload = newfilename

    print("Begin uploading " + file_to_upload + " to DropBox")

    # Create a dropbox object using an API v2 key
    d = dropbox.Dropbox(dropbox_api_key)

    # open the file and upload it
    with open(filename, "rb") as f:
        d.files_upload(f.read(), file_to_upload)

    # create a shared link
    link = d.sharing_create_shared_link(file_to_upload)

    # url which can be shared
    url = link.url

    # link which directly downloads by replacing ?dl=0 with ?dl=1
    dl_url = re.sub(r"\?dl\=0", "?raw=1", url)

    print (dl_url)
    return dl_url


def send_mms(url):
    client = Client(account_sid, auth_token)
    message = client.messages \
        .create(
             body='Here is a message from your ZA6 robot',
             from_= from_phone_number,
             media_url=[url],
             to= to_phone_number
         )

    print(message.sid)