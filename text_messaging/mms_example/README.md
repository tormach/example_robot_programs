# Text Messaging Example
This program contains an example of sending a photo taken from any attached USB camera or webcam via text message from the robot.  It requires a Twilio account, a Dropbox account (to host the image file and create a URL link for the multi-media text message).


This example is discussed on the Tormach user forums here: https://forums.tormach.com/t/example-program-taking-a-photo-with-a-webcam-and-sending-it-via-text-message/39

## Prerequisites
To run this example code you will need to sign up for an account with Twilio.  New accounts get a $10 credit, no CC information required as of 8/2021.  You must get from Twilio the following information:
1. Account SID
2. Authorization Token
3. "From" phone number

Then fill in that information in the following fields in mms.py:
account_sid = ''
auth_token = ''
from_number=''

You will also need to create a Dropbox API key, reasonably complete instructions here: http://99rabbits.com/get-dropbox-access-token/


NB: all phone numbers should be formatted like this:
'+6085551212'.  Lack of a leading + sign, or use of spaces, parentheses, or hyphens will not work.

## Libraries Used
Twilio: https://www.twilio.com/docs/libraries/python

Dropbox: https://www.dropbox.com/developers/documentation/http/documentation

OpenCV: https://pypi.org/project/opencv-python/
## Credit
This example was inspired by Matthew Olan's smart security camera project: https://github.com/mattolan/RT_OD 
