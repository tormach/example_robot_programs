import sh
import os

try:
    from twilio.rest import Client
except:
    sh.sudo.pip3.install("twilio")
    from twilio.rest import Client

# sign up for a Twillio account for the following ID and Token info
# the following three values must be filled in for this example to work
account_sid = ''
auth_token = ''
from_number=''

def send_sms_alert(alert, to, msg):
    try:
        client = Client(account_sid, auth_token)

        # prepend a + on the number if needed as twilio requires it
        if len(to) > 0 and to[0] != '+':
            to = '+' + to

        message = client.messages.create(to=to, from_=from_number, body=msg)
    finally:
        pass
